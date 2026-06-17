#include "carla_world.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <set>
#include <stdexcept>
#include <vector>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/Vehicle.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/rpc/EpisodeSettings.h>
#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>
#include <carla/sensor/data/Image.h>

#include <opencv2/core.hpp>

#include "conversions.hpp"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
using namespace std::chrono_literals;

namespace carla_cpp {

struct CarlaWorld::Impl {
  Impl(const std::string &host, uint16_t port, FrameCollector &collector)
      : client(host, port), collector(collector) {
    client.SetTimeout(20s);
  }

  cc::Client client;
  FrameCollector &collector;
  std::unique_ptr<cc::World> world;

  carla::SharedPtr<cc::Vehicle> vehicle;
  carla::SharedPtr<cc::Sensor> cam_left, cam_right, imu, gnss;
};

CarlaWorld::CarlaWorld(const std::string &host, uint16_t port,
                       FrameCollector &collector)
    : p_(new Impl(host, port, collector)) {}

CarlaWorld::~CarlaWorld() {
  try {
    shutdown();
  } catch (...) {
  }
}

void CarlaWorld::setup(double fixed_delta_seconds, const SpawnPose &ego,
                       const std::string &town) {
  printf("[carla_cpp] client %s / server %s\n",
         p_->client.GetClientVersion().c_str(),
         p_->client.GetServerVersion().c_str());

  // Load the requested town only if the server isn't already on it (a reload is
  // slow and resets all settings). The Python bridge did this via `town:=...`;
  // GetWorld() alone would just attach to whatever map is currently loaded.
  auto world = p_->client.GetWorld();
  if (!town.empty()) {
    const std::string cur = world.GetMap()->GetName();  // e.g. "Carla/Maps/Town10HD"
    if (cur != town && cur != ("Carla/Maps/" + town)) {
      printf("[carla_cpp] loading map '%s' (was '%s')...\n", town.c_str(),
             cur.c_str());
      world = p_->client.LoadWorld(town);
    } else {
      printf("[carla_cpp] map already '%s'\n", cur.c_str());
    }
  }
  p_->world.reset(new cc::World(std::move(world)));

  // Synchronous mode: this client owns time and drives it via tick().
  auto settings = p_->world->GetSettings();
  settings.synchronous_mode = true;
  settings.fixed_delta_seconds = fixed_delta_seconds;
  settings.substepping = true;
  p_->world->ApplySettings(settings, 10s);

  auto bp_lib = p_->world->GetBlueprintLibrary();

  // ---- ego vehicle ----
  auto ego_bp = *bp_lib->Find("vehicle.tesla.model3");
  // role_name matches the carla_ros_bridge convention so external tooling
  // (e.g. the visualizer's ground-truth poller) can find the ego actor.
  if (ego_bp.ContainsAttribute("role_name"))
    ego_bp.SetAttribute("role_name", "ego_vehicle");
  cg::Transform ego_tf(cg::Location(ego.x, ego.y, ego.z),
                       cg::Rotation(ego.pitch, ego.yaw, ego.roll));
  auto ego_actor = p_->world->SpawnActor(ego_bp, ego_tf);
  p_->vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
  printf("[carla_cpp] spawned %s\n", ego_actor->GetDisplayId().c_str());

  // ---- stereo cameras (objects.json geometry) ----
  auto make_camera = [&](const char *id, float y, bool left) {
    auto bp = *bp_lib->Find("sensor.camera.rgb");
    bp.SetAttribute("image_size_x", "960");
    bp.SetAttribute("image_size_y", "720");
    bp.SetAttribute("fov", "90.0");
    bp.SetAttribute("sensor_tick", "0.05");   // 20 Hz cameras (every 10 ticks @ 200 Hz world)
    cg::Transform tf(cg::Location(1.5f, y, 1.5f), cg::Rotation(0.f, 0.f, 0.f));
    auto actor = p_->world->SpawnActor(bp, tf, ego_actor.get());
    auto sensor = boost::static_pointer_cast<cc::Sensor>(actor);
    sensor->Listen([this, left](auto data) {
      auto img = boost::static_pointer_cast<csd::Image>(data);
      // Wrap the CARLA BGRA buffer (no copy) then deep-copy via clone() so
      // the cv::Mat stays valid after this callback returns.
      cv::Mat bgra(static_cast<int>(img->GetHeight()),
                   static_cast<int>(img->GetWidth()), CV_8UC4,
                   const_cast<void *>(static_cast<const void *>(img->data())));
      p_->collector.addImage(img->GetFrame(), img->GetTimestamp(), left,
                             bgra.clone());
    });
    (void)id;
    return sensor;
  };
  // CARLA frame: +y = RIGHT of the vehicle. So the physical-left camera sits at
  // y=-0.25 and the physical-right at y=+0.25. (Earlier these were swapped, which
  // crossed left<->right -> negative disparity -> stereo depth runaway/divergence.)
  p_->cam_left = make_camera("cam_front_left", -0.25f, true);
  p_->cam_right = make_camera("cam_front_right", 0.25f, false);

  // ---- IMU (every tick) ----
  {
    auto bp = *bp_lib->Find("sensor.other.imu");
    bp.SetAttribute("sensor_tick", "0.005");   // 200 Hz IMU (every tick @ 200 Hz world)
    cg::Transform tf(cg::Location(0.f, 0.f, 1.5f), cg::Rotation(0.f, 0.f, 0.f));
    auto actor = p_->world->SpawnActor(bp, tf, ego_actor.get());
    p_->imu = boost::static_pointer_cast<cc::Sensor>(actor);
    p_->imu->Listen([this](auto data) {
      auto m = boost::static_pointer_cast<csd::IMUMeasurement>(data);
      auto a = m->GetAccelerometer();
      auto g = m->GetGyroscope();
      p_->collector.addImu(m->GetFrame(), m->GetTimestamp(),
                           carlaAccelToVins(a.x, a.y, a.z),
                           carlaGyroToVins(g.x, g.y, g.z));
    });
  }

  // ---- GNSS (for global_fusion + rtabmap GPS) ----
  {
    auto bp = *bp_lib->Find("sensor.other.gnss");
    bp.SetAttribute("sensor_tick", "0.1");
    cg::Transform tf(cg::Location(0.f, 0.f, 1.5f), cg::Rotation(0.f, 0.f, 0.f));
    auto actor = p_->world->SpawnActor(bp, tf, ego_actor.get());
    p_->gnss = boost::static_pointer_cast<cc::Sensor>(actor);
    p_->gnss->Listen([this](auto data) {
      auto m = boost::static_pointer_cast<csd::GnssMeasurement>(data);
      p_->collector.addGnss(m->GetFrame(), m->GetTimestamp(), m->GetLatitude(),
                            m->GetLongitude(), m->GetAltitude());
    });
  }

  printf("[carla_cpp] sensors spawned; ticking at %.4fs\n",
         fixed_delta_seconds);
}

uint64_t CarlaWorld::tick(int timeout_ms) {
  return p_->world->Tick(std::chrono::milliseconds(timeout_ms));
}

void CarlaWorld::applyControl(const ControlCmd &c) {
  cc::Vehicle::Control control;
  control.throttle = c.throttle;
  control.steer = c.steer;
  control.brake = c.brake;
  control.reverse = c.reverse;
  control.hand_brake = c.hand_brake;
  p_->vehicle->ApplyControl(control);
}

bool CarlaWorld::getEgoState(double &x, double &y, double &yaw, double &speed) const {
  if (!p_->vehicle) return false;
  auto tf = p_->vehicle->GetTransform();
  auto vel = p_->vehicle->GetVelocity();
  // CARLA (left-handed: y-right, yaw clockwise) -> ROS/ENU (y-left, yaw CCW),
  // matching the /trajectory_cmd frame: negate y and yaw.
  x = tf.location.x;
  y = -tf.location.y;
  yaw = -static_cast<double>(tf.rotation.yaw) * M_PI / 180.0;
  speed = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
  return true;
}

bool CarlaWorld::getEgoOdom(EgoOdom &o, double t) const {
  if (!p_->vehicle) return false;
  auto tf = p_->vehicle->GetTransform();
  auto vel = p_->vehicle->GetVelocity();
  auto av = p_->vehicle->GetAngularVelocity();  // CARLA reports deg/s
  o.t = t;
  o.x = tf.location.x;
  o.y = -tf.location.y;
  o.z = tf.location.z;
  double yaw = -static_cast<double>(tf.rotation.yaw) * M_PI / 180.0;
  o.qz = std::sin(yaw / 2.0);
  o.qw = std::cos(yaw / 2.0);
  o.vx = vel.x;
  o.vy = -vel.y;
  o.vz = vel.z;
  o.wz = -static_cast<double>(av.z) * M_PI / 180.0;
  return true;
}

bool CarlaWorld::getWheelInputs(double &forward_speed, double &yaw_rate) const {
  if (!p_->vehicle) return false;
  auto tf = p_->vehicle->GetTransform();
  auto vel = p_->vehicle->GetVelocity();
  auto av = p_->vehicle->GetAngularVelocity();
  double yaw = static_cast<double>(tf.rotation.yaw) * M_PI / 180.0;  // CARLA frame
  // Signed forward speed = body velocity projected on heading (a speedometer).
  forward_speed = vel.x * std::cos(yaw) + vel.y * std::sin(yaw);
  yaw_rate = -static_cast<double>(av.z) * M_PI / 180.0;  // ENU (CCW)
  return true;
}

void CarlaWorld::setAutopilot(bool enabled) {
  if (!p_->vehicle) return;
  if (enabled) {
    // In a synchronous world the Traffic Manager MUST also be synchronous,
    // otherwise its control stage never advances with our ticks and the
    // autopilot vehicle just sits still.
    auto tm = p_->client.GetInstanceTM();  // default TM port
    tm.SetSynchronousMode(true);
    p_->vehicle->SetAutopilot(true, tm.Port());
  } else {
    p_->vehicle->SetAutopilot(false);
  }
}

void CarlaWorld::setCoverageMode(bool enabled) {
  if (!p_->vehicle) return;

  if (!enabled) {
    p_->vehicle->SetAutopilot(false);
    printf("[carla_cpp] coverage (right-loop) OFF\n");
    return;
  }

  namespace ctm = carla::traffic_manager;
  auto tm = p_->client.GetInstanceTM();  // default TM port
  tm.SetSynchronousMode(true);           // sync world => sync TM (else no motion)

  // Right/Left/Left/Left closed loop. RoadOption codes: Left=1, Right=2
  // (carla/trafficmanager/SimpleWaypoint.h). Mirrors carla_manual_control's
  // tm.set_route(['Right','Left','Left','Left']).
  // const ctm::Route route = {2, 1, 1, 1};
  const ctm::Route route = {2, 2, 2, 2, 2, 2};
  tm.SetImportedRoute(p_->vehicle, route, /*empty_buffer=*/true);

  tm.SetPercentageSpeedDifference(p_->vehicle, 30.0f);
  // Python ignore_lights/signs_percentage == C++ SetPercentageRunningLight/Sign.
  tm.SetPercentageRunningLight(p_->vehicle, 100.0f);  // run reds
  tm.SetPercentageRunningSign(p_->vehicle, 100.0f);   // run stop signs
  tm.SetPercentageIgnoreVehicles(p_->vehicle, 0.0f);
  tm.SetPercentageIgnoreWalkers(p_->vehicle, 0.0f);

  // Give every light a uniform 1s phase, then reset each junction group once so
  // the cycle starts from a known state -> reproducible loop timing run-to-run.
  auto lights = p_->world->GetActors()->Filter("*traffic_light*");
  std::vector<carla::SharedPtr<cc::TrafficLight>> tls;
  for (auto a : *lights) {
    auto tl = boost::static_pointer_cast<cc::TrafficLight>(a);
    tl->Freeze(false);
    tl->SetGreenTime(1.0f);
    tl->SetYellowTime(1.0f);
    tl->SetRedTime(1.0f);
    tls.push_back(tl);
  }
  std::set<carla::ActorId> reset_ids;
  for (auto &tl : tls) {
    if (reset_ids.count(tl->GetId())) continue;
    for (auto &member : tl->GetGroupTrafficLights()) reset_ids.insert(member->GetId());
    tl->ResetGroup();
  }

  p_->vehicle->SetAutopilot(true, tm.Port());
  printf("[carla_cpp] coverage (right-loop) ON: route R/L/L/L, speed +30%%, "
         "ignore lights+signs, %zu traffic lights re-phased\n", tls.size());
}

void CarlaWorld::shutdown() {
  if (!p_->world) return;
  for (auto *s : {&p_->cam_left, &p_->cam_right, &p_->imu, &p_->gnss}) {
    if (*s) {
      if ((*s)->IsListening()) (*s)->Stop();
      (*s)->Destroy();
      s->reset();
    }
  }
  if (p_->vehicle) {
    p_->vehicle->Destroy();
    p_->vehicle.reset();
  }
  // Restore asynchronous mode so the server isn't left waiting on ticks
  // (both the world and the Traffic Manager).
  try { p_->client.GetInstanceTM().SetSynchronousMode(false); } catch (...) {}
  auto settings = p_->world->GetSettings();
  settings.synchronous_mode = false;
  settings.fixed_delta_seconds = boost::optional<double>{};
  p_->world->ApplySettings(settings, 10s);
  p_->world.reset();
  printf("[carla_cpp] world cleaned up\n");
}

}  // namespace carla_cpp
