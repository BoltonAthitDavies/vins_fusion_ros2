/*******************************************************
 * carla_cpp: carla_vins_multi_node — main
 *
 * Like carla_vins_node, but runs ALL FIVE VINS variants off the same live
 * CARLA stream in one process (one sim tick feeds every estimator, so they see
 * identical data). Publishes:
 *   /vins_mono/odometry  /vins_stereo/odometry  /vins_stereo_imu/odometry
 *   /vins_stereo_gps/odometry  /vins_stereo_imu_gps/odometry
 * GT comes from CARLA on_tick in the visualizer (plot_result_rtab_cpp.py).
 *
 * Usage:
 *   carla_vins_multi_node <config_dir_or_yaml> [host] [port]
 *                         [--town T] [--spawn x,y,z,roll,pitch,yaw] [--autopilot]
 * The config arg only locates the config/carla dir; the three variant configs
 * (carla_native_mono/stereo/.yaml) are loaded from there.
 *******************************************************/
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

#include <cmath>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "carla_world.hpp"
#include "frame_collector.hpp"
#include "keyboard_control.hpp"
#include "mpc.hpp"
#include "multi_vins.hpp"
#include "noise_odom.hpp"
#include "ros_output.hpp"
#include "types.hpp"
#include "wheel_odom.hpp"

#include <opencv2/core.hpp>

using namespace std::chrono_literals;

namespace {

carla_cpp::CarlaWorld::SpawnPose parseSpawn(const std::string &s) {
  carla_cpp::CarlaWorld::SpawnPose p{100.f, 10.f, 1.f, 0.f, 0.f, -90.f};
  std::vector<float> v;
  std::string tok;
  for (char c : s + ",") {
    if (c == ',') {
      if (!tok.empty()) { v.push_back(std::stof(tok)); tok.clear(); }
    } else {
      tok += c;
    }
  }
  if (v.size() >= 6) { p.x = v[0]; p.y = v[1]; p.z = v[2]; p.roll = v[3]; p.pitch = v[4]; p.yaw = v[5]; }
  return p;
}

std::string dirOf(const std::string &path) {
  auto slash = path.find_last_of('/');
  return slash == std::string::npos ? std::string(".") : path.substr(0, slash);
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  cv::theRNG().state = 42;  // seed RANSAC for deterministic feature geometry
  auto plain = rclcpp::remove_ros_arguments(argc, argv);

  if (plain.size() < 2) {
    printf("usage: carla_vins_multi_node <config_dir_or_yaml> [host] [port] "
           "[--town Town10HD] [--spawn x,y,z,roll,pitch,yaw] "
           "[--autopilot|--coverage|--trajectory] [--mpc-state <odom_topic>] "
           "[--wheel-noise <level>] [--noise-odom <level>] "
           "[--bootstrap-secs <N>] [--target-speed <m/s>] [--horizon <steps>] "
           "[--variants all|none|<csv>]\n");
    return 1;
  }

  std::string config_arg = plain[1];
  std::string host = "localhost";
  uint16_t port = 2000;
  std::string spawn_str = "100.0,10.0,1.0,0.0,0.0,-90.0";
  std::string town = "Town10HD";
  bool start_autopilot = false;
  bool start_coverage = false;
  bool start_trajectory = false;
  std::string mpc_state_topic;   // if set, T-mode MPC uses this odom as state
  double wheel_noise = -1.0;     // >=0 -> publish /carla/ego_vehicle/wheel_odometry
  double noise_odom = -1.0;      // >=0 -> publish /carla/ego_vehicle/noise_odometry
  double bootstrap_secs = 0.0;   // >0 -> bootstrap T-mode on noise_odom, then switch
  double mpc_target_speed = -1.0;// >=0 -> override MPC cruise speed (m/s)
  int mpc_horizon = -1;          // >0  -> override MPC prediction horizon (steps)
  std::string variants_arg;      // CSV of variants to run; empty -> derive/all

  int positional = 0;
  for (size_t i = 2; i < plain.size(); ++i) {
    const std::string &a = plain[i];
    if (a == "--autopilot") {
      start_autopilot = true;
    } else if (a == "--coverage") {
      start_coverage = true;
    } else if (a == "--trajectory") {
      start_trajectory = true;
    } else if (a == "--mpc-state" && i + 1 < plain.size()) {
      mpc_state_topic = plain[++i];
    } else if (a == "--wheel-noise" && i + 1 < plain.size()) {
      wheel_noise = std::stod(plain[++i]);
    } else if (a == "--noise-odom" && i + 1 < plain.size()) {
      noise_odom = std::stod(plain[++i]);
    } else if (a == "--bootstrap-secs" && i + 1 < plain.size()) {
      bootstrap_secs = std::stod(plain[++i]);
    } else if (a == "--target-speed" && i + 1 < plain.size()) {
      mpc_target_speed = std::stod(plain[++i]);
    } else if (a == "--horizon" && i + 1 < plain.size()) {
      mpc_horizon = std::stoi(plain[++i]);
    } else if (a == "--variants" && i + 1 < plain.size()) {
      variants_arg = plain[++i];
    } else if (a == "--spawn" && i + 1 < plain.size()) {
      spawn_str = plain[++i];
    } else if (a == "--town" && i + 1 < plain.size()) {
      town = plain[++i];
    } else if (positional == 0) {
      host = a; positional++;
    } else if (positional == 1) {
      port = static_cast<uint16_t>(std::stoi(a)); positional++;
    }
  }

  // config_arg may be a directory or one of the yaml files; use its directory.
  std::string cfg_dir = (config_arg.size() > 5 &&
                         config_arg.substr(config_arg.size() - 5) == ".yaml")
                            ? dirOf(config_arg)
                            : config_arg;

  auto node = rclcpp::Node::make_shared("vins_estimator");
  carla_cpp::RosOutput ros(node);

  // Decide which VINS estimators to build. Fewer = less live CPU load so the
  // real-time path keeps up. Map variant/topic names -> estimator keys
  // (stereo[_gps] -> "stereo", stereo_imu[_gps] -> "stereo_imu", mono -> "mono").
  auto estKey = [](const std::string &s) -> std::string {
    if (s.find("mono") != std::string::npos) return "mono";
    if (s.find("stereo_imu") != std::string::npos) return "stereo_imu";
    if (s.find("stereo") != std::string::npos) return "stereo";
    return "";
  };
  std::set<std::string> enabled;
  std::string va = variants_arg;
  std::transform(va.begin(), va.end(), va.begin(), ::tolower);
  if (va == "auto") { va.clear(); variants_arg.clear(); }   // auto = derive from mpc_state
  if (va == "none") {
    enabled.insert("__none__");                 // build no estimators (e.g. GT-only control)
  } else if (!va.empty() && va != "all") {
    std::string tok;
    for (char c : variants_arg + ",")
      if (c == ',') { auto k = estKey(tok); if (!k.empty()) enabled.insert(k); tok.clear(); }
      else if (c != ' ') tok += c;
  } else if (va.empty() && !mpc_state_topic.empty()) {
    auto k = estKey(mpc_state_topic);           // auto: just the mpc_state's estimator
    if (!k.empty()) enabled.insert(k);
  }
  if (enabled.count("__none__")) printf("[carla_cpp] VINS estimators: NONE\n");
  else if (enabled.empty())      printf("[carla_cpp] VINS estimators: ALL (mono, stereo, stereo_imu)\n");
  else { printf("[carla_cpp] VINS estimators:"); for (auto &e : enabled) printf(" %s", e.c_str()); printf("\n"); }

  carla_cpp::MultiVins multi(node, cfg_dir, enabled);
  carla_cpp::FrameCollector collector;
  carla_cpp::KeyboardControl kb;

  // T mode: MPC that follows a single /trajectory_cmd target (Pose2D, ENU).
  carla_cpp::TrajectoryMpc mpc;
  if (mpc_target_speed >= 0.0) {
    mpc.setTargetSpeed(mpc_target_speed);
    printf("[carla_cpp] MPC target_speed set to %.2f m/s\n", mpc.targetSpeed());
  }
  if (mpc_horizon > 0) {
    mpc.setHorizon(mpc_horizon);
    printf("[carla_cpp] MPC horizon set to %d steps (%.2f s)\n",
           mpc.horizon(), mpc.horizon() * 0.10);
  }
  carla_cpp::TrajTarget traj_target;
  std::mutex traj_mtx;
  auto traj_sub = node->create_subscription<geometry_msgs::msg::Pose2D>(
      "/carla/ego_vehicle/trajectory_cmd", rclcpp::QoS(rclcpp::KeepLast(10)),
      [&](const geometry_msgs::msg::Pose2D::SharedPtr m) {
        std::lock_guard<std::mutex> lk(traj_mtx);
        traj_target.x = m->x; traj_target.y = m->y; traj_target.theta = m->theta;
        traj_target.valid = true;
      });
  bool traj_mode = start_trajectory;

  // Optional external state for the MPC. With --mpc-state /vins_.../odometry the
  // loop closes on the SAME estimate that play_gt_path.py anchors its targets to
  // (its --state), instead of the CARLA ground truth. Empty -> use CARLA GT.
  struct MpcState { double x = 0, y = 0, yaw = 0, speed = 0; bool valid = false; };
  MpcState ext_state;
  std::mutex ext_state_mtx;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub;
  if (!mpc_state_topic.empty()) {
    state_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        mpc_state_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [&](const nav_msgs::msg::Odometry::SharedPtr m) {
          const auto &q = m->pose.pose.orientation;
          double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
          std::lock_guard<std::mutex> lk(ext_state_mtx);
          ext_state.x = m->pose.pose.position.x;
          ext_state.y = m->pose.pose.position.y;
          ext_state.yaw = yaw;
          ext_state.speed = std::hypot(m->twist.twist.linear.x, m->twist.twist.linear.y);
          ext_state.valid = true;
        });
    printf("[carla_cpp] MPC state source: %s\n", mpc_state_topic.c_str());
  }

  // Optional synthesised wheel odometry on /carla/ego_vehicle/wheel_odometry.
  // Off by default (level < 0). level 0 = clean, >0 = realistic drift. Feed it
  // to the MPC with --mpc-state /carla/ego_vehicle/wheel_odometry to bootstrap
  // motion before VINS converges, or to RTK/global_fusion as a dead-reckoning
  // prior. Available from t=0 with no initialization.
  const bool publish_wheel = wheel_noise >= 0.0;
  carla_cpp::WheelOdom wheel_odom(std::max(0.0, wheel_noise));
  if (publish_wheel)
    printf("[carla_cpp] wheel odometry ENABLED (noise=%.2f) -> "
           "/carla/ego_vehicle/wheel_odometry\n", wheel_noise);

  // Noise odometry on /carla/ego_vehicle/noise_odometry: GT pose + bounded noise.
  // Used as the cheap "available at t=0" bootstrap state for T-mode (--bootstrap-
  // secs). Auto-enabled whenever bootstrap is on. Off otherwise unless requested.
  const bool bootstrap_on = bootstrap_secs > 0.0 && state_sub;
  if (bootstrap_on && noise_odom < 0.0) noise_odom = 0.5;   // sensible default level
  const bool publish_noise = noise_odom >= 0.0;
  carla_cpp::NoiseOdom noise_gen(std::max(0.0, noise_odom));
  if (publish_noise)
    printf("[carla_cpp] noise odometry ENABLED (noise=%.2f) -> "
           "/carla/ego_vehicle/noise_odometry\n", noise_odom);

  // Bootstrap/handover state. While bootstrapping, the MPC reads the noise odom
  // (world frame). At the switch we capture the SE(2) transform mapping the
  // selected odom (--mpc-state, its own frame) onto the world frame so the MPC
  // state stays continuous and the world-frame targets stay valid afterward.
  bool switched = !bootstrap_on;   // if no bootstrap, "switched" from the start
  double traj_start_t = -1.0;      // sim time T-mode (re)engaged
  double cur_t = -1.0;            // latest sim time (from the tick loop); <0 = not set yet
  double hand_dyaw = 0.0, hand_tx = 0.0, hand_ty = 0.0;  // selected -> world SE(2)
  carla_cpp::EgoOdom last_noise{};
  bool have_noise = false;
  if (bootstrap_on)
    printf("[carla_cpp] bootstrap: noise_odometry for >=%.1fs, then switch to %s\n",
           bootstrap_secs, mpc_state_topic.c_str());

  try {
    carla_cpp::CarlaWorld world(host, port, collector);
    world.setup(0.005, parseSpawn(spawn_str), town);

    kb.start();
    if (start_trajectory) {
      printf("[carla_cpp] trajectory (MPC) mode ENABLED --- waiting for "
             "/carla/ego_vehicle/trajectory_cmd%s\n",
             kb.interactive() ? " (press P/C/T to change)" : " (no TTY)");
    } else if (start_coverage) {
      world.setCoverageMode(true);   // C-mode right-loop route (the original setup)
      printf("[carla_cpp] coverage mode ENABLED (right-loop)%s\n",
             kb.interactive() ? " (press P/C to change)" : " (no TTY)");
    } else if (start_autopilot || !kb.interactive()) {
      world.setAutopilot(true);
      printf("[carla_cpp] autopilot ENABLED%s\n",
             kb.interactive() ? " (press P for manual)" : " (no TTY)");
    }

    while (rclcpp::ok() && !kb.quitRequested()) {
      rclcpp::spin_some(node);   // process /trajectory_cmd callbacks
      kb.poll();
      if (kb.consumeCoverageToggle()) { world.setCoverageMode(kb.coverage()); traj_mode = false; }
      if (kb.consumeAutopilotToggle()) {
        if (kb.autopilot()) world.setCoverageMode(false);
        world.setAutopilot(kb.autopilot());
        traj_mode = false;
        printf("[carla_cpp] autopilot %s\n", kb.autopilot() ? "ON" : "OFF");
      }
      if (kb.consumeTrajectoryToggle()) {
        traj_mode = kb.trajectory();
        if (traj_mode) {
          world.setAutopilot(false); world.setCoverageMode(false);
          traj_start_t = -1.0; switched = !bootstrap_on;   // re-arm bootstrap window
        }
        printf("[carla_cpp] trajectory (MPC) %s\n", traj_mode ? "ON" : "OFF");
      }

      if (traj_mode) {
        // T mode: drive the ego toward the latest /trajectory_cmd via the MPC.
        // State source progression:
        //   bootstrap window -> noise odom (world frame, available at t=0)
        //   after window + selected odom live -> selected odom (--mpc-state)
        //   no --mpc-state -> CARLA ground truth throughout
        // Remember engage time -- but ONLY once cur_t holds a real sim time
        // (it's set from b.t at the END of the loop, so it's <0 on the very
        // first iteration). Stamping it at cur_t=0 while sim time is huge made
        // (cur_t - traj_start_t) >= bootstrap_secs true instantly -> the bootstrap
        // was skipped and the MPC handed over to the unconverged state at t=0.
        if (traj_start_t < 0.0 && cur_t >= 0.0) traj_start_t = cur_t;
        const bool boot_timing_ready = traj_start_t >= 0.0;

        // Selected-odom snapshot (--mpc-state), in its own frame.
        bool sel_valid = false; double sx = 0, sy = 0, syaw = 0, sspeed = 0;
        if (state_sub) {
          std::lock_guard<std::mutex> lk(ext_state_mtx);
          sel_valid = ext_state.valid;
          sx = ext_state.x; sy = ext_state.y; syaw = ext_state.yaw;
          sspeed = ext_state.speed;
        }

        // Hand over once the minimum window has elapsed AND the selected odom is
        // actually publishing (so 2 s is a floor, not a guess at convergence).
        // Capture the SE(2) that maps the selected frame onto the world frame at
        // this instant, so the MPC state and world-frame targets stay continuous.
        if (!switched && boot_timing_ready && sel_valid && have_noise &&
            (cur_t - traj_start_t) >= bootstrap_secs) {
          double byaw = std::atan2(2.0 * last_noise.qw * last_noise.qz,
                                   1.0 - 2.0 * last_noise.qz * last_noise.qz);
          hand_dyaw = byaw - syaw;
          double c = std::cos(hand_dyaw), s = std::sin(hand_dyaw);
          hand_tx = last_noise.x - (c * sx - s * sy);
          hand_ty = last_noise.y - (s * sx + c * sy);
          switched = true;
          printf("[carla_cpp] bootstrap -> handover to %s at t+%.2fs "
                 "(dyaw=%.1f deg)\n", mpc_state_topic.c_str(),
                 cur_t - traj_start_t, hand_dyaw * 180.0 / M_PI);
        }

        double ex, ey, eyaw, espeed; bool have_state = false;
        if (switched) {
          if (state_sub) {
            if (sel_valid) {              // selected odom mapped into world frame
              double c = std::cos(hand_dyaw), s = std::sin(hand_dyaw);
              ex = c * sx - s * sy + hand_tx;
              ey = s * sx + c * sy + hand_ty;
              eyaw = syaw + hand_dyaw;
              espeed = sspeed; have_state = true;
            }
          } else {                        // no --mpc-state: CARLA ground truth
            have_state = world.getEgoState(ex, ey, eyaw, espeed);
          }
        } else if (have_noise) {          // bootstrap on the noise odom (world frame)
          ex = last_noise.x; ey = last_noise.y;
          eyaw = std::atan2(2.0 * last_noise.qw * last_noise.qz,
                            1.0 - 2.0 * last_noise.qz * last_noise.qz);
          espeed = std::hypot(last_noise.vx, last_noise.vy);
          have_state = true;
        }
        if (have_state) {
          carla_cpp::TrajTarget tgt;
          { std::lock_guard<std::mutex> lk(traj_mtx); tgt = traj_target; }
          auto cmd = mpc.compute(ex, ey, eyaw, espeed, tgt, 0.0125);
          world.applyControl(cmd);
          // Periodic control debug: lets us tell a commanded stop (thr~0/brk>0)
          // from a physical stall (thr>0 but speed stays ~0).
          static int dbg = 0;
          if (tgt.valid && (dbg++ % 40 == 0))
            printf("[mpc] spd=%.2f thr=%.2f brk=%.2f steer=%+.2f  d2tgt=%.1f  "
                   "pos=(%.1f,%.1f) tgt=(%.1f,%.1f)\n",
                   espeed, cmd.throttle, cmd.brake, cmd.steer,
                   std::hypot(tgt.x - ex, tgt.y - ey), ex, ey, tgt.x, tgt.y);
        }
      } else if (kb.interactive() && kb.manualDriving()) {
        // Manual control only from a TTY; otherwise the zero default fights the TM.
        world.applyControl(kb.command());
      }

      uint64_t frame = world.tick();
      auto b = collector.take(frame, 50ms);

      if (!b.imu.empty()) {
        cur_t = b.t;
        ros.publishClock(b.t);
        carla_cpp::EgoOdom eo;
        if (world.getEgoOdom(eo, b.t)) {
          ros.publishOdometry(eo);  // GT odometry
          if (publish_noise) {
            carla_cpp::EgoOdom no;
            noise_gen.apply(eo, no);  // GT + bounded noise (bootstrap state)
            ros.publishNoiseOdom(no);
            last_noise = no; have_noise = true;
          }
        }
        if (publish_wheel) {
          double fwd, yawrate;
          if (world.getWheelInputs(fwd, yawrate)) {
            carla_cpp::EgoOdom wo;
            // One step per published frame; b.imu may carry several IMU samples,
            // so integrate over their span (dt = #samples * imu period).
            wheel_odom.step(fwd, yawrate, b.imu.size() * 0.0125, b.t, wo);
            ros.publishWheelOdom(wo);
          }
        }
      }
      for (const auto &s : b.imu) multi.pushImu(s);
      if (b.hasStereo) multi.pushStereo(b.left, b.right, b.tImg);
      if (b.hasGnss) multi.pushGnss(b.tGnss, b.lat, b.lon, b.alt);
      ros.publish(b);
    }

    printf("[carla_cpp] shutting down...\n");
    kb.stop();
    multi.stop();
    world.shutdown();
  } catch (const std::exception &e) {
    kb.stop();
    printf("[carla_cpp] FATAL: %s\n", e.what());
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::shutdown();
  return 0;
}
