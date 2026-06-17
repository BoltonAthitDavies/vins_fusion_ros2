/*******************************************************
 * CARLA rosbag2  →  VINS  →  RTAB-Map  (no ROS between them)
 *
 * Ported from the original-API CarlaBagRtabDirect.cpp to the vins_fusion_ros2
 * refactored Estimator API (VINSOptions + initialize(), IMUData/ImageData,
 * options->OUTPUT_FOLDER). The VINS body pose is read with getPoseInWorldFrame()
 * — same as before — and also published on /vins_estimator/odometry (there is
 * no registerPub() in the refactored API).
 *
 * Data path:
 *   disk (.db3)
 *     → rosbag2_cpp::Reader     (C++ direct read)
 *         → Estimator::inputImage()    (VINS stereo odometry)
 *         → Rtabmap::process()         (RTAB-Map loop closure + GPS fusion)
 *
 * NOTE: this binary needs RTAB-Map core (rtabmap::Rtabmap etc.). It is NOT yet
 * wired into CMakeLists — add an add_executable + find_package(rtabmap) and the
 * rtabmap core libs before building it.
 *
 * Usage:
 *   ros2 run vins_fusion_ros2 carla_rtab_direct <config> <bag_path> [rtab_db]
 *
 * Outputs:
 *   <OUTPUT_FOLDER>/vio.txt          VINS body-frame trajectory (TUM-like)
 *   <OUTPUT_FOLDER>/rtab_direct.txt  RTAB optimised trajectory (TUM-like)
 *   <rtab_db>                        RTAB-Map SQLite database
 *******************************************************/

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <memory>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>

// RTAB-Map core (no ROS dependency)
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/GPS.h>

#include <vins/estimator/estimator.h>
#include <vins/estimator/parameters.h>

using namespace std;
using namespace Eigen;

Estimator estimator;

static const string TOP_IMU   = "/carla/ego_vehicle/imu";
static const string TOP_LEFT  = "/carla/ego_vehicle/cam_front_left/image";
static const string TOP_RIGHT = "/carla/ego_vehicle/cam_front_right/image";
static const string TOP_GNSS  = "/carla/ego_vehicle/gnss";

struct ImuRec  { double t; double ax, ay, az, wx, wy, wz; };
struct GnssRec { double t; double lat, lon, alt, err; };

// — Deserialise a bag message —————————————————————————————————————————————

template<typename T>
static T deserialise(const shared_ptr<rcutils_uint8_array_t>& raw)
{
    rclcpp::SerializedMessage ser(raw->buffer_length);
    auto& rcl = ser.get_rcl_serialized_message();
    memcpy(rcl.buffer, raw->buffer, raw->buffer_length);
    rcl.buffer_length = raw->buffer_length;
    T msg;
    rclcpp::Serialization<T>{}.deserialize_message(&ser, &msg);
    return msg;
}

// — TUM-format writer ——————————————————————————————————————————————————————

static void writeTum(FILE* f, double t, const rtabmap::Transform& T)
{
    Eigen::Quaternionf q = T.getQuaternionf();
    fprintf(f, "%.6f %f %f %f %f %f %f %f\n",
            t,
            T.x(), T.y(), T.z(),
            q.x(), q.y(), q.z(), q.w());
}

// — Main ——————————————————————————————————————————————————————————————————

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    cv::theRNG().state = 42;
    auto n = rclcpp::Node::make_shared("vins_estimator");

    // — Publishers (for live visualisation) ——————————————————————————————
    auto pubLeft      = n->create_publisher<sensor_msgs::msg::Image>("/leftImage",  1000);
    auto pubRight     = n->create_publisher<sensor_msgs::msg::Image>("/rightImage", 1000);
    auto pubInfoLeft  = n->create_publisher<sensor_msgs::msg::CameraInfo>("/leftCameraInfo",  10);
    auto pubInfoRight = n->create_publisher<sensor_msgs::msg::CameraInfo>("/rightCameraInfo", 10);
    // Mirrors /rtabmap/mapPath so carla_plot can show the RTAB trajectory
    auto pubRtabPath  = n->create_publisher<nav_msgs::msg::Path>("/rtabmap/mapPath", 10);
    // No registerPub() in the refactored API — publish VINS odometry ourselves.
    auto pubOdom      = n->create_publisher<nav_msgs::msg::Odometry>("/vins_estimator/odometry", 1000);

    // CARLA stereo: fx=fy=480, cx=480, cy=360, w=960, h=720, baseline=0.5m
    sensor_msgs::msg::CameraInfo ci_left, ci_right;
    for (auto* ci : {&ci_left, &ci_right}) {
        ci->width=960; ci->height=720;
        ci->distortion_model="plumb_bob";
        ci->d={0,0,0,0,0};
        ci->k={480,0,480, 0,480,360, 0,0,1};
        ci->r={1,0,0, 0,1,0, 0,0,1};
    }
    ci_left.header.frame_id  = "camera";
    ci_left.p  = {480,0,480,0,   0,480,360,0,   0,0,1,0};
    ci_right.header.frame_id = "right_camera";
    ci_right.p = {480,0,480,-240, 0,480,360,0,   0,0,1,0};

    // — Parse arguments ———————————————————————————————————————————————————
    auto clean = rclcpp::remove_ros_arguments(argc, argv);
    if (clean.size() < 3 || clean.size() > 4) {
        printf("usage: carla_rtab_direct <config> <bag_path> [rtab_db]\n");
        return 1;
    }
    string configFile = clean[1];
    string bagPath    = clean[2];
    string dbPath     = (clean.size() == 4) ? clean[3]
                        : string(getenv("HOME")) + "/rtab_direct.db";
    while (bagPath.size() > 1 && bagPath.back() == '/') bagPath.pop_back();

    printf("config  : %s\n", configFile.c_str());
    printf("bag     : %s\n", bagPath.c_str());
    printf("rtab_db : %s\n", dbPath.c_str());

    auto options = std::make_shared<VINSOptions>();
    options->readParameters(configFile);
    estimator.initialize(options);

    // — Open bag —————————————————————————————————————————————————————————
    rosbag2_cpp::Reader reader;
    {
        rosbag2_storage::StorageOptions opts;
        opts.uri = bagPath; opts.storage_id = "";
        reader.open(opts);
    }
    rosbag2_storage::StorageFilter flt;
    flt.topics = {TOP_IMU, TOP_LEFT, TOP_RIGHT, TOP_GNSS};
    reader.set_filter(flt);

    // — Load all messages ————————————————————————————————————————————————
    vector<ImuRec>       imuData;
    vector<GnssRec>      gnssData;
    map<double, cv::Mat> leftMap, rightMap, leftColorMap;

    printf("reading bag ...\n");
    size_t n_imu=0, n_left=0, n_right=0, n_gnss=0;

    while (reader.has_next()) {
        auto bag_msg = reader.read_next();
        const string& topic = bag_msg->topic_name;

        if (topic == TOP_IMU) {
            auto imu = deserialise<sensor_msgs::msg::Imu>(bag_msg->serialized_data);
            double t = imu.header.stamp.sec + imu.header.stamp.nanosec*1e-9;
            imuData.push_back({t,
                imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
                imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z});
            n_imu++;

        } else if (topic == TOP_GNSS) {
            auto gnss = deserialise<sensor_msgs::msg::NavSatFix>(bag_msg->serialized_data);
            double t = gnss.header.stamp.sec + gnss.header.stamp.nanosec*1e-9;
            double err = (gnss.position_covariance_type > 0 && gnss.position_covariance[0] > 0)
                         ? sqrt(gnss.position_covariance[0]) : 1.0;
            gnssData.push_back({t, gnss.latitude, gnss.longitude, gnss.altitude, err});
            n_gnss++;

        } else if (topic == TOP_LEFT || topic == TOP_RIGHT) {
            auto img_msg = deserialise<sensor_msgs::msg::Image>(bag_msg->serialized_data);
            double t = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec*1e-9;
            cv::Mat gray;
            try {
                gray = cv_bridge::toCvCopy(img_msg, "mono8")->image.clone();
            } catch (const cv_bridge::Exception& e) {
                printf("warning: cv_bridge error at t=%.3f: %s\n", t, e.what());
                continue;
            }
            if (topic == TOP_LEFT) {
                leftMap[t] = gray;
                cv::Mat color = cv_bridge::toCvCopy(img_msg, "bgr8")->image.clone();
                leftColorMap[t] = color;
                n_left++;
            } else {
                rightMap[t] = gray; n_right++;
            }
        }
    }
    printf("loaded  : imu=%zu  left=%zu  right=%zu  gnss=%zu\n",
           n_imu, n_left, n_right, n_gnss);

    if (leftMap.empty()) { printf("error: no left frames found\n"); return 1; }

    sort(imuData.begin(), imuData.end(),
         [](const ImuRec& a, const ImuRec& b){ return a.t < b.t; });

    // — RTAB-Map init —————————————————————————————————————————————————————
    // StereoCameraModel localTransform (body_T_cam0) and baseline are derived
    // from the VINS config's extrinsics (options->RIC[0]/TIC[0]), so RTAB always
    // matches whatever rig the config describes — no hardcoded sign.
    const Eigen::Matrix3d& Rc = options->RIC[0];
    const Eigen::Vector3d& tc = options->TIC[0];
    rtabmap::Transform localTransform(
        (float)Rc(0,0), (float)Rc(0,1), (float)Rc(0,2), (float)tc.x(),
        (float)Rc(1,0), (float)Rc(1,1), (float)Rc(1,2), (float)tc.y(),
        (float)Rc(2,0), (float)Rc(2,1), (float)Rc(2,2), (float)tc.z());
    // Stereo baseline = lateral distance between cam0 and cam1 (config body-y).
    double baseline = 0.5;
    if (options->TIC.size() >= 2)
        baseline = std::abs(options->TIC[0].y() - options->TIC[1].y());
    printf("rtab    : localTransform from config (cam0 y=%.3f), baseline=%.3f m\n",
           tc.y(), baseline);

    rtabmap::StereoCameraModel stereoModel(
        "carla_stereo",
        480.0, 480.0,   // fx, fy
        480.0, 360.0,   // cx, cy
        baseline,       // from config
        localTransform,
        cv::Size(960, 720));

    rtabmap::ParametersMap params;
    using P = rtabmap::Parameters;
    params.insert({P::kRtabmapDetectionRate(),            "0"});
    params.insert({P::kRtabmapLoopRatio(),                "0.0"});
    params.insert({P::kRtabmapLoopGPS(),                  "true"});
    params.insert({P::kRGBDLocalRadius(),                 "10"});
    params.insert({P::kRGBDLoopClosureReextractFeatures(),"true"});
    params.insert({P::kRGBDOptimizeMaxError(),            "0.0"}); // disabled: Robust=tr     ue handles outliers internally
    params.insert({P::kMemSTMSize(),                      "10"});
    params.insert({P::kOptimizerStrategy(),               "2"});   // GTSAM
    params.insert({P::kOptimizerPriorsIgnored(),          "false"});
    params.insert({P::kOptimizerRobust(),                 "true"});
    params.insert({P::kOptimizerGravitySigma(),           "0"});   // VINS handles gravity
    params.insert({P::kRegForce3DoF(),                    "false"});
    params.insert({P::kVisMinInliers(),                   "30"}); // stricter LC: fewer false positives
    params.insert({P::kKpMaxFeatures(),                   "250"});
    params.insert({P::kKpDetectorStrategy(),              "8"});   // GFTT/ORB
    params.insert({P::kVisFeatureType(),                  "8"});
    params.insert({P::kVisMaxDepth(),                     "20.0"});
    params.insert({P::kStereoMinDisparity(),              "2.0"});

    // Always start fresh — remove stale database from a previous run
    if (std::remove(dbPath.c_str()) == 0)
        printf("rtabmap : removed old db %s\n", dbPath.c_str());

    rtabmap::Rtabmap rtab;
    rtab.init(params, dbPath);
    printf("rtabmap : initialised  db=%s\n", dbPath.c_str());

    // — Output files ——————————————————————————————————————————————————————
    FILE* fVio  = fopen((options->OUTPUT_FOLDER + "/vio.txt").c_str(),          "w");
    FILE* fRtab = fopen((options->OUTPUT_FOLDER + "/rtab_direct.txt").c_str(),  "w");
    if (!fVio)  printf("warning: cannot open %s/vio.txt\n",          options->OUTPUT_FOLDER.c_str());
    if (!fRtab) printf("warning: cannot open %s/rtab_direct.txt\n",  options->OUTPUT_FOLDER.c_str());

    size_t imuIdx=0, gnssIdx=0, frameIdx=0;
    int    rtabNodes=0, loopClosures=0;
    map<int, double> nodeStamps;   // node_id → image timestamp (for path header)

    for (auto& [tImg, imLeft] : leftMap) {
        if (!rclcpp::ok()) break;

        // Match right frame
        cv::Mat imRight;
        auto itR = rightMap.find(tImg);
        if (itR != rightMap.end()) {
            imRight = itR->second;
        } else {
            double best_dt = 0.05;
            cv::Mat* best = nullptr;
            auto itLow = rightMap.lower_bound(tImg);
            if (itLow != rightMap.end()) {
                double dt = itLow->first - tImg;
                if (dt < best_dt) { best_dt = dt; best = &itLow->second; }
            }
            if (itLow != rightMap.begin()) {
                --itLow;
                double dt = tImg - itLow->first;
                if (dt < best_dt) { best = &itLow->second; }
            }
            if (!best) { printf("warning: no right frame for t=%.6f, skipping\n", tImg); continue; }
            imRight = *best;
        }

        // Publish images for optional visualisation
        auto stamp = rclcpp::Time(static_cast<uint64_t>(tImg * 1e9));
        auto lMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imLeft).toImageMsg();
        auto rMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imRight).toImageMsg();
        lMsg->header.frame_id = "camera";      lMsg->header.stamp = stamp;
        rMsg->header.frame_id = "right_camera"; rMsg->header.stamp = stamp;
        ci_left.header.stamp = ci_right.header.stamp = stamp;
        pubLeft->publish(*lMsg);  pubRight->publish(*rMsg);
        pubInfoLeft->publish(ci_left); pubInfoRight->publish(ci_right);

        // Feed IMU to VINS (ignored if imu:0 in config)
        while (imuIdx < imuData.size() && imuData[imuIdx].t <= tImg) {
            const auto& r = imuData[imuIdx++];
            IMUData imu;
            imu.timestamp = r.t;
            imu.linear_acceleration = Vector3d(r.ax, r.ay, r.az);
            imu.angular_velocity    = Vector3d(r.wx, r.wy, r.wz);
            estimator.inputIMU(imu);
        }

        // VINS stereo processing
        ImageData img;
        img.timestamp = tImg;
        img.image0 = imLeft;
        img.image1 = imRight;
        estimator.inputImage(img);

        // VINS body pose in world frame
        Matrix4d pose;
        estimator.getPoseInWorldFrame(pose);

        // Write VINS trajectory
        if (fVio)
            fprintf(fVio, "%f %f %f %f %f %f %f %f %f %f %f %f\n",
                    pose(0,0),pose(0,1),pose(0,2),pose(0,3),
                    pose(1,0),pose(1,1),pose(1,2),pose(1,3),
                    pose(2,0),pose(2,1),pose(2,2),pose(2,3));

        // Publish VINS odometry for carla_plot.py
        {
            Matrix3d R = pose.block<3,3>(0,0);
            Quaterniond q(R);
            nav_msgs::msg::Odometry od;
            od.header.stamp = stamp;
            od.header.frame_id = "world";
            od.child_frame_id  = "body";
            od.pose.pose.position.x = pose(0,3);
            od.pose.pose.position.y = pose(1,3);
            od.pose.pose.position.z = pose(2,3);
            od.pose.pose.orientation.x = q.x();
            od.pose.pose.orientation.y = q.y();
            od.pose.pose.orientation.z = q.z();
            od.pose.pose.orientation.w = q.w();
            pubOdom->publish(od);
        }

        // — RTAB-Map direct call ——————————————————————————————————————————
        // VINS uses MULTIPLE_THREAD=1 so inputImage() is async — the pose
        // returned here is for the previous frame (one step lag), which is
        // fine: same lag exists in the ROS topic version.

        rtabmap::Transform odomPose(
            (float)pose(0,0),(float)pose(0,1),(float)pose(0,2),(float)pose(0,3),
            (float)pose(1,0),(float)pose(1,1),(float)pose(1,2),(float)pose(1,3),
            (float)pose(2,0),(float)pose(2,1),(float)pose(2,2),(float)pose(2,3));

        // Flatten z so the map is built on a level plane in real-time.
        rtabmap::Transform odomFlat(
            odomPose.r11(), odomPose.r12(), odomPose.r13(), odomPose.x(),
            odomPose.r21(), odomPose.r22(), odomPose.r23(), odomPose.y(),
            odomPose.r31(), odomPose.r32(), odomPose.r33(), 0.0f);

        cv::Mat imLeftColor;
        auto itC = leftColorMap.find(tImg);
        if (itC != leftColorMap.end()) imLeftColor = itC->second;
        else imLeftColor = imLeft;

        rtabmap::SensorData data(imLeftColor, imRight, stereoModel,
                                 (int)frameIdx, tImg);

        // GPS: convert lat/lon to ENU then apply Rz(180°) to align with VINS world frame.
        if (!gnssData.empty()) {
            while (gnssIdx + 1 < gnssData.size() && gnssData[gnssIdx+1].t <= tImg)
                gnssIdx++;
            const auto& g = gnssData[gnssIdx];
            constexpr double R_EARTH = 6378137.0;
            static double lat0 = g.lat, lon0 = g.lon, alt0 = g.alt;
            double de =  (g.lon - lon0) * (M_PI/180.0) * R_EARTH * std::cos(lat0 * M_PI/180.0);
            double dn =  (g.lat - lat0) * (M_PI/180.0) * R_EARTH;
            double vx = -de, vy = -dn, vz = g.alt - alt0;
            rtabmap::Transform gpsTf(1,0,0, (float)vx, 0,1,0, (float)vy, 0,0,1, (float)vz);
            double err2 = g.err * g.err;
            cv::Mat gpsCov = cv::Mat::eye(6,6,CV_64FC1) * 9999.0;
            gpsCov.at<double>(0,0) = err2;
            gpsCov.at<double>(1,1) = err2;
            gpsCov.at<double>(2,2) = err2 > 1.0 ? err2 : 1.0;
            data.setGlobalPose(gpsTf, gpsCov);
        }

        bool newNode = rtab.process(data, odomFlat);
        if (newNode) {
            const rtabmap::Statistics& stats = rtab.getStatistics();
            int nodeId = stats.refImageId();
            if (nodeId > 0) nodeStamps[nodeId] = tImg;
            ++rtabNodes;
            if (stats.loopClosureId() > 0) {
                ++loopClosures;
                printf("  [LOOP] frame %zu → node %d  (total loops: %d)\n",
                       frameIdx, stats.loopClosureId(), loopClosures);
            }

            // Publish optimised path so carla_plot can display RTAB trajectory
            const auto& poses = rtab.getLocalOptimizedPoses();
            if (!poses.empty()) {
                nav_msgs::msg::Path pathMsg;
                pathMsg.header.frame_id = "map";
                pathMsg.header.stamp    = stamp;
                for (const auto& [id, T] : poses) {
                    geometry_msgs::msg::PoseStamped ps;
                    ps.header.frame_id = "map";
                    ps.header.stamp    = rclcpp::Time(static_cast<uint64_t>(
                        nodeStamps.count(id) ? nodeStamps.at(id) * 1e9 : 0));
                    Eigen::Quaternionf q = T.getQuaternionf();
                    ps.pose.position.x    = T.x();
                    ps.pose.position.y    = T.y();
                    ps.pose.position.z    = 0.0f;
                    ps.pose.orientation.x = q.x();
                    ps.pose.orientation.y = q.y();
                    ps.pose.orientation.z = q.z();
                    ps.pose.orientation.w = q.w();
                    pathMsg.poses.push_back(ps);
                }
                pubRtabPath->publish(pathMsg);
            }
        }

        if (frameIdx % 50 == 0)
            printf("frame %zu/%zu  rtab_nodes=%d  loops=%d\n",
                   frameIdx, leftMap.size(), rtabNodes, loopClosures);
        frameIdx++;
    }

    if (fVio) fclose(fVio);

    // — Extract RTAB optimised trajectory ——————————————————————————————————
    printf("\nextracting optimised graph ...\n");
    {
        map<int, rtabmap::Transform> optimisedPoses;
        multimap<int, rtabmap::Link>  links;
        rtab.getGraph(optimisedPoses, links, true, true);

        if (fRtab) {
            for (auto& [id, T] : optimisedPoses)
                writeTum(fRtab, (double)id, T);
            fclose(fRtab);
        }

        printf("optimised graph: %zu nodes  %zu links\n",
               optimisedPoses.size(), links.size());
    }

    rtab.close();
    printf("done.  db saved to %s\n", dbPath.c_str());

    rclcpp::shutdown();
    return 0;
}
