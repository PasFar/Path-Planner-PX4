#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <unordered_set>
#include <mutex>
#include <algorithm>
#include <cstdint>

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "aruco_ros/aruco_ros_utils.hpp"

#if __has_include("cv_bridge/cv_bridge.hpp")
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "builtin_interfaces/msg/duration.hpp"

class ArucoSimple : public rclcpp::Node
{
private:
  rclcpp::Node::SharedPtr subNode;
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf2::Stamped<tf2::Transform> rightToLeft;
  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  std::vector<aruco::Marker> markers;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pixel_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;

  // multi-marker support: use int64_t because rclcpp parameters return int64_t
  std::vector<int64_t> marker_ids;       // configured IDs
  std::vector<double> marker_sizes;      // corresponding sizes (meters)

  // backward compatibility single marker
  double marker_size_single;
  int64_t marker_id_single;

  // bookkeeping of seen markers (to ignore duplicates)
  std::unordered_set<int64_t> seen_markers;
  std::mutex file_mutex;
  std::ofstream outfile;
  std::string output_file;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  ArucoSimple()
  : Node("aruco_single"), cam_info_received(false),
    marker_size_single(0.05), marker_id_single(300)
  {
  }

  bool setup()
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subNode = this->create_sub_node(this->get_name());

    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    if (this->has_parameter("corner_refinement")) {
      RCLCPP_WARN(
        this->get_logger(),
        "Corner refinement options have been removed in ArUco 3.0.0, corner_refinement ROS parameter is deprecated");
    }

    // print detector params
    aruco::MarkerDetector::Params params = mDetector.getParameters();
    std::string thresh_method;
    switch (params.thresMethod) {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Threshold method: " << thresh_method);

    // Declare parameters (including vectors)
    this->declare_parameter<double>("marker_size", 0.05);
    this->declare_parameter<int>("marker_id", 300);
    this->declare_parameter<std::string>("reference_frame", "");
    this->declare_parameter<std::string>("camera_frame", "");
    this->declare_parameter<std::string>("marker_frame", "");
    this->declare_parameter<bool>("image_is_rectified", true);
    this->declare_parameter<float>("min_marker_size", 0.02);
    this->declare_parameter<std::string>("detection_mode", "");
    this->declare_parameter<std::vector<int64_t>>("marker_ids", std::vector<int64_t>{});
    this->declare_parameter<std::vector<double>>("marker_sizes", std::vector<double>{});
    this->declare_parameter<std::string>("output_file", "/root/ros2_ws/src/pkg/results/aruco_poses.txt");

    float min_marker_size;  // percentage of image area
    this->get_parameter_or<float>("min_marker_size", min_marker_size, 0.02f);

    std::string detection_mode;
    this->get_parameter_or<std::string>("detection_mode", detection_mode, "DM_FAST");
    if (detection_mode == "DM_FAST") {
      mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
    } else if (detection_mode == "DM_VIDEO_FAST") {
      mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
    } else {
      mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Marker size min: " << min_marker_size << " of image area");
    RCLCPP_INFO_STREAM(this->get_logger(), "Detection mode: " << detection_mode);

    // Subscriptions & publishers
    image_sub = it_->subscribe("/camera", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 1, std::bind(&ArucoSimple::cam_info_callback, this, std::placeholders::_1));

    image_pub = it_->advertise(this->get_name() + std::string("/result"), 1);
    debug_pub = it_->advertise(this->get_name() + std::string("/debug"), 1);
    pose_pub = subNode->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 100);
    transform_pub = subNode->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 100);
    position_pub = subNode->create_publisher<geometry_msgs::msg::Vector3Stamped>("position", 100);
    marker_pub = subNode->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
    pixel_pub = subNode->create_publisher<geometry_msgs::msg::PointStamped>("pixel", 10);

    // read parameters (single + vector)
    this->get_parameter_or<double>("marker_size", marker_size_single, 0.05);
    {
      int tmp_id;
      if (this->get_parameter("marker_id", tmp_id)) {
        marker_id_single = static_cast<int64_t>(tmp_id);
      } else {
        marker_id_single = 300;
      }
    }
    this->get_parameter_or<std::string>("reference_frame", reference_frame, "");
    this->get_parameter_or<std::string>("camera_frame", camera_frame, "");
    this->get_parameter_or<std::string>("marker_frame", marker_frame, "");
    this->get_parameter_or<bool>("image_is_rectified", useRectifiedImages, true);
    this->get_parameter_or<std::vector<int64_t>>("marker_ids", marker_ids, std::vector<int64_t>{});
    this->get_parameter_or<std::vector<double>>("marker_sizes", marker_sizes, std::vector<double>{});
    this->get_parameter_or<std::string>("output_file", output_file, std::string("/root/ros2_ws/src/pkg/results/aruco_poses.txt"));

    // If no vector provided, fall back to single pair for backward compatibility
    if (marker_ids.empty()) {
      marker_ids.push_back(marker_id_single);
      marker_sizes.push_back(marker_size_single);
    } else {
      // if sizes vector shorter than ids vector, pad with single size
      if (marker_sizes.size() < marker_ids.size()) {
        while (marker_sizes.size() < marker_ids.size()) {
          marker_sizes.push_back(marker_size_single);
        }
      }
    }

    rcpputils::assert_true(
      camera_frame != "" && marker_frame != "",
      "Found the camera frame or the marker_frame to be empty!. camera_frame : " +
      camera_frame + " and marker_frame : " + marker_frame);

    if (reference_frame.empty()) {
      reference_frame = camera_frame;
    }

    // open output file: truncate on startup, then keep open for append during runtime
    {
      std::lock_guard<std::mutex> lk(file_mutex);
      // Truncate existing contents so each run starts fresh
      std::ofstream trunc_out(output_file.c_str(), std::ios::out | std::ios::trunc);
      if (trunc_out.is_open()) {
        trunc_out.close();
      } else {
        RCLCPP_WARN(this->get_logger(), "Could not truncate output file (it may not exist yet): %s", output_file.c_str());
      }

      // Open for appending during node runtime
      outfile.open(output_file.c_str(), std::ios::out | std::ios::app);
      if (!outfile.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open output file for append: %s", output_file.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Writing first-time detections to: %s", output_file.c_str());
      }
    }

    // log marker list
    {
      std::stringstream ss;
      for (size_t i = 0; i < marker_ids.size(); ++i) {
        ss << marker_ids[i] << "(" << marker_sizes[i] << "m)";
        if (i + 1 < marker_ids.size()) ss << ", ";
      }
      RCLCPP_INFO(this->get_logger(), "ArUco node started with marker ids to track: %s", ss.str().c_str());
    }

    RCLCPP_INFO(this->get_logger(), "ArUco node will publish pose to TF with %s as parent and %s as child.",
                reference_frame.c_str(), marker_frame.c_str());

    RCLCPP_INFO(this->get_logger(), "Setup of aruco_simple node is successful!");
    return true;
  }

  bool getTransform(
    const std::string & refFrame, const std::string & childFrame,
    geometry_msgs::msg::TransformStamped & transform)
  {
    std::string errMsg;
    if (!tf_buffer_->canTransform(refFrame, childFrame, tf2::TimePointZero,
                                  tf2::durationFromSec(0.5), &errMsg))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get pose from TF: " << errMsg);
      return false;
    } else {
      try {
        transform = tf_buffer_->lookupTransform(refFrame, childFrame, tf2::TimePointZero,
                                                tf2::durationFromSec(0.5));
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
        return false;
      }
    }
    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // if nobody listens anywhere, skip processing
    if ((image_pub.getNumSubscribers() == 0) &&
      (debug_pub.getNumSubscribers() == 0) &&
      (pose_pub->get_subscription_count() == 0) &&
      (transform_pub->get_subscription_count() == 0) &&
      (position_pub->get_subscription_count() == 0) &&
      (marker_pub->get_subscription_count() == 0) &&
      (pixel_pub->get_subscription_count() == 0))
    {
      RCLCPP_DEBUG(this->get_logger(), "No subscribers, not looking for ArUco markers");
      return;
    }

    if (!cam_info_received) {
      return;
    }

    builtin_interfaces::msg::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      // detection results will go into "markers"
      markers.clear();
      // <<== USO LA FIRMA COMPATIBILE PER EVITARE LINKER ERROR ==
      mDetector.detect(inImage, markers, camParam, static_cast<float>(marker_size_single), false);

      for (std::size_t i = 0; i < markers.size(); ++i) {
        int id_int = markers[i].id;
        int64_t id = static_cast<int64_t>(id_int);

        // always draw marker boundary
        markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);

        // check if this id is configured
        auto it = std::find(marker_ids.begin(), marker_ids.end(), id);
        if (it == marker_ids.end()) {
          continue; // not a tracked marker
        }
        size_t idx = std::distance(marker_ids.begin(), it);
        double this_marker_size = marker_sizes.size() > idx ? marker_sizes[idx] : marker_size_single;

        // if we've already seen this marker before, ignore further detections
        // if (seen_markers.find(id) != seen_markers.end()) {
        //   continue;
        // }

        // compute extrinsics for this marker (if camera parameters valid)
        if (camParam.isValid() && this_marker_size > 0) {
          markers[i].calculateExtrinsics(static_cast<float>(this_marker_size), camParam);
        }

        // build transform from marker (marker -> camera)
        tf2::Quaternion q_correction;
        q_correction.setRPY(-M_PI/2, 0.0, 0.0); // togli il 3 in y e - /2 in z
        tf2::Transform correction(q_correction, tf2::Vector3(0,0,0));

        tf2::Transform transform = aruco_ros::arucoMarker2Tf2(markers[i]);
        transform = transform * correction;

        tf2::Stamped<tf2::Transform> cameraToReference;
        cameraToReference.setIdentity();

        if (reference_frame != camera_frame) {
          geometry_msgs::msg::TransformStamped transform_stamped_tmp;
          if (!getTransform(reference_frame, camera_frame, transform_stamped_tmp)) {
            RCLCPP_WARN(this->get_logger(), "Cannot get TF %s -> %s, skipping marker %ld",
                        reference_frame.c_str(), camera_frame.c_str(), static_cast<long>(id));
            continue;
          }
          tf2::fromMsg(transform_stamped_tmp, cameraToReference);
        }

        transform = static_cast<tf2::Transform>(cameraToReference) *
                    static_cast<tf2::Transform>(rightToLeft) *
                    transform;

        // Publish TF/pose/position/pixel
        geometry_msgs::msg::TransformStamped stampedTransform;
        stampedTransform.header.frame_id = reference_frame;
        stampedTransform.header.stamp = curr_stamp;
        stampedTransform.child_frame_id = marker_frame + std::string("_") + std::to_string(id);
        tf2::toMsg(transform, stampedTransform.transform);
        tf_broadcaster_->sendTransform(stampedTransform);

        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.header = stampedTransform.header;
        tf2::toMsg(transform, poseMsg.pose);
        poseMsg.header.frame_id = reference_frame;
        poseMsg.header.stamp = curr_stamp;
        
        // Trasforma la pose del marker in frame "map"
        if (tf_buffer_->canTransform("map", poseMsg.header.frame_id, tf2::TimePointZero)) {
            try {
                geometry_msgs::msg::PoseStamped pose_in_map =
                    tf_buffer_->transform(poseMsg, "map", tf2::durationFromSec(0.1));

                // Aggiorna poseMsg con la versione in "map"
                poseMsg = pose_in_map;

                // Aggiorna anche stampedTransform per TF
                stampedTransform.header.frame_id = "map";
                stampedTransform.header.stamp = curr_stamp;
                tf2::toMsg(tf2::Transform(tf2::Quaternion(poseMsg.pose.orientation.x,
                                                          poseMsg.pose.orientation.y,
                                                          poseMsg.pose.orientation.z,
                                                          poseMsg.pose.orientation.w),
                                          tf2::Vector3(poseMsg.pose.position.x,
                                                      poseMsg.pose.position.y,
                                                      poseMsg.pose.position.z)),
                          stampedTransform.transform);

            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(get_logger(), "Trasformazione in map fallita: %s", ex.what());
            }
        }

        
        pose_pub->publish(poseMsg);

        transform_pub->publish(stampedTransform);

        geometry_msgs::msg::Vector3Stamped positionMsg;
        positionMsg.header = stampedTransform.header;
        positionMsg.vector = stampedTransform.transform.translation;
        position_pub->publish(positionMsg);

        geometry_msgs::msg::PointStamped pixelMsg;
        pixelMsg.header = stampedTransform.header;
        pixelMsg.point.x = markers[i].getCenter().x;
        pixelMsg.point.y = markers[i].getCenter().y;
        pixelMsg.point.z = 0;
        pixel_pub->publish(pixelMsg);

        // publish rviz marker representing the ArUco marker patch
        visualization_msgs::msg::Marker visMarker;
        visMarker.header = stampedTransform.header;
        visMarker.id = static_cast<int>(id); // visMarker.id è int32
        visMarker.type = visualization_msgs::msg::Marker::CUBE;
        visMarker.action = visualization_msgs::msg::Marker::ADD;
        visMarker.pose = poseMsg.pose;
        visMarker.scale.x = this_marker_size;
        visMarker.scale.y = this_marker_size;
        visMarker.scale.z = 0.001;
        visMarker.color.r = 1.0;
        visMarker.color.g = 0.0;
        visMarker.color.b = 0.0;
        visMarker.color.a = 1.0;
        visMarker.lifetime = builtin_interfaces::msg::Duration();
        visMarker.lifetime.sec = 0;
        marker_pub->publish(visMarker);

        // write to file only once per marker and only when z > 0
        // - we consider the pose expressed in the current stampedTransform frame
        //   ("map" if transform to map succeeded), and gate on positive Z
        {
          const double z = stampedTransform.transform.translation.z;
          const bool already_written = (seen_markers.find(id) != seen_markers.end());
          if (!already_written && z > 0.0) {
            std::lock_guard<std::mutex> lk(file_mutex);
            if (outfile.is_open()) {
              // timestamp sec.nanosec with zero padding for nanosec
              outfile << curr_stamp.sec << "." << std::setw(9) << std::setfill('0') << curr_stamp.nanosec
                      << " id:" << id
                      << " pos:[" << stampedTransform.transform.translation.x
                      << "," << stampedTransform.transform.translation.y
                      << "," << stampedTransform.transform.translation.z << "]"
                      << " orientation:[" << stampedTransform.transform.rotation.x
                      << "," << stampedTransform.transform.rotation.y
                      << "," << stampedTransform.transform.rotation.z
                      << "," << stampedTransform.transform.rotation.w << "]"
                      << " size:" << this_marker_size
                      << std::endl;
              outfile.flush();
              // mark as seen only after successful write so we don't write this marker again
              seen_markers.insert(id);
            } else {
              RCLCPP_WARN(this->get_logger(), "Output file %s not open; cannot write marker %ld", output_file.c_str(), static_cast<long>(id));
            }
          }
        }
      } // for markers

      // Draw 3D axis for tracked markers that have extrinsics
      if (camParam.isValid()) {
        for (std::size_t i = 0; i < markers.size(); ++i) {
          int64_t id = static_cast<int64_t>(markers[i].id);
          auto it = std::find(marker_ids.begin(), marker_ids.end(), id);
          if (it == marker_ids.end()) continue;
          size_t idx = std::distance(marker_ids.begin(), it);
          double this_marker_size = marker_sizes.size() > idx ? marker_sizes[idx] : marker_size_single;
          if (this_marker_size > 0) {
            markers[i].calculateExtrinsics(static_cast<float>(this_marker_size), camParam);
            aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }
      }

      // publish annotated image
      if (image_pub.getNumSubscribers() > 0) {
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
      }

      // publish debug threshold image
      if (debug_pub.getNumSubscribers() > 0) {
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector.getThresholdedImage();
        debug_pub.publish(debug_msg.toImageMsg());
      }

    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  void cam_info_callback(const sensor_msgs::msg::CameraInfo & msg)
  {
    if (!cam_info_received) {
      camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

      // handle cartesian offset between stereo pairs
      rightToLeft.setIdentity();
      rightToLeft.setOrigin(tf2::Vector3(-msg.p[3] / msg.p[0], -msg.p[7] / msg.p[5], 0.0));

      cam_info_received = true;
    }
  }

  ~ArucoSimple() override
  {
    std::lock_guard<std::mutex> lk(file_mutex);
    if (outfile.is_open()) outfile.close();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ArucoSimple> aruco_simple = std::make_shared<ArucoSimple>();
  if (!aruco_simple->setup()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to setup ArucoSimple node");
    return 1;
  }
  rclcpp::spin(aruco_simple);
  rclcpp::shutdown();
  return 0;
}