#include "include/CustomMsg.h"
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;

string bag_file;
string lidar_topic;
string camera_topic;
string save_dir;
double bag_reading_start_time, bag_reading_duration_in_sec;

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidarCamCalib");
  ros::NodeHandle nh;
  nh.param<string>("bag_file", bag_file, "");
  nh.param<string>("save_dir", save_dir, "");
  nh.param<string>("lidar_topic", lidar_topic, "");
  nh.param<string>("camera_topic", camera_topic, "");
  // Read start time and end time
  nh.param<double>("bag_reading_start_time", bag_reading_start_time, 0);
  nh.param<double>("bag_reading_duration_in_sec", bag_reading_duration_in_sec, 1e10);
  
  // Get the file name from the path using pathlib
  string bag_name = bag_file.substr(bag_file.find_last_of("/\\") + 1) + ".bag";
  save_dir = save_dir + "/" + bag_name;
  if (mkdir(save_dir.c_str(), 0777) == -1) {
    ROS_WARN_STREAM("The directory " << save_dir << " already exists");
  } else {
    ROS_INFO_STREAM("Create the directory " << save_dir);
  }
  
  // Check bag_reading_duration_in_sec is larger than 0
  if (bag_reading_duration_in_sec <= 0) {
    ROS_ERROR_STREAM("bag_reading_duration_in_sec should be larger than 0");
    return -1;
  }

  // check if the file exists
  std::fstream file_;
  file_.open(bag_file, ios::in);
  if (!file_) {
    std::string msg = "Loading the rosbag " + bag_file + " failue";
    ROS_ERROR_STREAM(msg.c_str());
    return -1;
  }
  ROS_INFO("Loading the rosbag %s", bag_file.c_str());
  
  // open the bag
  rosbag::Bag bag;
  try {
    bag.open(bag_file, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return -1;
  }

  // Check bag_reading_start_time is larger than rosbag start time and smaller than rosbag end time. If not, set as the start time of the rosbag
  if (bag_reading_start_time < bag.getBeginTime().toSec() || bag_reading_start_time > bag.getEndTime().toSec()) {
    ROS_WARN_STREAM("bag_reading_start_time is not in the range of the rosbag. Set as the start time of the rosbag");
    bag_reading_start_time = bag.getBeginTime().toSec();
  }

  // Check bag_reading_duration_in_sec is smaller than the duration of the rosbag. If not, set as the duration of the rosbag
  if (bag_reading_duration_in_sec > bag.getEndTime().toSec() - bag_reading_start_time) {
    ROS_WARN_STREAM("bag_reading_duration_in_sec is larger than the duration of the rosbag. Set as the duration of the rosbag");
    bag_reading_duration_in_sec = bag.getEndTime().toSec() - bag_reading_start_time;
  }

  std::vector<string> target_topic_vec;
  target_topic_vec.push_back(lidar_topic);
  target_topic_vec.push_back(camera_topic);
  rosbag::View view(bag, rosbag::TopicQuery(target_topic_vec));

  // Initialize the point cloud and image
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  cv::Mat output_cv_img;

  for (const rosbag::MessageInstance &m : view) {

    // Cut rosbag given start time and duration
    if ((m.getTime().toSec() < bag_reading_start_time) or 
        (m.getTime().toSec() > bag_reading_start_time + bag_reading_duration_in_sec)) {
      continue;
    }

    // Check m is point cloud message
    if (m.getDataType() == "sensor_msgs/PointCloud2") {
      // Accumulate point clouds
      sensor_msgs::PointCloud2 pcl_msg;
      pcl_msg = *(m.instantiate<sensor_msgs::PointCloud2>()); // message
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(pcl_msg, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, cloud);
      for (uint i = 0; i < cloud.size(); ++i) {
        output_cloud.points.push_back(cloud.points[i]);
      }
      is_pc_processed = true;
    }
    // Check m is compressed image message
    else if (m.getDataType() == "sensor_msgs/CompressedImage") {
      // Update to the last image
      sensor_msgs::CompressedImage img_msg;
      img_msg = *(m.instantiate<sensor_msgs::CompressedImage>()); // message
      output_cv_img = cv::imdecode(cv::Mat(img_msg.data), 1);
    }
    // Check m is raw image message
    else if (m.getDataType() == "sensor_msgs/Image") {
      // Update to the last image
      sensor_msgs::Image img_msg;
      img_msg = *(m.instantiate<sensor_msgs::Image>()); // message
      if (img_msg.encoding == "bgr8") {
        output_cv_img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
      } else if (img_msg.encoding == "mono8") {
        output_cv_img = cv_bridge::toCvCopy(img_msg, "mono8")->image;
      } else {
        ROS_ERROR_STREAM("The image encoding is not supported");
        return -1;
      }
    }
    else {
      ROS_ERROR_STREAM("The message type is not supported");
      return -1;
    }
  }

  // check output_cloud and output_cv_img are not empty
  if (output_cloud.points.size() == 0) {
    ROS_ERROR_STREAM("The point cloud is not processed.");
    return -1;
  }
  if (output_cv_img.empty()) {
    ROS_ERROR_STREAM("The image is not processed.");
    return -1;
  }

  // File path to save pcd and img
  pcd_file_path = save_dir + "/calib_pc.pcd";
  img_file_path = save_dir + "/calib_img.png";

  // Save point cloud to pcd file
  output_cloud.is_dense = false;
  output_cloud.width = output_cloud.points.size();
  output_cloud.height = 1;
  pcl::io::savePCDFileASCII(pcd_file_path, output_cloud);
  string msg = "Sucessfully save point cloud to pcd file: " + pcd_file_path;
  ROS_INFO_STREAM(msg.c_str());

  // Save image to png file
  cv::imwrite(img_file_path, img);
  msg = "Sucessfully save img to png file: " + img_file_path;
  ROS_INFO_STREAM(msg.c_str());

  return 0;
}