#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/filters/voxel_grid.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc.hpp"
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
ros::Publisher plane_pub, ledge_pub,  roll_vector_pub;
ros::Publisher pub_MultiDOFJointTrajectory, pub_Nav_msgs_trajectory;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
int ransac_max_iteration = 340;
float setDistanceThresholdvalue = 0.1;
float deg2rad(float alpha)
{
  return (alpha * 0.017453293f);
}

uint32_t shape = visualization_msgs::Marker::ARROW;
visualization_msgs::Marker marker;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{

  // All the objects needed
  pcl::PassThrough<PointT> pass;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PointCloud<PointT>::Ptr rawcloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*input, *rawcloud);


   pcl::VoxelGrid< PointT > avg;
   avg.setInputCloud(rawcloud);
   avg.setLeafSize(0.01f, 0.01f, 0.01f);
   avg.filter(*cloud);



  pcl::ModelCoefficients::Ptr floor_coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices());

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.4, 5);
  pass.filter(*cloud);
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-5, 5);
  pass.filter(*cloud);

  pcl::SACSegmentation<PointT> floor_finder;
  floor_finder.setOptimizeCoefficients(true);
  //floor_finder.setModelType(pcl::SACMODEL_PLANE);
  floor_finder.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  floor_finder.setMethodType(pcl::SAC_RANSAC);
  floor_finder.setMaxIterations(ransac_max_iteration);
  floor_finder.setAxis(Eigen::Vector3f(0, 1, 1));
  floor_finder.setDistanceThreshold(setDistanceThresholdvalue);
  floor_finder.setEpsAngle(deg2rad(15));
  floor_finder.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(*cloud));
  floor_finder.segment(*inliers_plane, *floor_coefficients);
  if (inliers_plane->indices.size() == 0)
  {
    std::cout << "No Roof!!! Could not estimate a planar model for the given dataset." << std::endl;
    return;
  }
  pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>);
  if (inliers_plane->indices.size() > 500)
  {
    // Extract the floor plane inliers
    pcl::PointCloud<PointT>::Ptr floor_points(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(boost::make_shared<pcl::PointCloud<PointT>  >(*cloud));
    extractor.setIndices(inliers_plane);
    //extractor.setNegative (false);
    extractor.filter(*floor_points);
    extractor.setNegative(true);
    extractor.filter(*cloud);

    // Project the floor inliers
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(floor_points);
    proj.setModelCoefficients(floor_coefficients);
    proj.filter(*cloud_projected);
    cloud_projected = floor_points;
    //std::cout << "Find Roof" << std::endl;
 
  }
  else
  {
    std::cout << "No Enough Supporting Points For Roof" << std::endl;
    return;
  }
  plane_pub.publish(cloud_projected);
 // std::cerr << "Roof coefficients: " << *floor_coefficients << std::endl;

  pcl::PointCloud<PointT>::Ptr ledge_projected(new pcl::PointCloud<PointT>);
  floor_finder.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  floor_finder.setAxis(Eigen::Vector3f(0, 1, 1));
  floor_finder.setDistanceThreshold (0.005);
  floor_finder.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(*cloud));
  floor_finder.segment(*inliers_plane, *floor_coefficients);
  if (inliers_plane->indices.size() == 0)
  {
    std::cout << "No Ledge!!! Could not estimate a Ledge model for the given dataset." << std::endl;
    return;
  }
  if (inliers_plane->indices.size() > 30)
  {
    // Extract the floor plane inliers
    pcl::PointCloud<PointT>::Ptr floor_points(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(boost::make_shared<pcl::PointCloud<PointT>  >(*cloud));
    extractor.setIndices(inliers_plane);
    //extractor.setNegative (false);
    extractor.filter(*floor_points);
    extractor.setNegative(true);
    extractor.filter(*cloud);

    // Project the floor inliers
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(floor_points);
    proj.setModelCoefficients(floor_coefficients);
    proj.filter(*ledge_projected);
    ledge_projected = floor_points;
  // std::cout << "Find Ledge" << std::endl;
 
  }
  else
  {
    std::cout << "No Enough Supporting Points For Ledge" << std::endl;
    return;
  }

  //std::cerr << "Ledge coefficients: " << *floor_coefficients << std::endl;

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*ledge_projected, centroid);
  //std::cerr << "Centroid: " << centroid << std::endl;

  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin(tf::Vector3(centroid[0], centroid[1], centroid[2]));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "os_sensor", "target_location"));

  ledge_pub.publish(ledge_projected);




  // // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  // marker.header.frame_id = "/os_sensor";
  // marker.header.stamp = ros::Time::now();
  // // Set the namespace and id for this marker.  This serves to create a unique ID
  // // Any marker sent with the same namespace and id will overwrite the old one
  // marker.ns = "basic_shapes";
  // marker.id = 0;
  // // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  // marker.type = shape;
  // // Set the marker action.  Options are ADD and DELETE
  // marker.action = visualization_msgs::Marker::ADD;
  // // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

  // tf2::Quaternion myQuaternion;
  // myQuaternion.setRPY( 0, 0 , 3.14159/2 );  // Create this quaternion from roll/pitch/yaw (in radians)

  // marker.pose.position.x = 0;
  // marker.pose.position.y = 0;
  // marker.pose.position.z = 0;
  // marker.pose.orientation.x = myQuaternion[0];
  // marker.pose.orientation.y = myQuaternion[1];
  // marker.pose.orientation.z = myQuaternion[2];
  // marker.pose.orientation.w = myQuaternion[3];

  // // Set the scale of the marker -- 1x1x1 here means 1m on a side
  // marker.scale.x = 10 * floor_coefficients->values[1];
  // marker.scale.y = 1* floor_coefficients->values[1];
  // marker.scale.z = 1* floor_coefficients->values[1];

  // // Set the color -- be sure to set alpha to something non-zero!
  // marker.color.r = 1.0f;
  // marker.color.g = 1.0f;
  // marker.color.b = 0.0f;
  // marker.color.a = 1.0;

  // marker.lifetime = ros::Duration();

  // // Publish the marker
  // roll_vector_pub.publish(marker);


  trajectory_msgs::MultiDOFJointTrajectory traj_msg_MDOF;
  trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;

  point_msg.time_from_start.fromSec(ros::Time::now().toSec());
  point_msg.transforms.resize(1);

  point_msg.transforms[0].translation.x= centroid[0];
  point_msg.transforms[0].translation.y= centroid[1];
  point_msg.transforms[0].translation.z= 0;

  point_msg.transforms[0].rotation.x = 0;
  point_msg.transforms[0].rotation.y = 0;
  point_msg.transforms[0].rotation.z = 0;
  point_msg.transforms[0].rotation.w = 1;

  traj_msg_MDOF.points.push_back(point_msg);

  point_msg.time_from_start.fromSec(ros::Time::now().toSec());
  point_msg.transforms.resize(1);

  point_msg.transforms[0].translation.x= centroid[0];
  point_msg.transforms[0].translation.y= centroid[1];
  point_msg.transforms[0].translation.z= centroid[2];

  point_msg.transforms[0].rotation.x = 0;
  point_msg.transforms[0].rotation.y = 0;
  point_msg.transforms[0].rotation.z = 0;
  point_msg.transforms[0].rotation.w = 1;

  traj_msg_MDOF.points.push_back(point_msg);
  pub_MultiDOFJointTrajectory.publish(traj_msg_MDOF);



  nav_msgs::Path traj_msg_Nav_path;
  geometry_msgs::PoseStamped traj_msg_Nav_path_ref_pose;
  traj_msg_Nav_path.header.stamp = ros::Time::now();
  traj_msg_Nav_path.header.frame_id = "os_sensor";

  traj_msg_Nav_path_ref_pose.header.stamp = ros::Time::now();
  traj_msg_Nav_path_ref_pose.header.seq = 0;
  traj_msg_Nav_path_ref_pose.header.frame_id = "os_sensor";
  traj_msg_Nav_path_ref_pose.pose.position.x = 0;//centroid[0];
  traj_msg_Nav_path_ref_pose.pose.position.y = 0;//centroid[0];
  traj_msg_Nav_path_ref_pose.pose.position.z = 0;//centroid[0];
  traj_msg_Nav_path.poses.push_back(traj_msg_Nav_path_ref_pose);

  traj_msg_Nav_path_ref_pose.header.stamp = ros::Time::now();
  traj_msg_Nav_path_ref_pose.header.seq = 1;
  traj_msg_Nav_path_ref_pose.header.frame_id = "os_sensor";
  traj_msg_Nav_path_ref_pose.pose.position.x = 0;//centroid[0];
  traj_msg_Nav_path_ref_pose.pose.position.y = centroid[1];
  traj_msg_Nav_path_ref_pose.pose.position.z = centroid[2];
  traj_msg_Nav_path.poses.push_back(traj_msg_Nav_path_ref_pose);

  traj_msg_Nav_path_ref_pose.header.stamp = ros::Time::now();
  traj_msg_Nav_path_ref_pose.header.seq = 2;
  traj_msg_Nav_path_ref_pose.header.frame_id = "os_sensor";
  traj_msg_Nav_path_ref_pose.pose.position.x = centroid[0];
  traj_msg_Nav_path_ref_pose.pose.position.y = centroid[1];
  traj_msg_Nav_path_ref_pose.pose.position.z = centroid[2];
  traj_msg_Nav_path.poses.push_back(traj_msg_Nav_path_ref_pose);


  
  pub_Nav_msgs_trajectory.publish(traj_msg_Nav_path);

}
void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::Mat input_image=cv_bridge::toCvShare(msg, "bgr8")->image;
    std::cerr << "Image Res"<< input_image.rows<< " "<< input_image.cols << std::endl;
    cv::Mat cropped_input_image = input_image(cv::Rect(input_image.cols*3/8,0,input_image.cols/4, input_image.rows));
    cv::Mat edge_image;
    cv::Canny(cropped_input_image, edge_image, 50, 200, 3);


    cv::imshow("edge_image", edge_image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "ledge_detector");
  ros::NodeHandle nh;
  
  // Create a ROS subscriber for the input point cloud
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("/range_image", 1, image_cb);
  ros::Subscriber LIDAR_sub = nh.subscribe("/os_cloud_node/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  plane_pub = nh.advertise<PCLCloud>("model_plane_output", 1);
  ledge_pub = nh.advertise<PCLCloud>("model_ledge_output", 1);
  roll_vector_pub = nh.advertise<visualization_msgs::Marker>( "roll_vector_visualization_marker", 0 );
  pub_MultiDOFJointTrajectory  = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
  pub_Nav_msgs_trajectory  = nh.advertise<nav_msgs::Path>("/command/trajectory_nav_msgs", 1);

  ros::spin();
}
