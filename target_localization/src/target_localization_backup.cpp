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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
ros::Publisher plane_pub;
ros::Publisher ledge_pub;
using namespace cv;
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
int ransac_max_iteration = 340;
float setDistanceThresholdvalue = 0.1;
float deg2rad(float alpha)
{
  return (alpha * 0.017453293f);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  // All the objects needed
  pcl::PassThrough<PointT> pass;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*input, *cloud);
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
    std::cout << "Find Roof" << std::endl;
 
  }
  else
  {
	std::cout << "No Enough Supporting Points For Roof" << std::endl;
	return;
  }
  plane_pub.publish(cloud_projected);
  std::cerr << "Roof coefficients: " << *floor_coefficients << std::endl;

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
   std::cout << "Find Ledge" << std::endl;
 
  }
  else
  {
	std::cout << "No Enough Supporting Points For Ledge" << std::endl;
	return;
  }

  std::cerr << "Ledge coefficients: " << *floor_coefficients << std::endl;

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*ledge_projected, centroid);
  std::cerr << "Centroid: " << centroid << std::endl;

  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin(tf::Vector3(centroid[0], centroid[1], centroid[2]));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "os_sensor", "target_location"));



  ledge_pub.publish(ledge_projected);
  //cylinder_pub.publish(cloud_cylinder);
}


static const std::string OPENCV_WINDOW = "Image window";

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw an example circle on the video stream
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(1);

  // Output modified video stream
  //image_pub_.publish(cv_ptr->toImageMsg());
}


Mat3b canvas;
string buttonText("Search For Target");
string buttonText2("Cancel");
string winName = "Simple GUI";
int a = 0;//mozna pozniej usunac, potrzebne tylko czy button reaguje jak nalezy

Rect button, button2, button3, button4, button5;

void callBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        if (button.contains(Point(x, y)))
        {
          
            cout << "Start search mission\n" << endl;
            canvas(button) = Vec3b(50, 50, 50);
            putText(canvas(button), buttonText, Point(button.width*0.35, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));
            putText(canvas(button2), buttonText2, Point(button.width*0.40, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));

        }
        else if (button2.contains(Point(x, y)))//ponizej to co ma sie wykonac po nacisnieciu klawisza
        {
            //a = a + 7;
            cout << "Cancel mission clicked\n" << endl;
            //printf("liczba = %i\n", a);
            //rectangle(canvas(button2), button, Scalar(0, 0, 255), 2);
            //canvas(button2) = Vec3b(50, 50, 50);
            putText(canvas(button), buttonText, Point(button.width*0.35, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));
            putText(canvas(button2), buttonText2, Point(button.width*0.40, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));
        }
        else if (button3.contains(Point(x, y)))//ponizej to co ma sie wykonac po nacisnieciu klawisza
        {
            
            cout << "Go place 1 pick item 1\n" << endl;
            
        }
        else if (button4.contains(Point(x, y)))//ponizej to co ma sie wykonac po nacisnieciu klawisza
        {
            
            cout << "Go place 2 pick item 2\n" << endl;
            
        }
        else if (button5.contains(Point(x, y)))//ponizej to co ma sie wykonac po nacisnieciu klawisza
        {
            
            cout << "Go place 2 pick item 2\n" << endl;
            
        }
    }
    //if (event == EVENT_LBUTTONUP)
    //{
    //rectangle(canvas, button, Scalar(200, 200, 200), 2);
    //}

    imshow(winName, canvas);
    waitKey(1);
}


int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "ledge_detector");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
  ros::Subscriber LIDAR_sub = nh.subscribe("/os_cloud_node/points", 1, cloud_cb);
  image_transport::ImageTransport it_(nh);
  ros::Subscriber IMG_sub = nh.subscribe("/image_raw", 1, image_cb);
  // Create a ROS publisher for the output point cloud
  plane_pub = nh.advertise<PCLCloud>("model_plane_output", 1);
  ledge_pub = nh.advertise<PCLCloud>("model_ledge_output", 1);
  //cylinder_pub = nh.advertise<PCLCloud> ("cylinder_output", 1);
  cv::namedWindow(OPENCV_WINDOW);
  // Spin
  //ros::spin();
  ros::Rate r(10);
  // An image
    Mat3b img(720,1280, Vec3b(0, 255, 0));

    // Your button
    button = Rect(0, 50, img.cols, 100);
    button2 = Rect(0,200, img.cols, 100);


    button3 = Rect(img.cols*1/8,500, img.cols/4, img.cols/4);
    button4 = Rect(img.cols*3/8,500, img.cols/4, img.cols/4);;
    button5 = Rect(img.cols*5/8,500, img.cols/4, img.cols/4);

    // The canvas
    canvas = Mat3b(img.rows + button.height, img.cols, Vec3b(0, 0, 0));
    cv::Scalar overlayColor = cv::Scalar(50, 50, 50);
    cv::Mat overlayImage = cv::Mat(Size(img.cols/4, img.cols/4), CV_8UC3, overlayColor);

    Mat srcimg1 = imread("/home/av/ws_ugv_arm/src/target_localization/icon/1.png", -1);
    Mat srcimg2 = imread("/home/av/ws_ugv_arm/src/target_localization/icon/2.png", -1);
    Mat srcimg3 = imread("/home/av/ws_ugv_arm/src/target_localization/icon/3.png", -1);
    resize(srcimg1,srcimg1,Size(img.cols/4, img.cols/4));
    resize(srcimg2,srcimg2,Size(img.cols/4, img.cols/4));
    resize(srcimg3,srcimg3,Size(img.cols/4, img.cols/4));
    // Draw the button
    canvas(button)  = Vec3b(200, 200, 200);
    canvas(button2) = Vec3b(200, 200, 200);
    //canvas(button3) = Vec3b(200, 200, 200);
    //canvas(button4) = Vec3b(200, 200, 200);
    //canvas(button5) = Vec3b(200, 200, 200);
 

   
    Mat insetImage1(canvas, button3);
    srcimg1.copyTo(insetImage1);
    Mat insetImage2(canvas, button4);
    srcimg2.copyTo(insetImage2);
    Mat insetImage3(canvas, button5);
    srcimg3.copyTo(insetImage3);


    putText(canvas(button), buttonText, Point(button.width*0.35, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));
    putText(canvas(button2), buttonText2, Point(button.width*0.40, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));

    namedWindow(winName);
    setMouseCallback(winName, callBackFunc);

  while (ros::ok())
  {
       
    //srcimg1=0.5*srcimg1+0.1*overlayImage;
    //srcimg2=0.5*srcimg2+0.1*overlayImage;

    Mat insetImage1(canvas, button3);
    srcimg1.copyTo(insetImage1);
    Mat insetImage2(canvas, button4);
    srcimg2.copyTo(insetImage2);
    Mat insetImage3(canvas, button5);
    srcimg3.copyTo(insetImage3);
    imshow(winName, canvas);
    char key =  cv::waitKey(1);
    if(key == 'q')
    {
      destroyAllWindows();
      exit;
    }
    ros::spinOnce();
    r.sleep();
  }




}
