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
#include <target_localization/bbox_array.h>
#include <target_localization/bbox.h>
#include <darknet_ros/bbox_array.h>
#include <darknet_ros/bbox.h>
#include <sensor_msgs/LaserScan.h>
ros::Publisher plane_pub;
ros::Publisher ledge_pub;
using namespace cv;
using namespace std;

Mat3b canvas;
string buttonText;
string buttonText2;
String winName;
Rect button, button2, button3, button4, button5;
  bool item1_found,item2_found,item3_found;
void onMouse(int event, int x, int y, int flags, void* userdata)
{
  if (event == EVENT_LBUTTONDOWN)
  {
      if (button.contains(Point(x, y)))
      {
        
          cout << "start search mission\n" << endl;

          canvas(button) = Vec3b(50, 50, 50);
          putText(canvas(button), buttonText, Point(button.width*0.35, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));
          putText(canvas(button2), buttonText2, Point(button.width*0.40, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));
          // to be added here
          // run system script to go to fixed points

      }
      else if (button2.contains(Point(x, y)))//ponizej to co ma sie wykonac po nacisnieciu klawisza
      {
          cout << "Cancel mission clicked\n" << endl;
          // to be added here
          // run system script to cancel all events  kill them all
          item1_found=false;
          item2_found=false;
          item3_found=false;
          putText(canvas(button), buttonText, Point(button.width*0.35, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));
          putText(canvas(button2), buttonText2, Point(button.width*0.40, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));
      }
      else if (button3.contains(Point(x, y)))//ponizej to co ma sie wykonac po nacisnieciu klawisza
      {
          
          cout << "Go place 1 pick item 1\n" << endl;
          // this icon only lights up if item has been found
          // go to pre-recored are to sweep the item
          
      }
      else if (button4.contains(Point(x, y)))//ponizej to co ma sie wykonac po nacisnieciu klawisza
      {
          
          cout << "Go place 2 pick item 2\n" << endl;
          // this icon only lights up if item has been found
          // go to pre-recored are to sweep the item
      }
      else if (button5.contains(Point(x, y)))//ponizej to co ma sie wykonac po nacisnieciu klawisza
      {
          
          cout << "Go place 2 pick item 2\n" << endl;
          // this icon only lights up if item has been found
          // go to pre-recored are to sweep the item
      }
  }
  //if (event == EVENT_LBUTTONUP)
  //{
  //rectangle(canvas, button, Scalar(200, 200, 200), 2);
  //}

  // imshow(winName, canvas);
  //waitKey(1);
}



class target_localization_gui
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber sub_object_detector_array;
  ros::Subscriber sub_2D_LIDAR_scan;
  std::string item1,item2,item3;

  //static void callBackFunc(int event,int x,int y, int flags,void* param);
  //static void callBackFuncStatic( int event, int x, int y, int flags, void* that );
  //void callBackFunc( int event, int x, int y, int flags);


public:
  target_localization_gui()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1, &target_localization_gui::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    sub_object_detector_array = nh_.subscribe("/YOLO_bboxes",1, &target_localization_gui::object_detect_cb, this);
    sub_2D_LIDAR_scan = nh_.subscribe("/scan",1, &target_localization_gui::lidar_2d_scan_cb, this);

    buttonText="Search For Target";
    buttonText2="Reset";
    winName = "Simple GUI";

    item1="bottle";
    item2="mouse";
    item2="scissors";
    item1_found=false;
    item2_found=false;
    item3_found=false;
    ros::Rate r(10);




    Mat3b img(720,1280, Vec3b(0, 255, 0));

    // Your button
    button = Rect(0, 50, img.cols, 100);
    button2 = Rect(0,200, img.cols, 100);


    button3 = Rect(img.cols*1/8-50,400, img.cols/4, img.cols/4);
    button4 = Rect(img.cols*3/8,400, img.cols/4, img.cols/4);;
    button5 = Rect(img.cols*5/8+50,400, img.cols/4, img.cols/4);

    // The canvas
    canvas = Mat3b(img.rows + button.height, img.cols, Vec3b(0, 0, 0));
    cv::Scalar overlayColor = cv::Scalar(50, 50, 50);
    cv::Mat overlayImage = cv::Mat(Size(img.cols/4, img.cols/4), CV_8UC3, overlayColor);
    cv::Mat overlaybg = cv::Mat(Size(img.cols/4, img.cols/4), CV_8UC3, overlayColor);
    
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
 

   
    // Mat insetImage1(canvas, button3);
    // srcimg1.copyTo(insetImage1);
    // Mat insetImage2(canvas, button4);
    // srcimg2.copyTo(insetImage2);
    // Mat insetImage3(canvas, button5);
    // srcimg3.copyTo(insetImage3);


    putText(canvas(button), buttonText, Point(button.width*0.35, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));
    putText(canvas(button2), buttonText2, Point(button.width*0.40, button.height*0.7), FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0, 0));

    namedWindow(winName,WINDOW_NORMAL);
    setWindowProperty (winName, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
    //setMouseCallback(winName, this->callBackFunc, (void*)this);
    // cv::setMouseCallback(winName, callBackFunc,(void*)this);
    cv::setMouseCallback(winName, onMouse);


    while (ros::ok())
    {
      std::cout<<"Found "<<item1_found<< endl;
      //
      if(item1_found)
      {
        Mat insetImage1(canvas, button3);
        srcimg1.copyTo(insetImage1);
      }
      else 
        //srcimg1=0.5*srcimg1+0.1*overlayImage;
      {
        Mat insetImage1(canvas, button3);
        overlaybg.copyTo(insetImage1);
      }


      if(item2_found)
      {
        Mat insetImage2(canvas, button4);
        srcimg2.copyTo(insetImage2);
      }
      else
      //  srcimg2=0.5*srcimg2+0.1*overlayImage;
       {
        Mat insetImage2(canvas, button4);
        overlaybg.copyTo(insetImage2);
      }

      if(item3_found)
      {
        Mat insetImage3(canvas, button5);
        srcimg3.copyTo(insetImage3);
      }
      else
        //srcimg3=0.5*srcimg3+0.1*overlayImage;
      {
        Mat insetImage3(canvas, button5);
        overlaybg.copyTo(insetImage3);
      }


      imshow(winName, canvas);
      char key =  cv::waitKey(1);
      if( key == 27 )
            return ;
      
      ros::spinOnce();
      r.sleep();
    }


  }

  ~target_localization_gui()
  {
   
    destroyAllWindows();
  }


  void lidar_2d_scan_cb(const sensor_msgs::LaserScan::ConstPtr msg)
  {

  }
  void object_detect_cb(const darknet_ros::bbox_array::ConstPtr msg  )
  {
      //This pipeline is to subscibe the YOLO message with 
      // string Class
      // float32 prob
      // uint32 xmin
      // uint32 ymin
      // uint32 xmax
      // uint32 ymax
      // uint32 xmid
      // uint32 ymid
    for(int i =0; i< msg->bboxes.size(); i++)
    {
      
      if( msg->bboxes[i].Class.find(item1)!= string::npos)
      {  item1_found=true;
      }
      if( msg->bboxes[i].Class.find(item2)!= string::npos)
        item2_found=true;
    
      if( msg->bboxes[i].Class.find(item3)!= string::npos)
        item3_found=true;    

      //std::cout<<"Found "<<msg->bboxes[i].Class<< endl;
      //printf("%s\n",msg->bboxes[0].Class);
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  { // routing not in use at momemnt
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

    //cv::waitKey(1);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }




};

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "target_localization_gui");
  target_localization_gui tl;
  return 0;
}
