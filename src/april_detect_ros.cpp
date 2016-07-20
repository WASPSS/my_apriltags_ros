#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"

/*
void print_detection(AprilTags::TagDetection& detection) const {
   cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";
}*/
AprilTags::TagDetector* tag_detector_;
cv_bridge::CvImagePtr cv_ptr;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
      image_transport::ImageTransport it(n_);
      sub_ = it.subscribe("/ardrone/image_raw", 1, &SubscribeAndPublish::imageCallback, this);
      m_tagDetector=NULL;
      m_tagCodes = AprilTags::tagCodes36h11;
  
      m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
    }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cout << "Callback"<<endl;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::imshow("view", cv_ptr->image);    
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    cv::imshow("next", gray);
    cv::waitKey(1);
    cout<< "Calling the tag_detector"<<endl;
    
    std::vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(gray);
    //cout << detections.size() << " tags detected:" ;
    
    /*
    for (int i=0; i<detections.size(); i++) {
       print_detection(detections[i]);
    }
    */
  }

private:
  ros::NodeHandle n_;
  image_transport::Subscriber sub_;
  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltag_detector");
  SubscribeAndPublish SAPObject;
  cv::namedWindow("view");
  cv::namedWindow("next");
  cv::startWindowThread();
  cout << "Main instantiated"<<endl; 
 // image_transport::Subscriber sub = it.subscribe("/ardrone/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
  return 0;
}
