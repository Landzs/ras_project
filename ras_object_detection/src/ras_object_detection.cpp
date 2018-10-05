#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <opencv2/highgui.hpp>
#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;

int main(int argc, char **argv){
  ros::init(argc, argv, "ras_object_detection");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  VideoCapture cap(0);
  if(!cap.isOpened()){
    std::cout<<"Error opening the webcam"<<std::endl;
    return -1;
  }

  Mat frame;

  while(ros::ok()){

    bool frame_success = cap.read(frame);
    if(!frame_success){
      std::cout<<"Cannot read a frame from webcam"<<std::endl;
      break;
    }
    imshow("Webcam image", frame);

    if(waitKey(30)==27){
      std::cout<<"esc key pressed"<<std::endl;
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
