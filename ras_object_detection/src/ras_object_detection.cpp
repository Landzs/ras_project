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
  ros::Rate loop_rate(20);
  image_transport::ImageTransport it(n);
  image_transport::Publisher img_pub = it.advertise("camera_bgr_image", 1);

  VideoCapture cap(0);
  if(!cap.isOpened()){
    std::cout<<"Error opening the webcam"<<std::endl;
    return -1;
  }

  Mat cam_image;
  Mat image_clone;
  Mat hsv_image;
  //Mat channelHSV[3];
  Mat green_image;
  Mat red_image;
  Mat yellow_image;
  Mat orange_image;
  Mat purple_image;
  Mat blue_image;

  while(ros::ok()){

    bool frame_success = cap.read(cam_image);
    if(!frame_success){
      std::cout<<"Cannot read a frame from webcam"<<std::endl;
      break;
    }

    //Image operations
    cvtColor(cam_image, hsv_image, CV_BGR2HSV);
    image_clone = cam_image.clone();

    //split(hsv_image, channelHSV);

    //Separate the HSV using Hue as threshold for geting colors in image
    inRange(hsv_image, Scalar(40,160,90), Scalar(50,255,200), green_image);
    inRange(hsv_image, Scalar(1,160,90), Scalar(6,255,200), red_image);
    inRange(hsv_image, Scalar(16,160,90), Scalar(23,255,200), yellow_image);
    inRange(hsv_image, Scalar(7,160,90), Scalar(14,255,200), orange_image);
    inRange(hsv_image, Scalar(145,160,90), Scalar(179,255,200), purple_image);
    inRange(hsv_image, Scalar(95,160,90), Scalar(105,255,200), blue_image);



    //Results shown
    imshow("Green objects", green_image);
    imshow("Camera image", cam_image);
    imshow("HSV Output", hsv_image);






    if(waitKey(30)==27){
      std::cout<<"esc key pressed"<<std::endl;
      break;
    }

    //Convert the opencv image to Ros image and 
    //publish the ros image to RViz for visualization
    sensor_msgs::ImagePtr img_msg;
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cam_image).toImageMsg();
		img_pub.publish(img_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
