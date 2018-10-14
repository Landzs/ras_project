#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

#define DEBUG 0

using namespace cv;
using namespace std;

class maze_object
{
  // image containers
  Mat rgb_input, depth_input;

  /*
    Realsense camera intrinsic parameters. To obtain type 

          rostopic echo /camera/rgb/camera_info
      
    Source: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  */
  float fx = 616.344;
  float fy = 616.344;
  float cx = 314.855;
  float cy = 223.877;

  // Object Position
  float x_obj = 0;
  float y_obj = 0;
  float z_obj = 0;

  // Max and Min Depth in meters (0 and 255 respectively on depth image)
  float max_depth = 2.0;  
  float min_depth = 0.05; 
  

  public:

  // Subscriber callbacks
  void callback_inputDepth(const sensor_msgs::ImageConstPtr& );
  void callback_inputRGB(const sensor_msgs::ImageConstPtr& msg);

  // detecting regions of interest based on color 
  Rect detectColor(Scalar, Scalar);

  // Extract depth of detected object
  void extract_depth(Rect *, int);

  // Display Bounding box which has max area, i.e. dominant color
  void display_bBox(Rect *, int);
};


/*********************
  callback_inputDepth
**********************/
void maze_object::callback_inputDepth(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    depth_input = cv_bridge::toCvShare(msg, "32FC1")->image;
    imshow("Depth", depth_input);
    //imshow("Depth Image", cv_bridge::toCvShare(msg, "32FC1")->image);
    waitKey(30);
    cout<<"Depth at 0,0 is: "<<depth_input.at<float>(320,240)<<"\n";
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
  }
}

/*********************
  callback_inputRGB
**********************/
void maze_object::callback_inputRGB(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    rgb_input = cv_bridge::toCvShare(msg, "bgr8")->image;
    //imshow("RGB", rgb_input);
    //imshow("RGB Image", cv_bridge::toCvShare(msg, "bgr8")->image);
    //waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/*********************
  detectColor
**********************/
// int hmin, int smin, int vmin, int hmax, int smax, int vmax,
Rect maze_object::detectColor(Scalar min_val, Scalar max_val)
{
  Rect bBox;
  //std::cout<<"Entered detectColor \n";
  
  if(!rgb_input.empty())
  {
    //std::cout<<"rgb_imput is non empty \n";
    Mat img_clone = rgb_input.clone();
    Mat img_hsv, thresh_hsv, morph_opening;
  
    cvtColor(img_clone, img_hsv, COLOR_BGR2HSV);
    inRange(img_hsv, min_val, max_val, thresh_hsv);



    // Apply the specified morphology operation
	  int morph_operator = 2;		//OPENING
    int morph_elem = 2;			//ELLIPSE
    int morph_size = 3	;		//SIZE of Strel

    Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    
    morphologyEx( thresh_hsv, morph_opening, morph_operator, element );

    // Find Contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( morph_opening, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    if (contours.size() != 0)
  	{
      // find index of largest contour
      int ind_max = 0;
      double ctr_area = 0;
      double ctr_maxarea = 0;

      for ( int i = 0; i < contours.size(); i++)
		  {
        ctr_area = contourArea(contours[i]);
        if(ctr_area > ctr_maxarea)
        {
          ctr_maxarea = ctr_area;
          ind_max = i;
        }
      }

      // find Bounding Box of largest contour
      bBox = boundingRect(Mat(contours[ind_max]));

#if(DEBUG == 1)
      imshow("Thresholded image", thresh_hsv);

      imshow("Morphed image", morph_opening);
      waitKey(20);
#endif
    }
  }
  else
  {
    //std::cout<<"RGB is EMPTY \n";
  }

  return bBox;
}

/*********************
  display_bBox
**********************/
void maze_object::display_bBox(Rect *bBox, int ind)
{
  if(!rgb_input.empty())
  {
    Mat img_clone = rgb_input.clone();
    //rectangle( img_clone, bBox->tl(), bBox->br(), Scalar(0,255,0), 2, 8, 0 );
    rectangle( img_clone, bBox[ind].tl(), bBox[ind].br(), Scalar(0,255,0), 2, 8, 0 );

    imshow("Detected Object", img_clone);
    waitKey(20);
  }
}


/*********************
  extract_depth
**********************/
void maze_object::extract_depth(Rect *bBox, int ind)
{
  //cout<<"TRACE";
  int x_pos = int(bBox[ind].x + bBox[ind].width/2);
  int y_pos = int(bBox[ind].y + bBox[ind].height/2);
  //z_obj = depth_input[x_pos, y_pos];
  //cout<<"TRACE "<<x_pos<<"\t"<<y_pos<<"\n";
  //z_obj = depth_input.at<float>(y_pos, x_pos);
  //cout<<"Depth is :"<<z_obj<<"\n";
  //return 0.0;
}

/*********************
  MAIN
**********************/
int main(int argc, char **argv)
{
  maze_object object;
  int loop_frequency = 20;
  //Mat rgb_input, depth_input, rgb_output;
  Rect bBoxes[6];
  int bBox_area = 0;
  int max_bBox_area = 0;
  int max_bBox_ind = 0;

  ros::init(argc, argv, "ras_object_detection");
  ros::NodeHandle n;
  ros::Rate loop_rate(loop_frequency);
  image_transport::ImageTransport it(n);
  
  // namedWindow("view");
  // startWindowThread();
  
  image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image_raw", 1, &maze_object::callback_inputDepth, &object);
  image_transport::Subscriber rgb_sub = it.subscribe("/camera/rgb/image_rect_color", 1, &maze_object::callback_inputRGB, &object);
  
  while(ros::ok())
  {
    //std::cout<<"Entered While loop \n";
    // Call Color detection for all colors
    bBoxes[0] = object.detectColor(Scalar(40,160,90), Scalar(50,255,200));  // Green 
    bBoxes[1] = object.detectColor(Scalar(1,210,90), Scalar(6,255,160));    // Red
    bBoxes[2] = object.detectColor(Scalar(15,210,110), Scalar(22,255,190)); // Yellow
    bBoxes[3] = object.detectColor(Scalar(7,220,110), Scalar(13,255,205));  // Orange
    bBoxes[4] = object.detectColor(Scalar(142,45,80), Scalar(179,132,150)); // purple 
    bBoxes[5] = object.detectColor(Scalar(90,70,45), Scalar(101,255,150));  // blue 
    
    // find largest bounding box
    for( int i = 0; i < 6; i++)
		  {
        bBox_area = int(bBoxes[i].width * bBoxes[i].height);

        if(bBox_area > max_bBox_area)
        {
           max_bBox_area = bBox_area;
           max_bBox_ind = i;
        }
      }

      // Extract depth of largest object
      object.extract_depth(bBoxes, max_bBox_ind);

      //Display object with largest bBox 
      object.display_bBox(bBoxes, max_bBox_ind);

    ros::spinOnce();
  }

  //ros::spin();
  destroyWindow("view");
}