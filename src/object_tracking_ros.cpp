#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
//#include <opencv2/xfeatures2d.hpp>
#include <math.h>
#include <vector>
#include <iostream>
#include <iomanip>

#include "tracker.h"
//#include <ros/ros.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <boost/format.hpp>

using namespace cv;
using namespace std;

cv_bridge::CvImagePtr cv_ptr;
Mat frame;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    //cout << "Received image!" << endl;
    //cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame = cv_ptr->image; // get a new frame from camera
    //cv::imshow("test", frame);
    //cv::waitKey(2);

}

//void imageCallback(const sensor_msgs::ImageConstPtr& msg)
//{
//    cout << "here" << endl;
//    cv_bridge::CvImagePtr cv_ptr;
//    try
//    {
//        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//    const cv::Mat &image = cv_ptr->image;
//    cv::imshow("test", image);
//    cv::waitKey(0);
//}

/*
// perform optical flow on environment features
def perform_optical_flow(previous_frame, previous_fp, current_frame):
{

}
*/

//==============================================================================
//                                  MAIN
//==============================================================================

int main(int argc, char** argv)
{
    
    ros::init(argc,argv,"object_tracker");

    // create detector and orb_matcher
    Ptr<Feature2D> detector = ORB::create();
    //Ptr<Feature2D> detector = cv::xfeatures2d::SIFT::create(500);
    //Ptr<SURF> detector = SURF::create(500);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

    // initialize reference image
    Mat ref_img;
    ref_img = imread("/home/travisdriver/catkin_ws/src/object-tracking-master/imgs/ida_pixelink.jpg");
    imshow("ref",ref_img);
    waitKey(0);

    // initialize tracker
    Tracker tracker(detector, matcher);
    tracker.setReferenceImg(ref_img);

    // attempt to read pixelink image
    cout << "test" << endl;
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub;
    cout << "sub" << endl;
    img_sub = it.subscribe("/pixelink/image", 1, &Tracker::acquirePixelImage, &tracker);
    ros::spinOnce();

    //VideoCapture cap(0); // open the default camera
    //if(!cap.isOpened())  // check if we succeeded
    //    return -1;

    while(ros::ok())
    {
	ros::spinOnce();
        //tracker.calcMatches();//frame);

        //cout << tracker.getMatches() << endl;
        //if (tracker.getMatches() > 50)
        //{
        //    tracker.getRelativePose();

        //    cout << "rvec: " << tracker.getPoseRVec()*180./M_PI << endl;
        //    cout << "tvec: " << tracker.getPoseTVec() << endl;

        //    tracker.drawMyBoundingBox();
        //    tracker.drawFrameAxes();

        //}
        
	//Mat frame = tracker.getCurrentFrame();
	//cv::Size s = frame.size();
	//int rows = s.height;
	//cout << "result" << endl;
	//if (rows > 0)
	//{
        //    imshow("Tracking", frame);
        //    if(waitKey(30) >= 0) break;
	//}
	//else
	//{
	//    cout << "Empty image" << endl;
	//}
	Mat outImg;
	cv::resize(tracker.getCurrentFrame(), outImg, cv::Size(), 0.33, 0.33);
    	imshow("test", outImg);
    }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
