#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>

//#include "tracker.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/format.hpp>

//#ifndef TRACKER_H
//#define TRACKER_H

using namespace cv;
//using namespace cv::xfeatures2d;
using namespace std;

class Tracker
{
public:
    // constructor
    Tracker(Ptr<Feature2D> _detector, Ptr<DescriptorMatcher> _matcher);

    // methods
    void initializeSubscribers();
    Ptr<Feature2D> getDetector();
    void setReferenceImg(const Mat img);
    void calcMatches();//const Mat frame);
    int getMatches();
    void getRelativePose();
    Mat getFrameDescriptors();
    vector<KeyPoint> getFrameKeyPoints();
    void framePerspectiveTransform();
    Mat getPoseTVec();
    Mat getPoseRVec();
    geometry_msgs::Pose getPose();
    void drawMyBoundingBox();
    void drawFrameAxes();
    Mat getCurrentFrame();
    void acquirePixelImage(const sensor_msgs::ImageConstPtr& msg);
    void setCameraParams(Mat cm, Mat dc);
protected:
    cv_bridge::CvImagePtr cv_ptr;
    Ptr<Feature2D> detector;
    Ptr<DescriptorMatcher> matcher;
    Mat ref_img, ref_desc;
    Mat frm_img, frm_desc;
    vector<KeyPoint> ref_kp;
    vector<KeyPoint> frm_kp;
    vector<Point2f> ref_corners{4};
    vector<Point3f> ref_corners_3d{4};
    vector<Point2f> ref_center{1};
    vector<Point2f> frm_corners{4};
    vector<Point2f> frm_center{1};
    vector<vector<DMatch>> matches;
    vector<Point2f> ref_matched, frm_matched;
    int nmatches;
    Mat rvec{3,1,cv::DataType<double>::type};
    Mat tvec{3,1,cv::DataType<double>::type};
    Mat camera_matrix, dist_coeffs;
};

//#endif
