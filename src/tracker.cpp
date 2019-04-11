#include "tracker.h"

using namespace cv;
using namespace std;

const double NN_MATCH_RATIO = 0.75f; // nearest-neighbour matching ratio
const double RANSAC_THRESH  = 2.5f;  // RANSAC inlier threshold
//const Mat CAMERA_MAT = (Mat1d(3, 3) << 1158.03, 0.,      540.,
//                                         0.,      1158.03, 360.,
//                                         0.,      0.,      1.  );
//const Mat DIST_COEFFS = (Mat1d(1, 4) << 0., 0., 0., 0.);
//const Mat CAMERA_MAT = (Mat1d(3, 3) << 2373.287987, 0.000000, 1193.920583,
//				       0.000000, 2371.300868, 1096.973512,
//				       0.000000, 0.000000, 1.000000);
//
//const Mat DIST_COEFFS = (Mat1d(1, 4) << -0.299861, 0.247257, 0.003340, -0.000833, 0.000000);


image_transport::Subscriber img_sub;
image_transport::Subscriber img_sub2;
image_transport::Publisher img_pub;
ros::Publisher pose_pub;
sensor_msgs::ImagePtr msg;
Mat outImg;

Tracker::Tracker(Ptr<Feature2D> _detector, Ptr<DescriptorMatcher> _matcher)
{
    detector = _detector;
    matcher = _matcher;
    nmatches = 0;
}


void Tracker::setReferenceImg(const Mat img)
{
    // set reference image to be tracked
    ref_img = img;

    // calculate keypoints and descriptors for reference image
    detector->detectAndCompute(ref_img, Mat(), ref_kp, ref_desc);

    // set corners of reference image
    ref_corners[0] = Point(0,0);
    ref_corners[1] = Point( ref_img.cols, 0 );
    ref_corners[2] = Point( ref_img.cols, ref_img.rows );
    ref_corners[3] = Point( 0, ref_img.rows );

    // set corners of reference image in 3D
    ref_corners_3d[0] = Point3f(-ref_img.cols/2.,-ref_img.rows/2., 0.);
    ref_corners_3d[1] = Point3f(-ref_img.cols/2., ref_img.rows/2., 0.);
    ref_corners_3d[2] = Point3f( ref_img.cols/2., ref_img.rows/2., 0.);
    ref_corners_3d[3] = Point3f( ref_img.cols/2.,-ref_img.rows/2., 0.);

    // set center of reference image
    ref_center[0] = Point(ref_img.cols/2., ref_img.rows/2.);

    cout << ref_corners << endl;
}

void Tracker::setCameraParams(Mat cm, Mat dc)
{
		camera_matrix = cm;
		dist_coeffs = dc;
}

void Tracker::calcMatches()//const Mat frame)
{
    Mat frame = frm_img;
    frm_kp.clear();
    matches.clear();
    ref_matched.clear();
    frm_matched.clear();
    // detect keypoints and descriptors for frame
    detector->detectAndCompute(frame, Mat(), frm_kp, frm_desc);

    // make sure descriptor Mats are not empty
    if (!ref_desc.empty() && !frm_desc.empty())
    {
        matcher->knnMatch(ref_desc, frm_desc, matches, 2);
    }
    else
    {
        cout << "Warning: empty decriptor Mat." << endl;
        nmatches = 0;
        return;
    }

    // perform ratio test described in Lowe's paper and store good matches
    cout << "clear" << endl;
    for(unsigned i = 0; i < matches.size(); i++)
    {
        if(matches[i][0].distance < NN_MATCH_RATIO * matches[i][1].distance)
        {
            ref_matched.push_back( ref_kp[matches[i][0].queryIdx].pt );
            frm_matched.push_back( frm_kp[matches[i][0].trainIdx].pt );
        }
    }

    // set nmatches and frm_img
    nmatches = ref_matched.size();
    frm_img = frame;
}

void Tracker::getRelativePose()
{
    frm_corners.clear();
    frm_center.clear();

    // calculate homography transform
    Mat H;
    H = findHomography(ref_matched, frm_matched, RANSAC, RANSAC_THRESH);

    // if H empty return rvec full of -100
    if (H.empty())
    {
        cout << "Empty Homography" << endl;
        rvec.at<double>(0,0) = -100.;
        rvec.at<double>(1,0) = -100.;
        rvec.at<double>(2,0) = -100.;
        return;
    }

    // perform perspective transform
    perspectiveTransform(ref_corners, frm_corners, H);
    perspectiveTransform(ref_center, frm_center, H);

    // solve PnP using iterative LMA
    solvePnP(ref_corners_3d, frm_corners, camera_matrix, dist_coeffs, rvec, tvec, 1, 1);
}

void Tracker::drawMyBoundingBox()
{
    line( frm_img, frm_corners[0], frm_corners[1], Scalar( 0, 255, 0), 4 );
    line( frm_img, frm_corners[1], frm_corners[2], Scalar( 0, 255, 0), 4 );
    line( frm_img, frm_corners[2], frm_corners[3], Scalar( 0, 255, 0), 4 );
    line( frm_img, frm_corners[3], frm_corners[0], Scalar( 0, 255, 0), 4 );
}

void Tracker::drawFrameAxes()
{
    // idk why this makes it look right when drawn
    Mat temp_rvec(3,1,cv::DataType<double>::type);
    temp_rvec.at<double>(0,0) = rvec.at<double>(0,0)*-1.;
    temp_rvec.at<double>(1,0) = rvec.at<double>(1,0)*-1.;
    temp_rvec.at<double>(2,0) = rvec.at<double>(2,0);

    // reference axis
    vector<Point3f> axis(4);
    axis[0] = Point3f(0,  0,  0);
    axis[1] = Point3f(100, 0, 0);
    axis[2] = Point3f(0, 100, 0);
    axis[3] = Point3f(0, 0, 100);

    // project axis onto object in frame
    vector<Point2f> projectedPts;
    projectPoints(axis, temp_rvec, tvec, camera_matrix, dist_coeffs, projectedPts);

    // draw axis lines
    line( frm_img, projectedPts[0], projectedPts[1], Scalar( 255, 0, 0), 3 ); // x blue
    line( frm_img, projectedPts[0], projectedPts[2], Scalar( 0, 255, 0), 3 ); // y green
    line( frm_img, projectedPts[0], projectedPts[3], Scalar( 0, 0, 255), 3 ); // z red
}

Ptr<Feature2D> Tracker::getDetector() { return detector; }

int Tracker::getMatches() { return nmatches; }

Mat Tracker::getFrameDescriptors() { return frm_desc; }

vector<KeyPoint> Tracker::getFrameKeyPoints() { return frm_kp; }

Mat Tracker::getPoseTVec() { return tvec; }

Mat Tracker::getPoseRVec() { return rvec; }

geometry_msgs::Pose Tracker::getPose()
{
    geometry_msgs::Pose p;
    double rx = rvec.at<double>(0,0);
    double ry = rvec.at<double>(1,0);
    double rz = rvec.at<double>(2,0);
    double theta = (double)(sqrt(rx*rx + ry*ry + rz*rz));

    // normalize axis and convert theta to radians
    rx /= theta;
    ry /= theta;
    rz /= theta;
    theta *= 180/M_PI;

    // axis-angle to quaternion and add to pose
    double s = sin(theta/2);
    p.orientation.x = rx * s;
    p.orientation.y = ry * s;
    p.orientation.z = rz * s;
    p.orientation.w = cos(theta/2);

    // add relative position to pose
    p.position.x = tvec.at<double>(0,0);;
    p.position.y = tvec.at<double>(1,0);;
    p.position.z = tvec.at<double>(2,0);;

    return p;
}

Mat Tracker::getCurrentFrame() { return frm_img; }

void Tracker::acquirePixelImage(const sensor_msgs::ImageConstPtr& image)
{
    cout << "Received image!" << endl;
    //cv_bridge::CvImagePtr cv_ptr;
    //imageIn_ = image;
		cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //imageOut_ = cv_bridge::toCvCopy(imageIn_, sensor_msgs::image_encodings::BGR8);
				cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frm_img = cv_ptr->image.clone(); // get a new frame from pixelink
    calcMatches();

    cout << getMatches() << endl;
    if (getMatches() > 20)
    {
        getRelativePose();

        cout << "rvec: " << getPoseRVec()*180./M_PI << endl; // radians
        cout << "tvec: " << getPoseTVec()*0.00035 << endl; // mm

        drawMyBoundingBox();
        drawFrameAxes();
    }

    // show image
    cv::resize(frm_img, outImg, cv::Size(), 0.33, 0.33);

    // publish image
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outImg).toImageMsg();
    img_pub.publish(msg);
}

void testCallback(const sensor_msgs::ImageConstPtr& image)
{
    //cout << "Received image!" << endl;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat myimg = cv_ptr->image.clone(); // get a new frame from pixelink

    // show image
    imshow("pub", myimg);
    waitKey(2);
}

int main(int argc, char** argv)
{

    ros::init(argc,argv,"object_tracker");

    // create detector and orb_matcher
    Ptr<Feature2D> detector = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

		// camera params
		Mat CAMERA_MAT = (Mat1d(3, 3) << 2373.287987, 0.000000, 1193.920583,
						       0.000000, 2371.300868, 1096.973512,
						       0.000000, 0.000000, 1.000000);

		Mat DIST_COEFFS = (Mat1d(1, 4) << -0.299861, 0.247257, 0.003340, -0.000833, 0.000000);

    // initialize reference image
    Mat ref_img;
		std::string img_path = ros::package::getPath("object_tracking_ros") + "/imgs/ida_pixelink.jpg";
    //ref_img = imread("/home/travisdriver/catkin_ws/src/object_tracking_ros/imgs/ida_pixelink.jpg");
		std::cout << img_path << std::endl;
		ref_img = imread(img_path);
		if(ref_img.empty())
		{
			std::cout << "empty reference image" << std::endl;
			return 0;
		}

    // initialize tracker
    Tracker tracker(detector, matcher);
    tracker.setReferenceImg(ref_img);
		tracker.setCameraParams(CAMERA_MAT, DIST_COEFFS);

    // attempt to read pixelink image
    cout << "test" << endl;
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    img_sub = it.subscribe("/pixelink/image", 1, &Tracker::acquirePixelImage, &tracker);
    img_pub = it.advertise("/object_tracking_ros/image", 1);
    img_sub2 = it.subscribe("object_tracking_ros/image", 1, testCallback);
    pose_pub = nh.advertise<geometry_msgs::Pose>("object_tracking_ros/camera_pose",1);

    // run tracker and publish results
    ros::spin();

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
