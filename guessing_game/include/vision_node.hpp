// ROS library
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <ros/duration.h>

// Standard libraries
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <time.h>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <vector>
#include <algorithm>

// ArUco (for marker tracking)
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Transforms
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Messages and services
#include <beyoNAOnce/BlobCenter.h>
#include <beyoNAOnce/Marker.h>

#include <beyoNAOnce/blob_center.h>
#include <beyoNAOnce/marker_pos.h>

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

using namespace std;
using namespace cv;
using namespace aruco;

// Marker size in meters
#define MARKER_SIZE 0.05


bool stop_thread=false;

void spinThread()
{
    while(!stop_thread)
    {
        ros::spinOnce();
    }
}


// OpenCV windows names
static const char image_raw[] = "NAO Camera (raw image)";

// Structures defined for vision
struct camera_params {
    Mat distortion;
    Mat camera;
    float focalLength;
    float horizontalFOV;
    float verticalFOV;
};

// Helper functions
/**
 * Compares contours regarding area sizes
 * @param contour_1 first compared contour, vector<cv::Point> type,
 * @param contour_2 second compared contour, vector<cv::Point> type.
 */
bool compare_area(vector<cv::Point> contour_1,vector<cv::Point> contour_2);
/**
 * Compares contours regarding sizes
 * @param contour_1 first compared contour, vector<cv::Point> type,
 * @param contour_2 second compared contour, vector<cv::Point> type.
 */
bool compare_size(vector<cv::Point> contour_1,vector<cv::Point> contour_2);

/**
 * Compares contours regarding y coordinates of the center
 * @param contour_1 first compared contour, vector<cv::Point> type,
 * @param contour_2 second compared contour, vector<cv::Point> type.
 */
bool compare_y(vector<cv::Point> contour_1,vector<cv::Point> contour_2);

/**
 * Compares contours regarding x coordinates of the center
 * @param contour_1 first compared contour, vector<cv::Point> type,
 * @param contour_2 second compared contour, vector<cv::Point> type.
 */
bool compare_x(vector<cv::Point> contour_1,vector<cv::Point> contour_2);


class VisionPart{
public:
    // Constructors & Deconstructor
    VisionPart();
    ~VisionPart()
    {
    };

    // Preprocessing functions
    /**
    * CLAHE algorithm.
    * @param img            image which histogram should be normalized, Mat type.
    * @param ClipLimit      algorithm parameter on which the "grain" depends, float type.
    * @param TilesGridSize  algorithm parameter on which the size of the tiles depends, float type.
    */
    Mat Clahe(Mat img, float ClipLimit, int TilesGridSize);

    /**
    * Blurring algorithm.
    * @param img            image which should be blurred, Mat type.
    * @param type           type of the blur used, 0 - Gaussian, 1 - Median blur, 2 - Bilateral Filter.
    */
    Mat Blur(Mat img, int type);

    /**
    * Function rotating the image.
    * @param img            image that will be rotated, Mat type,
    * @param center         center of the rotation, Point2f type.
    */
    Mat rotate(Mat img,Point2f center, double angle);

    // Detection
    /**
     * Extract given color in the image
     * @param img   image from which the markers should be extracted, Mat type.
     * @param color name of the color that should be extracted
     */
    vector<cv::Point> extractColor(Mat img, string color);

    /**
     * Extract biggest color blob and publishes the message
     * @param img   image from which the markers should be extracted, Mat type.
     */
     bool detectBlob(Mat img);

    /**
     * Extract ArUco markers in the image
     * @param img   image from which the markers should be extracted, Mat type.
     */
    vector<Marker> extractMarkers(Mat img);

    /**
     * Extract ArUco marker with the given id in the image
     * @param img   image from which the markers should be extracted, Mat type,
     * @param id    id of the marker, int type.
     */
    vector<Mat> extractMarker(Mat img, int id);
    /**
     * Extract lowest ID ArUco marker in the image
     * @param img   image from which the markers should be extracted, Mat type.
     */
    bool extractFirstMarker(Mat img);
    /**
     * Callback function for the camera image
     * @param msg   pointer on an image received from the camera.
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    // Service Functions
    bool getBlobCenter(beyoNAOnce::blob_center::Request  &req,beyoNAOnce::blob_center::Response &res);

    bool getMarkerCenter(beyoNAOnce::marker_pos::Request  &req,beyoNAOnce::marker_pos::Response &res);

private:
    bool show = true;
    bool process_image = false;
    int sort_type = 3;
    pthread_mutex_t count_mutex = PTHREAD_MUTEX_INITIALIZER;

    boost::thread *spin_thread;

    // CAMERA SETUP
    camera_params top_camera, bottom_camera;
    CameraParameters cameraParameters;
    bool bottom_camera_use = false;
    Mat current_image;

    // ArUco
    MarkerDetector MDetector;

    // Subscribers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    // Publishers
    ros::Publisher markers_pub; // Markers detected
    ros::Publisher blob_center_pub; // Blob center

    // Services
    ros::ServiceServer marker_service;
    ros::ServiceServer blob_center_service;

    // OpenCV windows names
    string image_raw = "NAO Camera (raw image)";

    // COLOR DETECTION PARAMETERS

    // Minimum value and saturation for the detection
    int min_sat=100, min_value=100;
    //vector <string> colors = {"Yellow","Green","Blue","Red"};
    // Map of all used colors
    map < string, vector<cv::Scalar> > color_map{
        {"Blue",        {Scalar(  100,  min_sat, min_value), Scalar( 125, 255, 255)}},
        {"Red",         {Scalar(   0,  min_sat, min_value), Scalar(  15, 255, 255)}},
        //{"Orange",      {Scalar(  10,   100,    min_value), Scalar(  20, 255, 255)}},
        {"Yellow",      {Scalar(  20,  min_sat, min_value), Scalar(  30, 255, 255)}},
        {"Green",       {Scalar(  60,  min_sat, min_value), Scalar(  85, 255, 255)}},
        //{"Blue",        {Scalar(  90,  min_sat, min_value), Scalar( 100, 255, 255)}},
        //{"White",       {Scalar(   0,   0,      255),       Scalar( 255,   0, 255)}},
        //{"Purple",      {Scalar( 140,  min_sat, min_value), Scalar( 160, 255, 255)}},
        };
};
