/** @file vision_node.cpp
 *  @brief File storing definitions of the vision functions.
 */

#include "vision_node.hpp"

// HELPER FUNCTIONS
bool compare_area(vector<cv::Point> contour_1,vector<cv::Point> contour_2){
    // rotated rectangle
    RotatedRect boundRect1 = minAreaRect(Mat(contour_1));
    RotatedRect boundRect2 = minAreaRect(Mat(contour_2));
    // calculate area
    float area_1 = boundRect1.size.width*boundRect1.size.height;
    float area_2 = boundRect2.size.width*boundRect2.size.height;

    return area_1 < area_2;
}

bool compare_size(vector<cv::Point> contour_1,vector<cv::Point> contour_2){
  return contour_1.size() < contour_2.size();
}

bool compare_y(vector<cv::Point> contour_1,vector<cv::Point> contour_2){
    // rotated rectangle
    RotatedRect boundRect1 = minAreaRect(Mat(contour_1));
    RotatedRect boundRect2 = minAreaRect(Mat(contour_2));
    // calculate area
    float point_1 = boundRect1.center.y;
    float point_2 = boundRect2.center.y;

    return point_1 < point_2;
}

bool compare_x(vector<cv::Point> contour_1,vector<cv::Point> contour_2){
    // rotated rectangle
    RotatedRect boundRect1 = minAreaRect(Mat(contour_1));
    RotatedRect boundRect2 = minAreaRect(Mat(contour_2));
    // calculate area
    float point_1 = boundRect1.center.x;
    float point_2 = boundRect2.center.x;

    return point_1 < point_2;
}

VisionPart::VisionPart()
    : it_(nh_)
{
    top_camera.distortion = Mat(1, 5, CV_32FC1);
    top_camera.distortion.at<float>(0,0) = -0.066494;
    top_camera.distortion.at<float>(0,1) =  0.095481;
    top_camera.distortion.at<float>(0,2) = -0.000279;
    top_camera.distortion.at<float>(0,3) =  0.002292;
    top_camera.distortion.at<float>(0,4) =  0;

    top_camera.camera = Mat(3, 3, CV_32FC1, 0.0);
    top_camera.camera.at<float>(0,0) = 551.543059;
    top_camera.camera.at<float>(1,1) = 553.736023;
    top_camera.camera.at<float>(2,2) = 1.0;
    top_camera.camera.at<float>(0,2) = 327.382898;
    top_camera.camera.at<float>(1,2) = 225.026380;

    top_camera.focalLength = 581.25f;
    top_camera.horizontalFOV = 0.83147486; // 47.64 degrees in radians
    top_camera.verticalFOV   = 1.0641272;  // 60.97 degrees in radians

    bottom_camera.distortion = Mat(1, 5, CV_32FC1);
    bottom_camera.distortion.at<float>(0,0) = -0.064876;
    bottom_camera.distortion.at<float>(0,1) =  0.061252;
    bottom_camera.distortion.at<float>(0,2) =  0.003828;
    bottom_camera.distortion.at<float>(0,3) = -0.005511;
    bottom_camera.distortion.at<float>(0,4) =  0;

    bottom_camera.camera = Mat(3, 3, CV_32FC1, 0.0);
    bottom_camera.camera.at<float>(0,0) = 558.570339;
    bottom_camera.camera.at<float>(1,1) = 556.122943;
    bottom_camera.camera.at<float>(2,2) = 1.0;
    bottom_camera.camera.at<float>(0,2) = 308.885375;
    bottom_camera.camera.at<float>(1,2) = 247.600724;

    bottom_camera.focalLength = 573.5f;
    bottom_camera.horizontalFOV = 0.83147486; // 47.64 degrees in radians
    bottom_camera.verticalFOV   = 1.0641272;  // 60.97 degrees in radians

    // Subscribe to input video feed and publish output video feed
    if (bottom_camera_use){
        cameraParameters.setParams(bottom_camera.camera, bottom_camera.distortion,Size(640,480));
        cameraParameters.resize( Size(640,480));
        // Subscribe to input video feed and publish output video feed
        image_sub = it_.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1, &VisionPart::imageCb, this);
    }
    else{
        cameraParameters.setParams(top_camera.camera, top_camera.distortion,Size(640,480));
        cameraParameters.resize( Size(640,480));
        // Subscrive to input video feed and publish output video feed
        image_sub = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &VisionPart::imageCb, this);
    }

    // SERVICES
    blob_center_service = nh_.advertiseService("/beyoNAOnce/BlobCenter",&VisionPart::getBlobCenter,this);
    marker_service = nh_.advertiseService("/beyoNAOnce/MarkerPos",&VisionPart::getMarkerCenter, this);

    //MESSAGES
    blob_center_pub = nh_.advertise<beyoNAOnce::BlobCenter>("/beyoNAOnce/blob_center", 1000);
    markers_pub = nh_.advertise<beyoNAOnce::Marker>("/beyoNAOnce/marker_pos", 1000);

    stop_thread=false;
    spin_thread=new boost::thread(&spinThread);
}

// Preprocessing functions

Mat VisionPart::Clahe(Mat img, float ClipLimit, int TilesGridSize){
    // normalize image contrast & luminance
    Mat norm_img;
    cvtColor(img, norm_img, CV_BGR2Lab);

    // Extract the L channel
    std::vector<Mat> lab_planes(3);
    split(norm_img, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(ClipLimit);
    clahe->setTilesGridSize(Size(TilesGridSize,TilesGridSize));
    Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    merge(lab_planes, norm_img);
    cvtColor(norm_img, norm_img, CV_Lab2BGR);
    if (show)
      imshow("CLAHE", norm_img);
    return norm_img;
}

Mat VisionPart::Blur(Mat img, int type){
    Mat blurred;
    // Filter image without losing borders
    switch(type){
        case 0:
            GaussianBlur(img,blurred,Size( 5, 5),0,0);
        break;
        case 1:
            medianBlur(img,blurred, 5);
        break;
        case 2:
            bilateralFilter(img,blurred,5,10,2);
        break;
    }
    return blurred;
}

Mat VisionPart::rotate(Mat img,Point2f center, double angle){
  Mat rot = getRotationMatrix2D(center, angle, 1);
  Mat result;
  warpAffine(img, result, rot, img.size());
  if (show)
    imshow("Rotation", result);
  return result;
}

bool VisionPart::getBlobCenter(beyoNAOnce::blob_center::Request  &req,beyoNAOnce::blob_center::Response &res){
  if (current_image.empty())
      return false;
  pthread_mutex_lock( &this->count_mutex );
  process_image = true;
  vector<cv::Point> temp, max_size;
  string max_arg;
  for(std::map<string,vector<cv::Scalar> >::iterator iter = color_map.begin(); iter != color_map.end(); ++iter){
    temp = (extractColor(current_image,iter->first));
    if(temp.size() > max_size.size()){
      max_size = temp;
      max_arg = iter->first;
    }
  }
  RotatedRect boundRect = minAreaRect(Mat(max_size));
  float area = boundRect.size.width*boundRect.size.height;
  if (area > current_image.rows*current_image.cols*0.1)
    res.valid = true;
  else
    res.valid = false;
  Point2f center = boundRect.center;
  res.color = max_arg;
  res.x = center.x;
  res.y = center.y;
  pthread_mutex_unlock( &this->count_mutex );
  process_image = false;
  return true;
}

bool VisionPart::getMarkerCenter(beyoNAOnce::marker_pos::Request  &req,beyoNAOnce::marker_pos::Response &res){
  if (current_image.empty())
      return false;
  pthread_mutex_lock( &this->count_mutex );
  process_image = true;
  vector<Mat> extrinsics;
  extrinsics.clear();
  // MARKER DETECTION
  std::vector<Marker> Markers = extractMarkers(current_image);
  for (int i = 0; i < Markers.size(); i++){
    Markers[i].calculateExtrinsics(MARKER_SIZE, cameraParameters, false);
    extrinsics.push_back(Markers[i].Tvec);
    extrinsics.push_back(Markers[i].Rvec);
    beyoNAOnce::Marker msg;
    res.id.push_back(Markers[i].id);
    res.x.push_back(Markers[i].Tvec.at<float>(0));
    res.y.push_back(Markers[i].Tvec.at<float>(1));
    res.z.push_back(Markers[i].Tvec.at<float>(2));
    res.roll.push_back(Markers[i].Rvec.at<float>(0));
    res.pitch.push_back(Markers[i].Rvec.at<float>(1));
    res.yaw.push_back(Markers[i].Rvec.at<float>(2));
  }
  pthread_mutex_unlock( &this->count_mutex );
  process_image = false;
  if (Markers.size())
    return true;
  else
    return false;
}

bool VisionPart::detectBlob(Mat img){
  if (current_image.empty())
      return false;
  vector<cv::Point> temp, max_size;
  string max_arg;
  beyoNAOnce::BlobCenter msg;
  for(std::map<string,vector<cv::Scalar> >::iterator iter = color_map.begin(); iter != color_map.end(); ++iter){
    temp = (extractColor(current_image,iter->first));
    if(temp.size() > max_size.size()){
      max_size = temp;
      max_arg = iter->first;
    }
  }
  RotatedRect boundRect = minAreaRect(Mat(max_size));
  float area = boundRect.size.width*boundRect.size.height;
  if (area > current_image.rows*current_image.cols*0.2)
    msg.valid = true;
  else
    msg.valid = false;
  Point2f center = boundRect.center;
  msg.color = max_arg;
  msg.x = center.x;
  msg.y = center.y;
  blob_center_pub.publish(msg);
  process_image = false;
  return true;
}

// Detection
vector<cv::Point> VisionPart::extractColor(Mat img, string color){
    vector<cv::Point> result;
    if (img.empty())
        return result;
    //  HSV image
    Mat hsvImage;
    //  Extracted blue color image
    Mat blueImage;
    //  Images for the blob extraction
    //img = Clahe(img,4,12);

    //  Convert image to HSV
    cvtColor( img, hsvImage, CV_BGR2HSV);

    //  Extract color
    Mat color_img;
    Scalar color_min = this->color_map[color][0];
    Scalar color_max = this->color_map[color][1];
    inRange(hsvImage, color_min, color_max, color_img); // Extract the color

    //imshow("Erode", erosion);
    //  Detecting Contour
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    RotatedRect boundRect;
    /// Find contours
    cv::findContours(color_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    if (!(contours.size() > 0)){
        return result;
    }
    else{
        // Compare wrt. the Y coordinate
        if(sort_type == 1){
            std::sort(contours.begin(),contours.end(),compare_y);
        }
        // Compare wrt. the X coordinate
        else if(sort_type == 2){
            std::sort(contours.begin(),contours.end(),compare_x);
        }
        // Compare wrt. the area of the rectangle bounding the blob
        else if(sort_type == 3){
            std::sort(contours.begin(),contours.end(),compare_area);
        }
        // Compare wrt. the area of the size of the contour
        else {
            std::sort(contours.begin(),contours.end(),compare_size);
        }

        // Reverse the sort to have the biggest values first
        reverse(contours.begin(),contours.end());

        //Save biggest blobs
        boundRect= minAreaRect(Mat(contours[0]));
        Point2f center = boundRect.center;

        Mat show = img.clone();
        // Used for RGB image show
        Scalar shadow_color = Scalar( 0, 255, 0);
        // Only if blob is detected, draw
        drawContours(show, contours, 0, shadow_color, 2, 8, hierarchy, 0, Point() );
        string name = color;
        name += " color extracted";
        // Show result
        imshow(name, show);
        waitKey(3);
    }
    // Take the middle of the two central points
    return contours[0];
}

// Extract markers
vector<Marker> VisionPart::extractMarkers(Mat img){
  // MARKER DETECTION
  std::vector<Marker> Markers;
  Point2f left, right;
  Mat im_show = img.clone();

  // Detection Parameters
  MDetector.setWarpSize(100);
 //MDetector.enableLockedCornersMethod(true);
  MDetector.setMinMaxSize(0.01, 0.5);

  // Detect the markers
  MDetector.detect(im_show,Markers,cameraParameters,0.03);
  // For each marker, draw info and its boundaries in the image
  if (Markers.size()>0){
      for (unsigned int i=0;i<Markers.size();i++) {
        Markers[i].draw(im_show, cv::Scalar(0,0,255),2);
      }
      if (im_show.channels() == 3)
          imshow("Marker Detection", im_show);
  }
  return Markers;
}

vector<Mat> VisionPart::extractMarker(Mat img, int id){
    vector<Mat> extrinsics;
    extrinsics.clear();
    // MARKER DETECTION
    std::vector<Marker> Markers = extractMarkers(img);
    for (int i = 0; i < Markers.size(); i++){
        if (Markers[i].id == id){
            Markers[i].calculateExtrinsics(MARKER_SIZE, cameraParameters, false);
            extrinsics.push_back(Markers[i].Tvec);
            extrinsics.push_back(Markers[i].Rvec);
            beyoNAOnce::Marker msg;
            msg.id = Markers[i].id;
            msg.x = Markers[i].Tvec.at<float>(0);
            msg.y = Markers[i].Tvec.at<float>(1);
            msg.z = Markers[i].Tvec.at<float>(2);
            msg.roll =  Markers[i].Rvec.at<float>(0);
            msg.pitch = Markers[i].Rvec.at<float>(1);
            msg.yaw =   Markers[i].Rvec.at<float>(2);
            markers_pub.publish(msg);
        }
        else
            continue;
    }
    return extrinsics;
}

bool VisionPart::extractFirstMarker(Mat img){
    vector<Mat> extrinsics;
    extrinsics.clear();
    // MARKER DETECTION
    std::vector<Marker> Markers = extractMarkers(img);
    if (Markers.size() > 0){
      Markers[0].calculateExtrinsics(MARKER_SIZE, cameraParameters, false);
      extrinsics.push_back(Markers[0].Tvec);
      extrinsics.push_back(Markers[0].Rvec);
      beyoNAOnce::Marker msg;
      msg.id = Markers[0].id;
      msg.x = Markers[0].Tvec.at<float>(0);
      msg.y = Markers[0].Tvec.at<float>(1);
      msg.z = Markers[0].Tvec.at<float>(2);
      msg.roll =  Markers[0].Rvec.at<float>(0);
      msg.pitch = Markers[0].Rvec.at<float>(1);
      msg.yaw =   Markers[0].Rvec.at<float>(2);
      markers_pub.publish(msg);
      return true;
    }
    else
      return false;
}

// Callback functions
void VisionPart::imageCb(const sensor_msgs::ImageConstPtr& msg){
    // Reduce the frame rate
    ros::Duration(0.5).sleep();
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
    Mat InImage=cv_ptr->image;
    if(!process_image)
      current_image = InImage;
    imshow("RAW IMAGE",InImage);
    detectBlob(InImage);
    //ros::Duration(5).sleep();
    waitKey(1);
}

/**
* @brief Initialization of the node.
*
* @param    argc    number of the arguments
* @param    argv    arguments fed in the function*
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bey_vision");
    VisionPart vision;
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    sleep(1);
    while (ros::ok())
      spinner.spin();
    return 0;
}
