// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

// static const std::string OPENCV_WINDOW = "Image window";

// class ImageConverter
// {
//   ros::NodeHandle nh_;
//   image_transport::ImageTransport it_;
//   image_transport::Subscriber image_sub_;
//   image_transport::Publisher image_pub_;

// public:
//   ImageConverter()
//     : it_(nh_)
//   {
//     // Subscribe to input video feed and publish output video feed
//     image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1,
//       &ImageConverter::imageCb, this);
//     image_pub_ = it_.advertise("/image_converter/output_video", 1);

//     cv::namedWindow(OPENCV_WINDOW);
//   }

//   ~ImageConverter()
//   {
//     cv::destroyWindow(OPENCV_WINDOW);
//   }

//   void imageCb(const sensor_msgs::ImageConstPtr& msg)
//   {
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//     }

//     // // Draw an example circle on the video stream
//     // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//     //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));



// // Read image
// Mat im = imread( "blob.jpg", IMREAD_GRAYSCALE );
 
// // Set up the detector with default parameters.
// SimpleBlobDetector detector;
 
// // Detect blobs.
// std::vector<KeyPoint> keypoints;
// detector.detect( im, keypoints);
 
// // Draw detected blobs as red circles.
// // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
// Mat im_with_keypoints;
// drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
 
// // Show blobs
// imshow("keypoints", im_with_keypoints );
// waitKey(0);
//     // Update GUI Window
//     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//     cv::waitKey(3);

//     // Output modified video stream
//     image_pub_.publish(cv_ptr->toImageMsg());
//   }
// };

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "image_converter");
//   ImageConverter ic;
//   ros::spin();
//   return 0;
// }
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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

    // Rilevamento di blob basato su cerchi
    Mat gray_image;
    cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

    // Configurazione dei parametri per SimpleBlobDetector
    SimpleBlobDetector::Params params;
     params.minThreshold = 0;
     params.maxThreshold = 255;
    params.filterByCircularity = true;
    params.minCircularity = 0;  
    params.maxCircularity = 1;  
    // Creazione del rilevatore di blob con i parametri configurati
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // Rileva i blob
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray_image, keypoints);

    // Disegna cerchi rilevati sull'immagine
    drawKeypoints(cv_ptr->image, keypoints, cv_ptr->image, cv::Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);



    // Aggiorna finestra GUI
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);

    // Pubblica il video modificato
    image_pub_.publish(cv_ptr->toImageMsg());
    // Informazioni di debug
    ROS_INFO("Numero di cerchi rilevati: %lu", keypoints.size());
    for (const auto& keypoint : keypoints) {
        ROS_INFO("Cerchio rilevato - Posizione: (%f, %f), Raggio: %f",
                 keypoint.pt.x, keypoint.pt.y, keypoint.size);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
