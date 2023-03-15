#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class ObjectDetector
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

  public:
    ObjectDetector()
      : it_(nh_)
    {
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/camera/image_raw", 1,
        &ObjectDetector::imageCallback, this);
      image_pub_ = it_.advertise("/object_detection/output_video", 1);
    }

    ~ObjectDetector()
    {
      // Empty destructor
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {
        // Convert ROS image message to OpenCV image
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Define the lower and upper HSV thresholds for the color of the object to detect
        cv::Scalar lower_color(20, 100, 100);  // H: 20-40, S: 100-255, V: 100-255
        cv::Scalar upper_color(40, 255, 255);  // H: 20-40, S: 100-255, V: 100-255

        // Convert the input image from BGR to HSV color space
        cv::Mat img_hsv;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        // Apply the color threshold to the HSV image to extract the object of interest
        cv::Mat img_mask;
        cv::inRange(img_hsv, lower_color, upper_color, img_mask);

        // Find the contours of the object in the thresholded image
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(img_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Draw bounding boxes around the detected objects
        cv::Mat img_out = img.clone();
        for (size_t i = 0; i < contours.size(); i++)
        {
          cv::Rect bbox = cv::boundingRect(contours[i]);
          cv::rectangle(img_out, bbox, cv::Scalar(0, 255, 0), 2);
        }

        // Publish the output image
        sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_out).toImageMsg();
        image_pub_.publish(msg_out);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detector");
  ObjectDetector od;
  ros::spin();
  return 0;
}