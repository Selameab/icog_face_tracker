#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow("ASD", cv_ptr->image);
    cv::waitKey(3);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/usb_cam_node/image_raw", 1, imageCb);
    ros::spin();
}

