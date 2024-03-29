#include "common/common_utils/StrictMode.hpp"
#include "ros/ros.h"
#include <ros/console.h>
#include <ros/spinner.h>
#include <sensor_msgs/Image.h>
#include "common/AirSimSettings.hpp"
#include "common/Common.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "statistics.h"
#include "rpc/rpc_error.h"
#include <cv_bridge/cv_bridge.h>
#include <math.h>



typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

// number of seconds to record frames before printing fps
const int FPS_WINDOW = 3;

msr::airlib::CarRpcLibClient* airsim_api;
ros::Publisher image_pub, stereo_left_image_pub, stereo_right_image_pub;
ros_bridge::Statistics fps_statistic;

// settings
std::string camera_name = "";
double framerate = 0.0;
std::string host_ip = "localhost";
bool depthcamera = false;
bool stereocamera = false;

ros::Time make_ts(uint64_t unreal_ts)
{
   // unreal timestamp is a unix nanosecond timestamp
   // Ros also uses unix timestamps, as long as it is not running in simulated time mode.
   // For now we are not supporting simulated time.
   ros::Time out;
   return out.fromNSec(unreal_ts);
}

std::string logprefix() {
    return "[cambridge " + camera_name + "] ";
}

std::vector<ImageResponse> getImage(std::vector<ImageRequest> reqs) {
     // We are using simGetImages instead of simGetImage because the latter does not return image dimention information.

    std::vector<ImageResponse> img_responses;
    try {
        img_responses = airsim_api->simGetImages(reqs, "FSCar");
    } catch (rpc::rpc_error& e) {
        std::cout << "error" << std::endl;
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API while getting image:" << std::endl
                << msg << std::endl;
    }
    return img_responses;
}

void doImageUpdate(const ros::TimerEvent&)
{
    std::vector<ImageRequest> reqs;
    reqs.push_back(ImageRequest(camera_name, ImageType::Scene, false, false));

    auto img_responses = getImage(reqs);

    // if a render request failed for whatever reason, this img will be empty.
    if (img_responses.size() == 0 || img_responses[0].time_stamp == 0)
        return;

    ImageResponse img_response = img_responses[0];

    sensor_msgs::ImagePtr img_msg = boost::make_shared<sensor_msgs::Image>();

    img_msg->data = img_response.image_data_uint8;
    img_msg->height = img_response.height;
    img_msg->width = img_response.width;
    img_msg->step = img_response.width * 3; // image_width * num_bytes
    img_msg->encoding = "bgr8";
    img_msg->is_bigendian = 0;
    img_msg->header.stamp = make_ts(img_response.time_stamp);
    img_msg->header.frame_id = "/fsds/camera/"+camera_name;
    
    image_pub.publish(img_msg);
    fps_statistic.addCount();
}

void doStereoImageUpdate(const ros::TimerEvent&)
{
    // cam1 - right camera
    // cam2 - left camera
    std::vector<ImageRequest> reqs = {
        ImageRequest("cam_left", ImageType::Scene, false, false), 
        ImageRequest("cam_right", ImageType::Scene, false, false)
    };

    auto img_responses = getImage(reqs);

    if (img_responses.size() == 0 || img_responses[0].time_stamp == 0 || img_responses[1].time_stamp == 0)
        return;

    ImageResponse right_img_response = img_responses[1];
    ImageResponse left_img_response = img_responses[0];

    sensor_msgs::ImagePtr right_img_msg = boost::make_shared<sensor_msgs::Image>();

    right_img_msg->data = right_img_response.image_data_uint8;
    right_img_msg->height = right_img_response.height;
    right_img_msg->width = right_img_response.width;
    right_img_msg->step = right_img_response.width * 3; // image_width * num_bytes
    right_img_msg->encoding = "bgr8";
    right_img_msg->is_bigendian = 0;
    right_img_msg->header.stamp = make_ts(right_img_response.time_stamp);
    right_img_msg->header.frame_id = "/fsds/camera/cam_right";

    sensor_msgs::ImagePtr left_img_msg = boost::make_shared<sensor_msgs::Image>();

    left_img_msg->data = left_img_response.image_data_uint8;
    left_img_msg->height = left_img_response.height;
    left_img_msg->width = left_img_response.width;
    left_img_msg->step = left_img_response.width * 3; // image_width * num_bytes
    left_img_msg->encoding = "bgr8";
    left_img_msg->is_bigendian = 0;
    left_img_msg->header.stamp = make_ts(left_img_response.time_stamp);
    left_img_msg->header.frame_id = "/fsds/camera/cam_left";

    stereo_left_image_pub.publish(left_img_msg);
    stereo_right_image_pub.publish(right_img_msg);

    fps_statistic.addCount();
}

cv::Mat manual_decode_depth(const ImageResponse& img_response)
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
    return mat;
}

float roundUp(float numToRound, float multiple) 
{
    assert(multiple);
    return ((numToRound + multiple - 1) / multiple) * multiple;
}

cv::Mat noisify_depthimage(cv::Mat in)
{
    cv::Mat out = in.clone();

    // Blur
    cv::Mat kernel = cv::Mat::ones( 7, 7, CV_32F ) / (float)(7 * 7);
    cv::filter2D(in, out, -1 , kernel, cv::Point( -1, -1 ), 0, cv::BORDER_DEFAULT );

    // Decrease depth resolution
    for (int row = 0; row < in.rows; row++) {
        for (int col = 0; col < in.cols; col++) {
            float roundtarget = ceil(std::min(std::max(out.at<float>(row, col), 1.0f), 10.0f));
            out.at<float>(row, col) = roundUp(out.at<float>(row, col), roundtarget);
        }
    }

    return out;
}

void doDepthImageUpdate(const ros::TimerEvent&) {

    std::vector<ImageRequest> reqs;
    reqs.push_back(ImageRequest(camera_name, ImageType::DepthPerspective, true, false));

    auto img_responses = getImage(reqs);

    // if a render request failed for whatever reason, this img will be empty.
    if (img_responses.size() == 0 || img_responses[0].time_stamp == 0)
        return;

    ImageResponse img_response = img_responses[0];

    cv::Mat depth_img = noisify_depthimage(manual_decode_depth(img_response));
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    img_msg->header.stamp = make_ts(img_response.time_stamp);
    img_msg->header.frame_id = "/fsds/camera/"+camera_name;    
    
    image_pub.publish(img_msg);
    fps_statistic.addCount();
}

void printFps(const ros::TimerEvent&)
{
    std::cout << logprefix() << "Average FPS: " << (fps_statistic.getCount()/FPS_WINDOW) << std::endl;
    fps_statistic.Reset();
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "fsds_ros_bridge_camera");
    ros::NodeHandle nh("~");

    // load settings
    nh.param<std::string>("camera_name", camera_name, "");
    nh.param<double>("framerate", framerate, 0.0);
    nh.param<std::string>("host_ip", host_ip, "localhost");
    nh.param<bool>("depthcamera", depthcamera, false);
    nh.param<bool>("stereocamera", stereocamera, false);    

    if(camera_name == "") {
        std::cout << logprefix() << "camera_name unset." << std::endl;
        return 1;
    }
    if(framerate == 0) {
        std::cout << logprefix() << "framerate unset." << std::endl;
        return 1;
    }

    // initialize fps counter
    fps_statistic = ros_bridge::Statistics("fps");

    // ready airsim connection
    msr::airlib::CarRpcLibClient client(host_ip, RpcLibPort, 5);
    airsim_api = &client;

    try {
        airsim_api->confirmConnection();
    } catch (const std::exception &e) {
        std::string msg = e.what();
        std::cout << logprefix() << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
        return 1;
    }

    // ready topic
    if (stereocamera)
    {
        stereo_left_image_pub = nh.advertise<sensor_msgs::Image>("/fsds/camera/cam_left", 1);
        stereo_right_image_pub = nh.advertise<sensor_msgs::Image>("/fsds/camera/cam_right", 1);
    }
    else
    {
        image_pub = nh.advertise<sensor_msgs::Image>("/fsds/camera/" + camera_name, 1);
    }

    // start the loop
    // ros::Timer imageTimer = nh.createTimer(ros::Duration(1/framerate), depthcamera ? doDepthImageUpdate : doImageUpdate);
    ros::Timer imageTimer = nh.createTimer(ros::Duration(1/framerate), depthcamera ? doDepthImageUpdate : stereocamera ? doStereoImageUpdate : doImageUpdate);
    ros::Timer fpsTimer = nh.createTimer(ros::Duration(FPS_WINDOW), printFps);
    ros::spin();
    return 0;
} 

