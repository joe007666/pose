#include "MvCameraControl.h"
// ros
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>

int main(int argc, char** argv)
{
    // thread
    std::thread capture_thread_;
    cv::VideoCapture cap(2bdf:0001);
    // ros
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    sensor_msgs::ImagePtr image_msg_;
    ros::Rate loop_rate(100);
    cv::Mat frame;
    // sdk
    int nRet = MV_OK;
    void *camera_handle_;
    MV_IMAGE_BASIC_INFO img_info_;
    MV_CC_PIXEL_CONVERT_PARAM ConvertParam_;
    MV_CC_DEVICE_INFO_LIST DeviceList;
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &DeviceList);
    while (DeviceList.nDeviceNum == 0 && nh.ok()) {
      ROS_ERROR("Found camera count = %d", DeviceList.nDeviceNum);
      cap >> frame;
      cv::imshow("test",frame);
      ROS_INFO("Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &DeviceList);
    }
    ROS_INFO("camera has been found!");
    // get basic infomation
    MV_CC_CreateHandle(&camera_handle_, DeviceList.pDeviceInfo[0]);
    MV_CC_OpenDevice(camera_handle_);
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    MV_CC_OpenDevice(camera_handle_);
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_->data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);
    // set convert param
    ConvertParam_.nWidth = img_info_.nWidthMax;
    ConvertParam_.nHeight = img_info_.nHeightMax;
    ConvertParam_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    MV_CC_StartGrabbing(camera_handle_);


    MV_FRAME_OUT OutFrame;
    ROS_INFO("Publishing image!");
      image_msg_->header.frame_id = "rgb_frame";
      image_msg_->encoding = "rgb8";


    // start to pub
    while (nh.ok()) {
    nRet = MV_CC_GetImageBuffer(camera_handle_, &OutFrame, 1000);
    
        if (MV_OK == nRet) {
          ConvertParam_.pDstBuffer = image_msg_->data.data();
          ConvertParam_.nDstBufferSize = image_msg_->data.size();
          ConvertParam_.pSrcData = OutFrame.pBufAddr;
          ConvertParam_.nSrcDataLen = OutFrame.stFrameInfo.nFrameLen;
          ConvertParam_.enSrcPixelType = OutFrame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &ConvertParam_);
          image_msg_->height = OutFrame.stFrameInfo.nHeight;
          image_msg_->width = OutFrame.stFrameInfo.nWidth;
          image_msg_->step = OutFrame.stFrameInfo.nWidth * 3;
          image_msg_->data.resize(image_msg_->width * image_msg_->height * 3);
          pub.publish(image_msg_);
          MV_CC_FreeImageBuffer(camera_handle_, &OutFrame);

        }
        else {
          ROS_INFO("Get buffer failed! nRet: [%x]",nRet);
          MV_CC_StopGrabbing(camera_handle_);
          MV_CC_StartGrabbing(camera_handle_);
        }
    ros::spinOnce();
    loop_rate.sleep();
  }


    return 0;
}