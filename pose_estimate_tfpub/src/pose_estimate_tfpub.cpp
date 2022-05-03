#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/core/types.hpp>
#include <ostream>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#define width 30;
#define length 10;
using namespace cv;

class pose_estimate{

public:

    pose_estimate(){
        this->q.setRPY(0, 0, 0);
        template_1=imread("/home/joe/catkin_ws/src/33333.png");

        detector->detectAndCompute( template_1, noArray(), keypoints_object, descriptors_object );


    };   
    // camrea matrix
    cv::Mat _distortion_coeff=( Mat_<double>(5,1) << -6.9242772099144576e-02, 1.4831942311252244e-01, 0., 0., 0. );
    cv::Mat cameraMatrix= ( Mat_<double>(3,3) << 1.3082572456106684e+03, 0., 6.5259124163819479e+02, 0.,
      1.3073473013511968e+03, 4.9349608147204179e+02, 0., 0., 1.);
    // detect variable
    Mat descriptors_object, descriptors_scene ,img_matches, template_1 , template_2, img_scene;
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    int minHessian = 400;
    Ptr<BRISK> detector = cv::BRISK::create( minHessian );
    const float ratio_thresh = 0.75f;
    //ros tf 
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q;
    std::vector<cv::Point3f> point3d;
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);




    void imageCallback(const sensor_msgs::ImageConstPtr& msg)

    {   
        ROS_INFO("image process");
        std::vector< std::vector<DMatch> > knn_matches;
        std::vector<DMatch> good_matches;
        std::vector<Point2f> obj, scene, scene_corners(4);
        std::vector<cv::Mat> rvecs, tvecs;
        double reprojectionError;
        int solution = 0;
        cv::Mat_<double> reprojectionErrorMatrix;
        cv::Vec3d rvec;
        cv::Vec3d tvec;

        try
        {
        Mat cam_img=cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        catch (cv::Exception x)
         {
        return;
        }
        detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );
        matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );
        // 筛选出好的匹配点
         for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    drawMatches( template_1, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    // 在目标还没出现时防止暴毙
    try {
        Mat H = findHomography( obj, scene, RANSAC );

    } 
    catch (cv::Exception x) {
        return;
    }

    Mat H = findHomography( obj, scene, RANSAC );

    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = Point2f(0, 0);
    obj_corners[1] = Point2f( (float)template_1.cols, 0 );
    obj_corners[2] = Point2f( (float)template_1.cols, (float)template_1.rows );
    obj_corners[3] = Point2f( 0, (float)template_1.rows );
    // 防止暴毙
    if (! H.empty()) {
    perspectiveTransform( obj_corners, scene_corners, H);
    
    }
    else {
    return;
    
    }
    try {
           solution=solvePnPGeneric(point3d, 
                    scene_corners, 
                    cameraMatrix, 
                    _distortion_coeff, 
                     rvecs, 
                     tvecs,
                     reprojectionError);

    } 
    catch (cv::Exception x) {
        return;
    }
    // pnp成功则转换为vector
    if (solution > 0) {
        rvecs[0].convertTo(rvec, CV_64F);
        tvecs[0].convertTo(tvec, CV_64F);

    }
    // 发送tf
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera";
    transformStamped.child_frame_id = "gold";
    transformStamped.transform.translation.x = tvec[0];
    transformStamped.transform.translation.y = tvec[1];
    transformStamped.transform.translation.z = tvec[2];
    q.setRPY(rvec[0], rvec[1], rvec[2]);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    static tf2_ros::TransformBroadcaster br;

    br.sendTransform(transformStamped);
    float x=tvec[0];
    float y=tvec[1];
    float z=tvec[2];
    float distance=sqrt(x*x+y*y+z*z);



    }



};
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    pose_estimate p_e;
    image_transport::ImageTransport it(nh);
    ros::Subscriber sub = nh.subscribe("chatter", 1000,&pose_estimate::imageCallback, &p_e);
    ros::spin();


    return 0;
}