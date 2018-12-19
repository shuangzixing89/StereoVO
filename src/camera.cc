//
// Created by lixin on 18-12-19.
//

#include "camera.hpp"

namespace StereoVO
{
    Camera::Camera()
    {
        fx_ = Config::get<float>("camera.fx");
        fy_ = Config::get<float>("camera.fy");
        cx_ = Config::get<float>("camera.cx");
        cy_ = Config::get<float>("camera.cy");
        depth_scale_ = Config::get<float>("camera.depth_scale");
    }

    Point3d Camera::world2camera( const Point3d& pt_world, Mat T_c_w_ )
    {
        Mat R = T_c_w_.colRange(0,3).colRange(0,3),
                t = T_c_w_.colRange(3,4).colRange(0,3),
                ptworld = (cv::Mat_<double>(3,1) << pt_world.x, pt_world.y, pt_world.z);
        Mat cam  = R * ptworld + t;
        return Point3d(cam.at<double>(0), cam.at<double>(1), cam.at<double>(2));
    }
    Point2d Camera::world2pixel( const Point3d& pt_world, Mat T_c_w_ )
    {
        return camera2pixel(world2camera(pt_world, T_c_w_));
    }
    Point2d Camera::camera2pixel( const Point3d& p_c )
    {
        return Point2d( fx_ * p_c.x / p_c.z + cx_, fy_ * p_c.y / p_c.z + cy_);
    }
}