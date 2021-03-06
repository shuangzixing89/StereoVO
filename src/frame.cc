#include <memory>

//
// Created by lixin on 18-12-19.
//

#include "frame.hpp"

namespace StereoVO
{
    Frame::Frame()
            : id_(static_cast<unsigned long>(-1)), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
    {

    }

    Frame::Frame ( long id, double time_stamp, Mat T_c_w, Camera::Ptr camera, Mat left, Mat right )
            : id_(static_cast<unsigned long>(id)), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), left_(left), right_(right), is_key_frame_(false)
    {

    }

    Frame::~Frame()
    = default;

    Frame::Ptr Frame::createFrame()
    {
        static long factory_id = 0;
        return std::make_shared<Frame>(factory_id++);
    }

    double Frame::findDepth ( const cv::KeyPoint& kp )
    {

        return -1.0;
    }

    void Frame::setPose ( const Mat& T_c_w )
    {
        T_c_w_ = T_c_w;
    }


    Point3d Frame::getCamCenter() const
    {
        Mat R_c_w = T_c_w_.colRange(0,3).rowRange(0,3),
        t_c_w = T_c_w_.colRange(3,4).rowRange(0,3);
        Mat ret = -R_c_w.inv()*t_c_w;
        return Point3d( ret.at<double>(0),  ret.at<double>(1), ret.at<double>(2));
    }

    bool Frame::isInFrame ( const Point3d& pt_world )
    {
        Point3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
        // cout<<"P_cam = "<<p_cam.transpose()<<endl;
        if ( p_cam.z<0 ) return false;
        Point2d pixel = camera_->world2pixel( pt_world, T_c_w_ );

//         cout<<"P_pixel = "<<pixel <<endl<<endl;
        return pixel.x>0 && pixel.y>0
               && pixel.x<left_.cols
               && pixel.y<left_.rows;
    }

}
