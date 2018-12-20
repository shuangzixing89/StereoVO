//
// Created by lixin on 18-12-19.
//

#include "frame.hpp"

namespace StereoVO
{
    Frame::Frame()
            : id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
    {

    }

    Frame::Frame ( long id, double time_stamp, Mat T_c_w, Camera::Ptr camera, Mat left, Mat right )
            : id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), left_(left), right_(right), is_key_frame_(false)
    {

    }

    Frame::~Frame()
    {

    }

    Frame::Ptr Frame::createFrame()
    {
        static long factory_id = 0;
        return Frame::Ptr( new Frame(factory_id++) );
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
        //TODO Point3d(```)
        return Point3d( -T_c_w_.at<double>(0,3),  -T_c_w_.at<double>(1,3), -T_c_w_.at<double>(2,3));
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
