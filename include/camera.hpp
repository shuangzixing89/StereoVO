//
// Created by lixin on 18-12-19.
//

#ifndef STEREOVO_CAMERA_HPP
#define STEREOVO_CAMERA_HPP

#include "common.hpp"
#include "config.hpp"

namespace StereoVO
{
    // Pinhole Stereo camera model
    class Camera
    {
    public:
        typedef std::shared_ptr<Camera> Ptr;
        float   fx_, fy_, cx_, cy_, depth_scale_, ThDepth_, b_, bf_;

        Camera();
        Camera ( float fx, float fy, float cx, float cy, float depth_scale=0 ) :
                fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale )
        {}

        //// coordinate transform: world, camera, pixel
        Point3d world2camera( const Point3d& pt_world, Mat T_c_w_ );
        Point2d world2pixel( const Point3d& pt_world, Mat T_c_w_ );
        Point3d camera2world( const Point3d& p_c, const Mat& T_c_w );
        Point2d camera2pixel( const Point3d& p_c );
        Point3d pixel2camera( const Point2d& p_p, double depth=1 );
        Point3d pixel2world ( const Point2d& p_p, const Mat& T_c_w, double depth=1 );

    };
}


#endif //STEREOVO_CAMERA_HPP
