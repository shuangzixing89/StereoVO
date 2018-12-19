//
// Created by lixin on 18-12-19.
//

#ifndef STEREOVO_FRAME_HPP
#define STEREOVO_FRAME_HPP

#include "common.hpp"
#include "camera.hpp"

namespace StereoVO
{
    // forward declare
    class MapPoint;
    class Frame
    {
    public:
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long                  id_;         // id of this frame
        double                         time_stamp_; // when it is recorded
        Mat                            T_c_w_;      // transform from world to camera
        Camera::Ptr                    camera_;     // Pinhole RGBD Camera model
        Mat                            left_, right_; // color and depth image
        // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
        // std::vector<MapPoint*>         map_points_; // associated map points
        bool                           is_key_frame_;  // whether a key-frame

    public: // data members
        Frame();
        Frame( long id, double time_stamp=0, Mat T_c_w=Mat(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
        ~Frame();

        static Frame::Ptr createFrame();

        // find the depth in depth map
        double findDepth( const cv::KeyPoint& kp );

        // Get Camera Center
        Point3d getCamCenter() const;

        void setPose( const Mat& T_c_w );

        // check if a point is in this frame
        bool isInFrame( const Point3d& pt_world );
    };
}

#endif //STEREOVO_FRAME_HPP
