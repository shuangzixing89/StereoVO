//
// Created by lixin on 18-12-19.
//

#include "config.hpp"
#include "track.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
//#include <boost/timer.hpp>


namespace StereoVO
{
    Track::Track() :
            state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
    {
        num_of_features_    = Config::get<int> ( "number_of_features" );
        scale_factor_       = Config::get<double> ( "scale_factor" );
        level_pyramid_      = Config::get<int> ( "level_pyramid" );
        match_ratio_        = Config::get<float> ( "match_ratio" );
        max_num_lost_       = Config::get<float> ( "max_num_lost" );
        min_inliers_        = Config::get<int> ( "min_inliers" );
        key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
        key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
        map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
        orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
    }

    Track::~Track()
    {

    }

    bool Track::addFrame ( Frame::Ptr frame )
    {
        switch ( state_ )
        {
            case INITIALIZING:
            {
                state_ = OK;
                curr_ = ref_ = frame;
                // extract features from first frame and add them into map
                extractKeyPoints();
                computeDescriptors();
                addKeyFrame();      // the first frame is a key-frame
                break;
            }
            case OK:
            {
                curr_ = frame;
                curr_->T_c_w_ = ref_->T_c_w_;
                extractKeyPoints();
                computeDescriptors();
                featureMatching();
                poseEstimationPnP();
                if ( checkEstimatedPose() == true ) // a good estimation
                {
                    curr_->T_c_w_ = T_c_w_estimated_;
                    optimizeMap();
                    num_lost_ = 0;
                    if ( checkKeyFrame() == true ) // is a key-frame
                    {
                        addKeyFrame();
                    }
                }
                else // bad estimation due to various reasons
                {
                    num_lost_++;
                    if ( num_lost_ > max_num_lost_ )
                    {
                        state_ = LOST;
                    }
                    return false;
                }
                break;
            }
            case LOST:
            {
                cout<<"vo has lost."<<endl;
                break;
            }
        }

        return true;
    }

    void Track::extractKeyPoints()
    {
        orb_->detect ( curr_->left_, keypoints_curr_left_ );
        orb_->detect ( curr_->right_, keypoints_curr_right_ );
    }
    void Track::computeDescriptors()
    {
        orb_->compute ( curr_->left_, keypoints_curr_left_, descriptors_curr_left_ );
        orb_->compute ( curr_->right_, keypoints_curr_right_, descriptors_curr_right_ );
    }

    void Track::featureMatching()
    {
        vector<cv::DMatch> matches;
        // select the candidates in map
        Mat desp_map;
        vector<MapPoint::Ptr> candidate;
        for ( auto& allpoints: map_->map_points_ )
        {
            MapPoint::Ptr& p = allpoints.second;
            // check if p in curr frame image
            if ( curr_->isInFrame(p->pos_) )
            {
                // add to candidate
                p->visible_times_++;
                candidate.push_back( p );
                desp_map.push_back( p->descriptor_ );
            }
        }

        matcher_flann_.match ( desp_map, descriptors_curr_, matches );
        // select the best matches
        float min_dis = std::min_element (
                matches.begin(), matches.end(),
                [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
                {
                    return m1.distance < m2.distance;
                } )->distance;

        match_3dpts_.clear();
        match_2dkp_index_.clear();
        for ( cv::DMatch& m : matches )
        {
            if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
            {
                match_3dpts_.push_back( candidate[m.queryIdx] );
                match_2dkp_index_.push_back( m.trainIdx );
            }
        }
        cout<<"good matches: "<<match_3dpts_.size() <<endl;
    }


    void Track::addKeyFrame()
    {
        if ( map_->keyframes_.empty() )
        {
            // first key-frame, add all 3d points into map
            for ( size_t i=0; i<keypoints_curr_left_.size(); i++ )
            {
                // TODO
                double d = curr_->findDepth ( keypoints_curr_left_[i] , keypoints_curr_right_[  ]);
                if ( d < 0 )
                    continue;
                Point3d p_world = ref_->camera_->pixel2world (
                        Point2d( keypoints_curr_left_[i].pt.x , keypoints_curr_left_[i].pt.y ), curr_->T_c_w_, d
                );
                Point3d n = p_world - ref_->getCamCenter();
                double n_all = std::sqrt( n.x * n.x  + n.y * n.y + n.z * n.z );
                n.x /= n_all;
                n.y /= n_all;
                n.z /= n_all;
                MapPoint::Ptr map_point = MapPoint::createMapPoint(
                        p_world, n, descriptors_curr_left_.row(i).clone(), curr_.get()
                );
                map_->insertMapPoint( map_point );
            }
        }

        map_->insertKeyFrame ( curr_ );
        ref_ = curr_;
    }
}