//
// Created by lixin on 18-12-19.
//

#ifndef STEREOVO_TRACK_HPP
#define STEREOVO_TRACK_HPP

#include "common.hpp"
#include "map.hpp"
#include "opencv2/features2d.hpp"
//#include <opencv2/xfeatures2d.hpp>

namespace StereoVO
{
    class Track
    {
    public:
        typedef shared_ptr<Track> Ptr;
        enum VOState {
            INITIALIZING=-1,
            OK=0,
            LOST
        };

        VOState     state_;     // current VO status
        Map::Ptr    map_;       // map with all frames and map points

        Frame::Ptr  ref_;       // reference key-frame
        Frame::Ptr  curr_;      // current frame

//        cv::Ptr<cv::xfeatures2d::SURF> surf_;  // orb detector and computer
//        cv::Ptr<cv::ORB> orb_;  // orb detector and computer
        cv::Ptr<cv::FeatureDetector> detector_;
        cv::Ptr<cv::DescriptorExtractor> descriptor_;
        cv::Ptr<cv::DescriptorMatcher> matcher_  ;
        vector<cv::KeyPoint>    keypoints_curr_left_, keypoints_curr_right_;    // keypoints in current frame
        Mat                     descriptors_curr_left_, descriptors_curr_right_;  // descriptor in current frame
        vector <vector<cv::DMatch>> matches_curr_;
        vector < cv::DMatch > matches_curr_good_;

        cv::FlannBasedMatcher   matcher_flann_;     // flann matcher
        vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points
        vector<int>             match_2dkp_index_;  // matched 2d pixels (index of kp_curr)

        Mat T_c_w_estimated_;    // the estimated pose of current frame
        int num_inliers_;        // number of inlier features in icp
        int num_lost_;           // number of lost times

        // parameters
        int num_of_features_;   // number of features
        double scale_factor_;   // scale in image pyramid
        int level_pyramid_;     // number of pyramid levels
        float match_ratio_;     // ratio for selecting  good matches
        int max_num_lost_;      // max number of continuous lost times
        int min_inliers_;       // minimum inliers
        double key_frame_min_rot;   // minimal rotation of two key-frames
        double key_frame_min_trans; // minimal translation of two key-frames
        double  map_point_erase_ratio_; // remove map point ratio


        // Corresponding stereo coordinate and depth for each keypoint.
        // "Monocular" keypoints have a negative value.
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;

    public: // functions
        Track();
        ~Track();

        bool addFrame( Frame::Ptr frame );      // add a new frame

    protected:
        // inner operation
        void extractKeyPoints();
        void computeDescriptors();
        void matchCurr();
        void featureMatching();
        bool poseEstimationPnP();
        void optimizeMap();
        void ComputeStereoMatches();
        int  DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

        double findDepth ();
        double my_nom(Mat);

        void addKeyFrame();
        void addMapPoints();
        bool checkEstimatedPose();
        bool checkKeyFrame();

        double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );

    };
}

#endif //STEREOVO_TRACK_HPP
