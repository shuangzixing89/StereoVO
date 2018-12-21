//
// Created by lixin on 18-12-19.
//

#include "config.hpp"
#include "track.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
//#include <boost/timer.hpp>


namespace StereoVO
{
    struct MinReprojectionError
    {
        MinReprojectionError( Point3d p_w, Point2d p_p, Mat K_cam ): point_3d(p_w), point_2d(p_p), K(K_cam){}

        template <typename  T>
        bool operator()(
                const T* pose,
                T* residuals
                ) const{
            T p_world[3];
            p_world[0] = T(point_3d.x);
            p_world[1] = T(point_3d.y);
            p_world[2] = T(point_3d.z);

            T p_cam[3];
            ceres::AngleAxisRotatePoint(pose, p_world, p_cam);
            p_cam[0] += pose[3];
            p_cam[1] += pose[4];
            p_cam[2] += pose[5];

            T predicted_x = T( K.at<double>(0,0) ) * p_cam[0]/p_cam[2] + T(K.at<double>(0,2));
            T predicted_y = T( K.at<double>(1,1) ) * p_cam[1]/p_cam[2] + T(K.at<double>(1,2));

            T observed_x = T(point_2d.x);
            T observed_y = T(point_2d.y);

            residuals[0] = predicted_x - observed_x;
            residuals[1] = predicted_y - observed_y;

            return true;
        }

        static ceres::CostFunction* Create(const Point3d point_world,
                const Point2d Point_pixel, const Mat K_cam){
            return (new ceres::AutoDiffCostFunction< MinReprojectionError, 2, 6 >(new MinReprojectionError(point_world, Point_pixel, K_cam)));
        }

        Point3d point_3d;
        Point2d point_2d;
        Mat K;
    };


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
        detector_ = cv::ORB::create(num_of_features_,scale_factor_, level_pyramid_);
        descriptor_ = cv::ORB::create();
        matcher_ = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
        Mat R = Mat::eye(3,3,CV_64F),t = Mat::zeros(3,1,CV_64F);
        cv::hconcat(R,t,T_c_w_estimated_);
    }

    Track::~Track()
    {

    }

    bool Track::addFrame ( Frame::Ptr frame )
    {
        matches_curr_.clear();
        matches_curr_good_.clear();
        switch ( state_ )
        {
            case INITIALIZING:
            {
                state_ = OK;
                // TODO find curr_->frame->T_c_w_ / ref_->frame->T_c_w_ / T_c_w_estimated_ 地址相同
                curr_ = ref_ = frame;
                // extract features from first frame and add them into map
                extractKeyPoints();
                computeDescriptors();
                matchCurr();
//                ComputeStereoMatches();
                addKeyFrame();      // the first frame is a key-frame
                break;
            }
            case OK:
            {
                curr_ = frame;
                curr_->T_c_w_ = ref_->T_c_w_.clone() ;
                extractKeyPoints();
                computeDescriptors();
                matchCurr();
//                ComputeStereoMatches();
                featureMatching();
                if( poseEstimationPnP() == true)
                {
                    if ( checkEstimatedPose() == true ) // a good estimation
                    {
                        curr_->T_c_w_ = T_c_w_estimated_.clone();
                        optimizeMap();
                        num_lost_ = 0;
                        if ( checkKeyFrame() == true ) // is a key-frame
                        {
                            addKeyFrame();
                        }
                        else
                        {
                            std::cout << "checkKeyFrame false\n";
                        }
                    }
                    else // bad estimation due to various reasons
                    {
                        std::cout << "checkEstimatedPose false\n";
                        num_lost_++;
                        if ( num_lost_ > max_num_lost_ )
                        {
                            state_ = LOST;
                        }
                        return false;
                    }
                }
                else
                {
                    std::cout << "poseEstimationPnP false\n";
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
        keypoints_curr_right_.clear();
        keypoints_curr_left_.clear();
        detector_->detect ( curr_->left_, keypoints_curr_left_ );
        detector_->detect ( curr_->right_, keypoints_curr_right_ );
        std::cout << "ORB num :" << keypoints_curr_left_.size() + keypoints_curr_right_.size() << std::endl;
    }

    void Track::computeDescriptors()
    {
        descriptor_->compute ( curr_->left_, keypoints_curr_left_, descriptors_curr_left_ );
        descriptor_->compute ( curr_->right_, keypoints_curr_right_, descriptors_curr_right_ );
    }

    // TODO not very good
    void Track::ComputeStereoMatches()
    {
        int N = keypoints_curr_left_.size();
        mvuRight = vector<float>(N,-1.0f);
        mvDepth = vector<float>(N,-1.0f);

        const int thOrbDist = 75;

        const int nRows = curr_->left_.rows;

        //Assign keypoints to row table
        vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

        for(int i=0; i<nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = keypoints_curr_right_.size();

        for(int iR=0; iR<Nr; iR++)
        {
            const cv::KeyPoint &kp = keypoints_curr_right_[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f*keypoints_curr_right_[iR].octave;
            const int maxr = ceil(kpY+r);
            const int minr = floor(kpY-r);

            for(int yi=minr;yi<=maxr;yi++)
                vRowIndices[yi].push_back(iR);
        }

        // Set limits for search
        const float minZ = curr_->camera_->b_;
        const float minD = 0;
        const float maxD = curr_->camera_->bf_ / minZ;

        // For each left keypoint search a match in the right image
        vector<pair<int, int> > vDistIdx;
        vDistIdx.reserve(N);

        for(int iL=0; iL<N; iL++)
        {
            const cv::KeyPoint &kpL = keypoints_curr_left_[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if(vCandidates.empty())
                continue;

            const float minU = uL-maxD;
            const float maxU = uL-minD;

            if(maxU<0)
                continue;

            int bestDist = 100;
            size_t bestIdxR = 0;

            const cv::Mat &dL = descriptors_curr_left_.row(iL);

            // Compare descriptor to right keypoints
            for(size_t iC=0; iC<vCandidates.size(); iC++)
            {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = keypoints_curr_right_[iR];

                if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                    continue;

                const float &uR = kpR.pt.x;

                if(uR>=minU && uR<=maxU)
                {
                    const cv::Mat &dR = descriptors_curr_right_.row(iR);
                    const int dist = DescriptorDistance(dL,dR);

                    if(dist<bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }



            float disparity = (uL-bestIdxR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestIdxR = uL-0.01;
                }
                mvDepth[iL]=curr_->camera_->bf_/disparity;
                mvuRight[iL] = bestIdxR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }

        sort(vDistIdx.begin(),vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size()/2].first;
        const float thDist = median;

//        int num = 0;
        for(int i=vDistIdx.size()-1;i>=0;i--)
        {
            if(vDistIdx[i].first<thDist)
            {
                break;
            }
            else
            {
//                num ++;
                mvuRight[vDistIdx[i].second]=-1;
                mvDepth[vDistIdx[i].second]=-1;
            }
        }
        for( auto &i:vDistIdx )
        {
            if(mvuRight[i.second] != -1)
            {
                cv::DMatch x;
                x.queryIdx = i.second;
                x.trainIdx = mvuRight[i.second];
                matches_curr_good_.push_back(x);
            }
        }

        Mat img_goodmatch;
        drawMatches ( curr_->left_, keypoints_curr_left_, curr_->right_ ,keypoints_curr_right_, matches_curr_good_, img_goodmatch );
        cv::resize(img_goodmatch, img_goodmatch, cv::Size(1000,400));
        cv::imshow ( "strero matches: ", img_goodmatch );
        cout<<"strero matches: "<<matches_curr_good_.size() <<endl;
//        std::cout << num << "   all:  "  <<  mvuRight.size() << " ORB num " << N << std::endl;
    }

    void Track::matchCurr()
    {
        int N = keypoints_curr_left_.size();
        mvuRight = vector<float>(N,-1.0f);
        mvDepth = vector<float>(N,-1.0f);
//        matcher_flann_.match(descriptors_curr_left_, descriptors_curr_right_, matches_curr_);

        matcher_->knnMatch(descriptors_curr_left_, descriptors_curr_right_, matches_curr_, 2);

        // select the best matches


        vector<pair<int, int> > vDistIdx;
        vDistIdx.reserve(N);
        const float minZ = curr_->camera_->b_;
        const float minD = 0;
        const float maxD = curr_->camera_->bf_ / minZ;
        int iL = 0;
        for ( auto& m:matches_curr_ )
        {
//                for(auto & n:m)
//                {
//                    cout << n.distance << "  ";
//                }
//                 std::cout << std::endl;


            double perfect = m[0].distance / m[1].distance;
            if(perfect < 0.5 && m[0].distance < 30)
            {
                float disparity =  keypoints_curr_left_[ m[0].queryIdx ].pt.x-
                        keypoints_curr_right_[ m[0].trainIdx ].pt.x;
                if(disparity>=minD && disparity<maxD)
                {
                    if(disparity<=0)
                    {
                        disparity=0.01;
                        keypoints_curr_right_[ m[0].trainIdx ].pt.x = keypoints_curr_left_[ m[0].queryIdx ].pt.x-0.01;
                    }
                    mvDepth[iL]=curr_->camera_->bf_/disparity;
                    mvuRight[iL] = keypoints_curr_right_[ m[0].trainIdx ].pt.x;
                    vDistIdx.push_back(pair<int,int>(m[0].distance,iL));
                }

                matches_curr_good_.push_back(m[0]);
//                vIniMatches_.push_back(m[0].trainIdx);
//                     std::cout << m[0].trainIdx << std::endl;
            }
            iL ++;
        }

        sort(vDistIdx.begin(),vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size()/2].first;
        const float thDist = median;

//        int num = 0;
        for(int i=vDistIdx.size()-1;i>=0;i--)
        {
            if(vDistIdx[i].first<thDist)
            {
                break;
            }
            else
            {
//                num ++;
                mvuRight[vDistIdx[i].second]=-1;
                mvDepth[vDistIdx[i].second]=-1;
            }
        }
//        for( auto &i:vDistIdx )
//        {
//            if(mvuRight[i.second] != -1)
//            {
//                cv::DMatch x;
//                x.queryIdx = i.second;
//                x.trainIdx = mvuRight[i.second];
//                matches_curr_good_.push_back(x);
//            }
//        }


//        float disparity = (uL-bestIdxR);
//
//        if(disparity>=minD && disparity<maxD)
//        {
//            if(disparity<=0)
//            {
//                disparity=0.01;
//                bestIdxR = uL-0.01;
//            }
//            mvDepth[iL]=curr_->camera_->bf_/disparity;
//            mvuRight[iL] = bestIdxR;
//            vDistIdx.push_back(pair<int,int>(bestDist,iL));
//        }


        std::cout << "strero matches: " << matches_curr_good_.size() << std::endl;



        //绘制匹配结果
//        Mat img_match;
        Mat img_goodmatch;
//        drawMatches ( curr_->left_, keypoints_curr_left_, curr_->right_ ,keypoints_curr_right_, matches_curr_, img_match );
        drawMatches ( curr_->left_, keypoints_curr_left_, curr_->right_ ,keypoints_curr_right_, matches_curr_good_, img_goodmatch );
//        cv::imshow ( "all", img_match );
        cv::resize(img_goodmatch, img_goodmatch, cv::Size(1000,400));
        cv::imshow ( "strero matches: ", img_goodmatch );

    }

    void Track::featureMatching()
    {
        vector< vector<cv::DMatch>> matches;
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



        matcher_->knnMatch(desp_map, descriptors_curr_left_, matches, 2);

        match_3dpts_.clear();
        match_2dkp_index_.clear();
        // select the best matches
        vector<cv::DMatch> good_matchs;
        for ( auto& m:matches )
        {

            double perfect = m[0].distance / m[1].distance;
            if(perfect < 0.6 /*&& m[0].distance < 100*/)
            {
                good_matchs.push_back(m[0]);
                match_3dpts_.push_back( candidate[m[0].queryIdx] );
                match_2dkp_index_.push_back( m[0].trainIdx );
            }
        }

        vector<cv::KeyPoint>  cuur_key_;
//        detector_->detect ( ref_->left_, cuur_key_ );
        for( auto &c:candidate)
        {
            cv::KeyPoint key;
            key.pt = curr_->camera_->world2pixel( c->pos_, curr_->T_c_w_ );
            cuur_key_.push_back(key);
        }


        Mat match3d;
        drawMatches ( ref_->left_, cuur_key_, curr_->left_ ,keypoints_curr_left_, good_matchs, match3d );
        cv::resize(match3d, match3d, cv::Size(1000,400));
        cv::imshow ( "match3d", match3d );



//        matcher_flann_.match ( desp_map, descriptors_curr_left_, matches );
//        // select the best matches
//        float min_dis = std::min_element (
//                matches.begin(), matches.end(),
//                [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
//                {
//                    return m1.distance < m2.distance;
//                } )->distance;

//        match_3dpts_.clear();
//        match_2dkp_index_.clear();
//        for ( cv::DMatch& m : matches )
//        {
//            if ( m.distance < max<float> ( min_dis*match_ratio_, 50.0 ) )
//            {
//                match_3dpts_.push_back( candidate[m.queryIdx] );
//                match_2dkp_index_.push_back( m.trainIdx );
//            }
//        }
        cout<<"3D pointd: "<<desp_map.size()<<"\n2D points: "<<descriptors_curr_left_.size()<<endl;
        cout<<"3D-2D matches: "<<match_3dpts_.size() <<endl;
    }


    void Track::addKeyFrame()
    {
        if ( map_->keyframes_.empty() )
        {
            // first key-frame, add all 3d points into map
            for ( size_t i=0; i<keypoints_curr_left_.size(); i++ )
            {
                // TODO
                double d = mvDepth[i] ;//findDepth ( keypoints_curr_left_[i] , keypoints_curr_right_[ 0 ]);
                if ( d < 0 )
                    continue;
                Point3d p_world = ref_->camera_->pixel2world (
                        Point2d( keypoints_curr_left_[i].pt.x , keypoints_curr_left_[i].pt.y ), curr_->T_c_w_, d
                );
                Point3d cam = ref_->getCamCenter();
                Point3d n = p_world - cam;
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

    // Bit set count operation from
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    int Track::DescriptorDistance(const Mat &a, const Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist=0;

        for(int i=0; i<8; i++, pa++, pb++)
        {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

    double Track::findDepth ( )
    {
        ;
    }

    bool Track::poseEstimationPnP()
    {
        // construct the 3d 2d observations
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;

        for ( int index:match_2dkp_index_ )
        {
            pts2d.push_back ( keypoints_curr_left_[index].pt );
        }
        for ( MapPoint::Ptr pt:match_3dpts_ )
        {
            pts3d.push_back( pt->pos_ /*getPositionCV()*/ );
        }

        Mat K = ( cv::Mat_<double> ( 3,3 ) <<
                                           ref_->camera_->fx_, 0, ref_->camera_->cx_,
                0, ref_->camera_->fy_, ref_->camera_->cy_,
                0,0,1
        );
        Mat rvec, tvec, inliers, R;
        if(pts3d.size() < 10)
            return false;
        cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
        num_inliers_ = inliers.rows;
        cv::Rodrigues(rvec, R);
        double pose[6] = { rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2), tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
        cout<<"pnp inliers: "<<num_inliers_<<endl;
        cv::hconcat(R,tvec,T_c_w_estimated_);
        cout<<"T_c_w_estimated_(before BA): "<<endl<<T_c_w_estimated_<<endl;

        ceres::Problem problem;
        for(int i=0; i<inliers.rows; i++ )
        {
            int index = inliers.at<int> ( i,0 );
            ceres::CostFunction* cost_function = MinReprojectionError::Create( pts3d[index], pts2d[index], K );
            problem.AddResidualBlock(cost_function, NULL, pose);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 30;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        rvec.at<double>(0) = pose[0];
        rvec.at<double>(1) = pose[1];
        rvec.at<double>(2) = pose[2];
        tvec.at<double>(0) = pose[3];
        tvec.at<double>(1) = pose[4];
        tvec.at<double>(2) = pose[5];

        cv::Rodrigues(rvec, R);
        cv::hconcat(R,tvec,T_c_w_estimated_);
        cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_<<endl;

        return true;
        /*// using bundle adjustment to optimize the pose
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
        Block* solver_ptr = new Block ( linearSolver );
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm ( solver );

        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setId ( 0 );
        pose->setEstimate ( g2o::SE3Quat (
                T_c_w_estimated_.rotation_matrix(), T_c_w_estimated_.translation()
        ));
        optimizer.addVertex ( pose );

        // edges
        for ( int i=0; i<inliers.rows; i++ )
        {
            int index = inliers.at<int> ( i,0 );
            // 3D -> 2D projection
            EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
            edge->setId ( i );
            edge->setVertex ( 0, pose );
            edge->camera_ = curr_->camera_.get();
            edge->point_ = Vector3d ( pts3d[index].x, pts3d[index].y, pts3d[index].z );
            edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );
            edge->setInformation ( Eigen::Matrix2d::Identity() );
            optimizer.addEdge ( edge );
            // set the inlier map points
            match_3dpts_[index]->matched_times_++;
        }

        optimizer.initializeOptimization();
        optimizer.optimize ( 10 );*/

//        T_c_w_estimated_ = SE3 (
//                pose->estimate().rotation(),
//                pose->estimate().translation()
//        );


    }

    bool Track::checkEstimatedPose()
    {
        // check if the estimated pose is good
        if ( num_inliers_ < min_inliers_ )
        {
            cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
            return false;
        }


        Mat R_c_w = ref_->T_c_w_.colRange(0,3).rowRange(0,3),
            t_c_w = ref_->T_c_w_.colRange(3,4).rowRange(0,3),
            R_c_w_e = T_c_w_estimated_.colRange(0,3).rowRange(0,3),
            t_c_w_e = T_c_w_estimated_.colRange(3,4).rowRange(0,3);

        Mat R_r_c, t_r_c, T_r_c, r_r_c, d;

        R_r_c = R_c_w * R_c_w_e.inv();
        t_r_c = -R_c_w * R_c_w_e.inv()* t_c_w_e + t_c_w;
        cv::hconcat(R_r_c,t_r_c,T_r_c);

        cv::Rodrigues(R_r_c, r_r_c);
        cv::vconcat(r_r_c, t_r_c, d);

        double d_n = my_nom(d);

        if ( d_n > 5.0 )
        {
            cout<<"reject because motion is too large: "<<d_n <<endl;
            return false;
        }
        return true;
    }

    bool Track::checkKeyFrame()
    {
        Mat R_c_w = ref_->T_c_w_.colRange(0,3).rowRange(0,3),
                t_c_w = ref_->T_c_w_.colRange(3,4).rowRange(0,3),
                R_c_w_e = T_c_w_estimated_.colRange(0,3).rowRange(0,3),
                t_c_w_e = T_c_w_estimated_.colRange(3,4).rowRange(0,3);

        Mat R_r_c, t_r_c, T_r_c, r_r_c, d;

        R_r_c = R_c_w * R_c_w_e.inv();
        t_r_c = -R_c_w * R_c_w_e.inv() * t_c_w_e + t_c_w;
        cv::hconcat(R_r_c,t_r_c,T_r_c);

        cv::Rodrigues(R_r_c, r_r_c);
        cv::vconcat(r_r_c, t_r_c, d);


        if ( cv::norm(r_r_c) >key_frame_min_rot || cv::norm(t_r_c) >key_frame_min_trans )
            return true;
        return false;
    }

    double Track::my_nom(Mat src)
    {
        double sum = 0;
        for(int i=0;i<src.rows;++i)
        {
            for(int j=0;j<src.cols;j++)
            {
                sum += src.at<double>(i,j) * src.at<double>(i,j);
            }
        }
        return std::sqrt(sum);
    }

    void Track::optimizeMap()
    {
        // remove the hardly seen and no visible points
        for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
        {
            if ( !curr_->isInFrame(iter->second->pos_) )
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }
            float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
            if ( match_ratio < map_point_erase_ratio_ )
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }

            double angle = getViewAngle( curr_, iter->second );
            if ( angle > M_PI/6. )
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }
            if ( iter->second->good_ == false )
            {
                // TODO try triangulate this map point
            }
            iter++;
        }

        if ( match_2dkp_index_.size() < 500 )
            addMapPoints();
        if ( map_->map_points_.size() > 5000 )
        {
            // TODO map is too large, remove some one
            map_point_erase_ratio_ += 0.05;
        }
        else
            map_point_erase_ratio_ = 0.1;
        cout<<"map points: "<<map_->map_points_.size()<<endl;
    }

    void Track::addMapPoints()
    {
        // add the new map points into map
        vector<bool> matched(keypoints_curr_left_.size(), false);
        for ( int index:match_2dkp_index_ )
            matched[index] = true;
        for ( int i=0; i<keypoints_curr_left_.size(); i++ )
        {
            if ( matched[i] == true )
                continue;
            double d = mvDepth[i];//ref_->findDepth ( keypoints_curr_left_[i] );
            if ( d<0 )
                continue;
            Point3d p_world = ref_->camera_->pixel2world (
                    keypoints_curr_left_[i].pt,
                    curr_->T_c_w_, d
            );
            Point3d n = p_world - ref_->getCamCenter();
            double n_n = std::sqrt( n.x*n.x + n.y*n.y + n.z*n.z );
            n.x /= n_n;
            n.y /= n_n;
            n.z /= n_n;
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                    p_world, n, descriptors_curr_left_.row(i).clone(), curr_.get()
            );
            map_->insertMapPoint( map_point );
        }
    }

    double Track::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
    {
        Point3d n = point->pos_ - frame->getCamCenter();
        double n_n = std::sqrt( n.x*n.x + n.y*n.y + n.z*n.z );
        n.x /= n_n;
        n.y /= n_n;
        n.z /= n_n;

        return acos( n.x*point->norm_.x + n.y*point->norm_.y + n.z*point->norm_.z
                /*n.transpose()*point->norm_*/ );
    }
}