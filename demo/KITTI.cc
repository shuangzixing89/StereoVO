//
// Created by lixin on 18-12-19.
//
#include <fstream>
#include<iomanip>
//#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <chrono>

//#include <pangolin/pangolin.h>

#include "tic_toc.hpp"
#include "config.hpp"
#include "camera.hpp"
#include "frame.hpp"
#include "track.hpp"
#include "common.hpp"
#include "unistd.h"
#include "map.hpp"
#include "mappoint.hpp"

string PathToSequence = "/Users/lixin/Documents/KITTI/data_odometry/dataset/sequences/";//07 14  04 06
const string ParameterFile = "../config/KITTI00-02.yaml";
string GroundtruthFile = "/Users/lixin/Documents/KITTI/data_odometry/dataset/poses/";

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
void LoadTracks(const string &GroundtruthFile, vector<vector<double>> &truths, vector<vector<double>> &truths_R);
void DrawTrajectory(vector<vector<double>>  map_points);

// TODO add keyframe.cc and add a local BA
// TODO feature point should be
int main(int argc, char **argv) {
    PathToSequence.append( string(argv[1]) );
    GroundtruthFile.append( string(argv[1]) );
    GroundtruthFile.append( string(".txt") );

    bool view_t = (bool)std::atoi(argv[2]);
    vector<vector<double>> truths, truths_R;
    if (view_t)
    {
        LoadTracks(GroundtruthFile, truths, truths_R);
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(PathToSequence, vstrImageLeft, vstrImageRight, vTimestamps);


    const int nImages = static_cast<const int>(vstrImageLeft.size());

    //// Create VO system.
    StereoVO::Config::setParameterFile(ParameterFile);
    StereoVO::Track::Ptr track ( new StereoVO::Track );
    StereoVO::Camera::Ptr camera ( new StereoVO::Camera );


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize((unsigned long) nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    char text[100],text_t[100],text_times[100];
    Mat traj = Mat::zeros(1000, 600, CV_8UC3);

    Mat imLeft, imRight;
    //error
    double e_ATE = 0,e_RMSE = 0;
    int ei = 0;
    int begin = 0;

    // Main loop
    for(int ni=begin ; ni<nImages; ni++)
    {
        TicToc time;
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty() || imRight.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            continue;
            //return 1;
        }

//        auto clahe = cv::createCLAHE(1.0, cv::Size(20, 20));
//        clahe->apply(imLeft, imLeft);
//        clahe->apply(imRight, imRight);


        StereoVO::Frame::Ptr pFrame = StereoVO::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->left_ = imLeft;
        pFrame->right_ = imRight;
        pFrame->time_stamp_ = tframe;
        Mat R = Mat::eye(3,3,CV_64F),t = Mat::zeros(3,1,CV_64F);
        cv::hconcat(R,t,pFrame->T_c_w_);



        track->addFrame(pFrame);
        cv::imshow("Image", imLeft);
//        cv::waitKey( 30 );
        if(cv::waitKey( 1 ) == 27) break;

        Mat R_c_w = track->curr_->T_c_w_.colRange(0,3).rowRange(0,3),
                t_c_w = track->curr_->T_c_w_.colRange(3,4).rowRange(0,3);
        Mat ret = -R_c_w.inv()*t_c_w;
        Point3d cam_t( ret.at<double>(0),  ret.at<double>(1), ret.at<double>(2));

        int x = (int)(cam_t.x) + 300,
                y = (int)(-cam_t.z) + 700;
        cv::circle(traj, cv::Point(x,y), 1, CV_RGB(255,0,0), 2 );
        cv::rectangle(traj, cv::Point(0,0), cv::Point(600,100), CV_RGB(0,0,0), CV_FILLED);
        sprintf(text, "Coordinates: x = %6.2fm y = %6.2fm z = %6.2fm",cam_t.x, cam_t.y, cam_t.z);
        if(view_t)
        {
            int x_t = (int)(/*truths[begin][0] - */truths[ni][0]) + 300,
                    y_t = (int)-(/*truths[begin][2] - */truths[ni][2]) + 700;
            cv::circle(traj, cv::Point(x_t,y_t), 1, CV_RGB(255,255,255), 2 );
            sprintf(text_t, "Coordinat_t: x = %6.2fm y = %6.2fm z = %6.2fm", truths[ni][0], truths[ni][1], truths[ni][2]);
            cv::putText(traj, text_t, cv::Point(10,70), 1, 1, cv::Scalar::all(255));
            e_ATE += std::sqrt( (cam_t.x - truths[ni][0]) * (cam_t.x - truths[ni][0]) +
                            (cam_t.y - truths[ni][1]) * (cam_t.y - truths[ni][1]) +
                            (cam_t.z - truths[ni][2]) * (cam_t.z - truths[ni][2]));
            Mat t_e = ( cv::Mat_<double> ( 3,1 ) <<-truths[ni][0],-truths[ni][1],-truths[ni][2]);
            Mat R_e = ( cv::Mat_<double> ( 3,3 ) <<truths_R[ni][0],truths_R[ni][1],truths_R[ni][2],
                    truths_R[ni][3],truths_R[ni][4],truths_R[ni][5],
                    truths_R[ni][6],truths_R[ni][7],truths_R[ni][8]);
            t_e = -R_e.inv()*t_e + R_e.inv()*t_c_w;
            R_e = R_e.inv()*R_c_w;
            Mat rvec;
            cv::Rodrigues(R_e,rvec);
            double e_r = cv::norm(rvec);
            e_r += cv::norm(t);
            e_RMSE += e_r;
            ei ++;
        }
        cv::putText(traj, text, cv::Point(10, 50), 1, 1, cv::Scalar::all(255));
        sprintf(text_times, "PnP times:%7.2fms  Mappoints: %4ld  all cost: %5.2fms", track->PnP_times_, track->map_->map_points_.size(), time.toc());
        cv::putText(traj, text_times, cv::Point(10,90), 1, 1, cv::Scalar::all(255));
        cv::imshow("traj", traj );


        /*if(ni == 10)
        {
            vector<vector<double>> map_points;

            for ( auto iter = track->map_->map_points_.begin(); iter != track->map_->map_points_.end(); iter++)
            {
                vector<double> map_point;
                map_point.push_back(iter->second->pos_.x);
                map_point.push_back(iter->second->pos_.y);
                map_point.push_back(iter->second->pos_.z);
//                std::cout << map_point[0] << std::endl;
                map_points.push_back(map_point);
            }
            DrawTrajectory(map_points);
        }*/

    }

    if(view_t)
    std::cout << "ATE:" << std::sqrt(e_ATE/(double)ei) << std::endl;
    std::cout << "RMSE:" << std::sqrt(e_RMSE/(double)ei) << std::endl;
    // 04 ATE:2.34239   RMSE:19.5853
    // 07 ATE:2.17038   RMSE:10.2887
    return 0;
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    string strPathTimeFile = strPathToSequence + "/times.txt";
    ifstream fTimes(strPathTimeFile.c_str());
    if ( !fTimes )
    {
        std::cout << "can not open times.txt" <<endl;
        return;
    }
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = static_cast<const int>(vTimestamps.size());
    vstrImageLeft.resize((unsigned long) nTimes);
    vstrImageRight.resize((unsigned long) nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

/*
void DrawTrajectory(vector<vector<double>> map_points)
{
    if (map_points.empty() )
    {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Point3D Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);



        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: map_points)
        {
            glColor3f(0,0,255);
            glVertex3d( p[0] ,p[1], p[2]  );
        }
        glEnd();

        // pangolin::SaveFramebuffer();

        pangolin::FinishFrame();
        usleep(3500);   // sleep 5 ms
    }
}*/

void LoadTracks(const string &GroundtruthFile, vector<vector<double>> &truths, vector<vector<double>> &truths_R)
{
    ifstream fTruth(GroundtruthFile.c_str());
    if (!fTruth)
    {
        std::cout << "can not open GroudtruthFile" << endl;
        return ;
    }
    while (!fTruth.eof())
    {
        double t[12];
        for (auto &d:t)
        {
            fTruth >> d;
        }
        //        fTruth >> t;
        vector<double> ts;
        vector<double> R;
        ts.push_back(t[3]);
        ts.push_back(t[7]);
        ts.push_back(t[11]);
        R.push_back(t[0]);
        R.push_back(t[1]);
        R.push_back(t[2]);
        R.push_back(t[4]);
        R.push_back(t[5]);
        R.push_back(t[6]);
        R.push_back(t[8]);
        R.push_back(t[9]);
        R.push_back(t[10]);
        truths.push_back(ts);
        truths_R.push_back(R);
    }
}
