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
#include <chrono>

//#include <pangolin/pangolin.h>

#include "config.hpp"
#include "camera.hpp"
#include "frame.hpp"
#include "track.hpp"
#include "common.hpp"
#include "unistd.h"
#include "map.hpp"
#include "mappoint.hpp"

const string PathToSequence = "/Users/lixin/Documents/KITTI/data_odometry/dataset/sequences/14";//07
const string ParameterFile = "/Users/lixin/Documents/KITTI/KITTI00-02.yaml";

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
void DrawTrajectory(vector<vector<double>>  map_points);

int main(int argc, char **argv)
{
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(PathToSequence, vstrImageLeft, vstrImageRight, vTimestamps);


    const int nImages = vstrImageLeft.size();

    //// Create VO system.
    StereoVO::Config::setParameterFile(ParameterFile);
    StereoVO::Track::Ptr track ( new StereoVO::Track );
    StereoVO::Camera::Ptr camera ( new StereoVO::Camera );


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    char text[100];
    Mat traj = Mat::zeros(1000, 600, CV_8UC3);

    // Main loop
    Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
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

        StereoVO::Frame::Ptr pFrame = StereoVO::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->left_ = imLeft;
        pFrame->right_ = imRight;
        pFrame->time_stamp_ = tframe;
        Mat R = Mat::eye(3,3,CV_64F),t = Mat::zeros(3,1,CV_64F);
        cv::hconcat(R,t,pFrame->T_c_w_);



//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

//         Pass the images to the SLAM system
//        SLAM.TrackStereo(imLeft,imRight,tframe);
        track->addFrame(pFrame);
        cv::imshow("Image", imLeft);
//        cv::waitKey( 30 );
        if(cv::waitKey( 1 ) == 27) break;

//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

//        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

//        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
//        double T=0;
//        if(ni<nImages-1)
//            T = vTimestamps[ni+1]-tframe;
//        else if(ni>0)
//            T = tframe-vTimestamps[ni-1];
//
//        if(ttrack<T)
//            usleep((T-ttrack)*1e6);
        Mat R_c_w = track->curr_->T_c_w_.colRange(0,3).rowRange(0,3),
                t_c_w = track->curr_->T_c_w_.colRange(3,4).rowRange(0,3);
        Mat ret = -R_c_w.inv()*t_c_w;
        Point3d cam_t( ret.at<double>(0),  ret.at<double>(1), ret.at<double>(2));


        int x = (int)(cam_t.x) + 300,
                y = (int)(-cam_t.z) + 700;
        cv::circle(traj, cv::Point(x,y), 1, CV_RGB(255,0,0), 2 );
        cv::rectangle(traj, cv::Point(10,30), cv::Point(550,50), CV_RGB(0,0,0), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm",cam_t.x, cam_t.y, cam_t.z);
        cv::putText(traj, text, cv::Point(10,50), 1, 1, cv::Scalar::all(255));
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

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

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
