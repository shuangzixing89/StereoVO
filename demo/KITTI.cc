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

#include "config.hpp"
#include "camera.hpp"
#include "frame.hpp"
#include "track.hpp"
#include "common.hpp"
#include "unistd.h"

const string PathToSequence = "/Users/lixin/Documents/KITTI/data_odometry/dataset/sequences/01";
const string ParameterFile = "/Users/lixin/Documents/KITTI/KITTI00-02.yaml";

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

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

    // Main loop
    Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            continue;
            //return 1;
        }

        StereoVO::Frame::Ptr pFrame = StereoVO::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->left_ = imLeft;
        pFrame->right_ = imLeft;
        pFrame->time_stamp_ = tframe;



        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

//         Pass the images to the SLAM system
//        SLAM.TrackStereo(imLeft,imRight,tframe);
        cv::imshow("Image", imLeft);
//        cv::waitKey( 30 );
        if(cv::waitKey( 30 ) == 27) break;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    return 0;
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
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