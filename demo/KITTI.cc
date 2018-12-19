//
// Created by lixin on 18-12-19.
//
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "config.hpp"
#include "track.hpp"

const string ParameterFile = "";

int main(int argc, char **argv)
{
    StereoVO::Config::setParameterFile(ParameterFile);

    return 0;
}