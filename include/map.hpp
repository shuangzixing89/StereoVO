//
// Created by lixin on 18-12-19.
//

#ifndef STEREOVO_MAP_HPP
#define STEREOVO_MAP_HPP

#include "common.hpp"
#include "frame.hpp"
#include "mappoint.hpp"

namespace StereoVO
{
    class Map
    {
    public:
        typedef shared_ptr<Map> Ptr;
        unordered_map<unsigned long, MapPoint::Ptr >  map_points_;        // all landmarks
        unordered_map<unsigned long, Frame::Ptr >     keyframes_;         // all key-frames

        Map() {}

        void insertKeyFrame( Frame::Ptr frame );
        void insertMapPoint( MapPoint::Ptr map_point );
    };
}

#endif //STEREOVO_MAP_HPP
