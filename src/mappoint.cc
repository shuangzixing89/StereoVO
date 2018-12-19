//
// Created by lixin on 18-12-19.
//

#include "mappoint.hpp"

namespace StereoVO
{
    MapPoint::MapPoint()
            : id_(-1), pos_(Point3d(0,0,0)), norm_(Point3d(0,0,0)), good_(true), visible_times_(0), matched_times_(0)
    {

    }

    MapPoint::MapPoint ( long unsigned int id, const Point3d& position, const Point3d& norm, Frame* frame, const Mat& descriptor )
            : id_(id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor)
    {
        observed_frames_.push_back(frame);
    }

    MapPoint::Ptr MapPoint::createMapPoint()
    {
        return MapPoint::Ptr(
                new MapPoint( factory_id_++, Point3d(0,0,0), Point3d(0,0,0) )
        );
    }

    MapPoint::Ptr MapPoint::createMapPoint (
            const Point3d& pos_world,
            const Point3d& norm,
            const Mat& descriptor,
            Frame* frame )
    {
        return MapPoint::Ptr(
                new MapPoint( factory_id_++, pos_world, norm, frame, descriptor )
        );
    }

    unsigned long MapPoint::factory_id_ = 0;
}