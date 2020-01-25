#pragma once

#include <vector>
#include <string>

#include <pangolin/pangolin.h>

#include <tbb/tbb.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <stereo_processor.h>

class PoseSubscriberBase;

class PangolinPlotter{

public:

    struct Parameters{
        std::vector<std::string> pose_topics;
        std::vector<std::string> pose_names;
        std::vector<pangolin::Colour> colours;

        StereoProcessor::Parameters stereo_param;
    };

    

    PangolinPlotter(const Parameters& param);

    void run();

private:

    void setupSubs();

    Parameters param_;
    std::vector<std::shared_ptr<PoseSubscriberBase>> subs_;
    StereoProcessor stereo_sub_;

    using PoseMsg = geometry_msgs::PoseWithCovarianceStamped;
    using PoseQueue = tbb::concurrent_bounded_queue<PoseMsg>;
    std::vector<PoseQueue*> pose_queues_;

    tbb::concurrent_bounded_queue<StereoProcessor::SyncedData>* camera_queues_;

    pangolin::DataLog data_log;

};