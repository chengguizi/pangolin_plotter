#include <ros/ros.h>
#include <pangolin_plotter.hpp>

#include <thread>

int main(int argc, char * argv[])
{
    // cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);

    ros::init(argc, argv, "pangolin_plotter");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");

    PangolinPlotter::Parameters plotter_param;
    plotter_param.pose_names.push_back("RS T265 VIO");
    plotter_param.pose_topics.push_back("/rs2_ros/fisheye/pose_cov");
    plotter_param.colours.push_back(pangolin::Colour::Black());

    plotter_param.pose_names.push_back("ZED VO");
    plotter_param.pose_topics.push_back("/zed/pose");
    plotter_param.colours.push_back(pangolin::Colour(0.5,0.5,0.5));

    plotter_param.pose_names.push_back("Ours (Velocity EKF)");
    plotter_param.pose_topics.push_back("/ekf_fusion/pose_local");
    plotter_param.colours.push_back(pangolin::Colour::Red());

    plotter_param.pose_names.push_back("VINS-Fusion");
    plotter_param.pose_topics.push_back("/vins_estimator/odometry");
    plotter_param.colours.push_back(pangolin::Colour::Blue());

    plotter_param.pose_names.push_back("MSCKF");
    plotter_param.pose_topics.push_back("/firefly_sbx/vio/odom");
    plotter_param.colours.push_back(pangolin::Colour::Green()); // Colour(1.0,1.0,1.0)

    plotter_param.stereo_param.queue_size = 3;
    plotter_param.stereo_param.left_topic = "/zed/left/image_raw_color";
    plotter_param.stereo_param.right_topic = "/zed/right/image_raw_color";
    plotter_param.stereo_param.left_info_topic = "/zed/left/camera_info_raw";
    plotter_param.stereo_param.right_info_topic = "/zed/right/camera_info_raw";

    PangolinPlotter plotter(plotter_param); // DO NOT INCLUDE PARENTHESIS

    std::cout  << "PangolinPlotter Initiated" << std::endl;


    ros::AsyncSpinner spinner(4);
    spinner.start();

    // std::thread t_gui(&PangolinPlotter::run, plotter);
    plotter.run();

    std::cout << "main exit" << std::endl;

    return 0;
}