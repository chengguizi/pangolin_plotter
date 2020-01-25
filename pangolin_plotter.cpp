
#include <pangolin_plotter.hpp>

#include <cassert>

#include <pose_subscribers.hpp>
// #include <ros/topic_manager.h>
// #include <ros/publication.h>

#include <pangolin/display/image_view.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>

#include <limits>

#include <vis_utils.hpp>
void setImageData(unsigned char * imageArray, int size){
  for(int i = 0 ; i < size;i++) {
    imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
  }
}


constexpr int UI_WIDTH = 200;

// helper function
std::string getTopicType(const std::string& topic_name)
{
    // Obtain available topics and types
    static ros::master::V_TopicInfo master_topics;


    ros::master::getTopics(master_topics);

    auto itr = std::find_if(master_topics.begin(),master_topics.end(), 
        [&topic_name](ros::master::TopicInfo const& info){
            return info.name == topic_name;
        } 
    );

    if (itr != master_topics.end())
        return itr->datatype;
    else
        return std::string();
}

PangolinPlotter::PangolinPlotter(const Parameters& param) : stereo_sub_(param.stereo_param)
{
    assert(param.pose_names.size() == param.pose_topics.size() && param.pose_topics.size() > 0);
    param_ = param;

    subs_.resize(param.pose_names.size());
    pose_queues_.resize(param.pose_names.size());
    
    setupSubs();

    camera_queues_ = &stereo_sub_.multi_camera_buffer;

}

void PangolinPlotter::setupSubs()
{
    ros::NodeHandle local_nh("~");
    for (size_t i = 0; i < param_.pose_names.size(); i++)
    {
        std::string datatype = getTopicType(param_.pose_topics[i]);

        std::cout  << "datatype = " << datatype << std::endl;

        if (!datatype.empty())
        {
            bool subscribed = true;
            if (datatype == "geometry_msgs/PoseWithCovarianceStamped")
                subs_[i].reset(new PoseSubscriber<geometry_msgs::PoseWithCovarianceStamped>(local_nh, param_.pose_topics[i]));
            else if (datatype == "geometry_msgs/PoseStamped")
                subs_[i].reset(new PoseSubscriber<geometry_msgs::PoseStamped>(local_nh, param_.pose_topics[i]));
            else if (datatype == "nav_msgs/Odometry")
                subs_[i].reset(new PoseSubscriber<nav_msgs::Odometry>(local_nh, param_.pose_topics[i]));
            else
                subscribed = false;
            
            if (subscribed)
                pose_queues_[i] = &subs_[i]->pose_buffer;
            else
                std::cout << param_.pose_topics[i] << " has unsupported type" << std::endl;

        }else{
            std::cout << param_.pose_topics[i] << " is not published?" << std::endl;
            pose_queues_[i] = nullptr;
        }
        
    }
    
}

void PangolinPlotter::run()
{
    //// Setup ////

    pangolin::CreateWindowAndBind("Pangolin Plotter", 1800, 1000);
    
    // enable spatial occlusion awareness in OpenGL
    glEnable(GL_DEPTH_TEST);


    // Create views for video stream of cameras
    pangolin::View& img_view_display =
        pangolin::CreateDisplay()
            .SetBounds(0.4, 1.0, pangolin::Attach::Pix(UI_WIDTH), 0.4)
            .SetLayout(pangolin::LayoutEqual);
            
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));

    // Add stereo images
    std::vector<std::shared_ptr<pangolin::ImageView>> img_view;

    // left image
    std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

    img_view.push_back(iv);
    img_view_display.AddDisplay(*iv);
    // iv->SetBackgroundColour(pangolin::Colour::Blue());
    iv.reset(new pangolin::ImageView);
    img_view.push_back(iv);
    img_view_display.AddDisplay(*iv);


    // pangolin::View& plot_display = pangolin::CreateDisplay().SetBounds(
    //     0.0, 0.4, pangolin::Attach::Pix(UI_WIDTH), 1.0);

    // pangolin::Plotter* plotter =
    //     new pangolin::Plotter(&data_log, 0.0, 20, 0, 20, 0.01f, 0.01f);
    // plot_display.AddDisplay(*plotter);
    // plotter->SetBackgroundColour(pangolin::Colour::White());

    // create camera view
    Eigen::Affine3d T_i_c(Eigen::Affine3d::Identity());
    T_i_c.linear() << 0,0,1 ,
                     -1 , 0 , 0,
                     0, -1, 0;

    Eigen::Vector3d cam_p(50, -250, -50);

    cam_p = T_i_c.rotation() * cam_p;
    // cam_p[2] = 1;

    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(640, 480, 10000, 10000, 320, 240, 0.001, 10000),
        pangolin::ModelViewLookAt(cam_p[0], cam_p[1], cam_p[2], 0, 0, 0,
                                  pangolin::AxisZ));


    pangolin::View& display3D =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
            .SetHandler(new pangolin::Handler3D(camera));


    ///////////////

    std::vector<PoseMsg> pose_msg(pose_queues_.size());
    StereoProcessor::SyncedData synced_data;
    std::vector<Eigen::aligned_vector<Eigen::Vector3d>> vio_t_w_i(pose_queues_.size());

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::Var<bool> subscribe("ui.re-subscribe", false, false, true);
    pangolin::Var<bool> clear("ui.clear", false, false, true);
    pangolin::Var<bool> follow("ui.follow", true, false, true);
    pangolin::Var<bool> from_top("ui.fromTop", false, false, true);
    pangolin::Var<bool> reset_view("ui.resetView", false, false, true);

    std::vector<pangolin::Var<bool>*> toggles(pose_queues_.size());

    for (size_t i = 0; i < pose_queues_.size(); i++)
    {
        toggles[i] = new pangolin::Var<bool>("ui." + param_.pose_names[i], true, false, true);
    }

    pangolin::GlFont glFont("/home/huimin/git/catkin_ws/src/Pangolin/src/_embed_/fonts/AnonymousPro.ttf", 24.0);

    std::vector<Eigen::Affine3d> poses_eigen(pose_queues_.size());
    
    while (!pangolin::ShouldQuit() && ros::ok()) {

        // check if need to re-subscribe

        if (subscribe)
        {
            setupSubs();
            subscribe = false;
        }

        if (clear)
        {
            for (auto& track : vio_t_w_i)
                track.clear();
            
            clear = false;
        }

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // camera 3d view
        display3D.Activate(camera);

        if (from_top)
        {
            Eigen::Vector3d cam_p(0, 0, 1000);
            // cam_p = T_i_c.rotation() * cam_p;
            camera.SetModelViewMatrix(pangolin::ModelViewLookAt(cam_p[0], cam_p[1], cam_p[2], 0, 0, 0,
                                  pangolin::AxisX));

            from_top = false;
        }

        if (reset_view)
        {
            Eigen::Vector3d cam_p(50, -250, -50);

            cam_p = T_i_c.rotation() * cam_p;

            camera.SetModelViewMatrix(pangolin::ModelViewLookAt(cam_p[0], cam_p[1], cam_p[2], 
                0, 0, 0,
                                        pangolin::AxisZ));
            reset_view = false;
        }

        // Obtain poses
        for (size_t i = 0; i < pose_queues_.size(); i++)
        {
            if (pose_queues_[i] == nullptr) continue;

            while (pose_queues_[i]->try_pop(pose_msg[i]))
            {
                // // empty pose!
                // if (pose_msg[i].header.stamp == ros::Time()) continue;

                tf::poseMsgToEigen(pose_msg[i].pose.pose, poses_eigen[i]);

                vio_t_w_i[i].emplace_back(poses_eigen[i].translation());
            }

            // skip rendering if toggle is off
            if (*toggles[i] == false || vio_t_w_i[i].size() == 0) continue;

            // follow
            if (follow && i == 0)
            {
                auto T_w_i = Eigen::Affine3d::Identity();
                T_w_i.translation() = poses_eigen[i].translation();

                camera.Follow((T_w_i).matrix());
            }

            {
                
                // glColor4fv(param_.colours[i].Get());
                visualisation::render_camera((poses_eigen[i] * T_i_c).matrix(), 2.0f, param_.colours[i].Get(), 0.1f);
                
                glPointSize(3);
                Eigen::aligned_vector<Eigen::Vector3d> sub_gt(vio_t_w_i[i].begin(),
                                                vio_t_w_i[i].end());

                pangolin::glDrawLineStrip(sub_gt);
            }


        }

        glPointSize(2);
        pangolin::glDrawAxis(Eigen::Affine3d::Identity().matrix(), 1.0);
        glPointSize(1);
        glColor4f(0.8 ,0.8, 0.8, 0.5);
        pangolin::glDraw_z0(1, 20);


        // Draw text
        for (size_t i = 0; i < pose_queues_.size(); i++)
        {
            if (subs_[i] == nullptr || *toggles[i] == false) continue;
            glColor4fv(param_.colours[i].Get());
            pangolin::GlText txt = glFont.Text(param_.pose_names[i].c_str());
            txt.DrawWindow(display3D.v.r() - 250, display3D.v.t() - 100 - i*20);
            // txt.Draw(0.5, 0.5 + 0.1 * i, 0.5);
        }


        // Obtain images
        img_view_display.Activate();
        if (camera_queues_->try_pop(synced_data))
        {
            pangolin::GlPixFormat fmt;
            fmt.glformat = GL_BGRA;
            fmt.gltype = GL_UNSIGNED_BYTE;
            fmt.scalable_internal_format = GL_RGBA8;
            
            for (int i = 0; i < 2; i++)
            {
                sensor_msgs::Image image = *synced_data.image_ptrs[i];

                // std::cout << (unsigned int)image.data[1230] << " " <<  (unsigned int)synced_data.image_ptrs[i]->data[1230] << std::endl;
                // std::cout << image.encoding << " image " << synced_data.image_ptrs[i]->height << " " << synced_data.image_ptrs[i]->step << std::endl;
                img_view[i]->SetImage(image.data.data(), image.width, image.height, image.step, fmt);
            }
                
        }


        pangolin::FinishFrame();
    }

    std::cout << "run() ends" << std::endl;
}