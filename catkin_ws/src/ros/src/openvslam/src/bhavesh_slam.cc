#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <system.h>
#include <config.h>
#include <openvslam_ros.h>

#include <iostream>
#include <chrono>
#include <numeric>

#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

// void tracking(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path,
//               const std::string& mask_img_path, const bool eval_log, const std::string& map_db_path,
//               const bool rectify) {
//     std::shared_ptr<openvslam_ros::system> ros;
//     // if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
//     //     ros = std::make_shared<openvslam_ros::mono>(cfg, vocab_file_path, mask_img_path);
//     // }
//     // else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo) {
//     //     ros = std::make_shared<openvslam_ros::stereo>(cfg, vocab_file_path, mask_img_path, rectify);
//     // }
//     if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD) {
//         ros = std::make_shared<openvslam_ros::rgbd>(cfg, vocab_file_path, mask_img_path);
//     }
//     else {
//         throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
//     }

//     auto& SLAM = ros->SLAM_;
//     // startup the SLAM process
//     SLAM.startup();

//     // create a viewer object
//     // and pass the frame_publisher and the map_publisher
// #ifdef USE_PANGOLIN_VIEWER
//     pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
// #elif USE_SOCKET_PUBLISHER
//     socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
// #endif

//     // run the viewer in another thread
// #ifdef USE_PANGOLIN_VIEWER
//     std::thread thread([&]() {
//         viewer.run();
//         if (SLAM.terminate_is_requested()) {
//             // wait until the loop BA is finished
//             while (SLAM.loop_BA_is_running()) {
//                 std::this_thread::sleep_for(std::chrono::microseconds(5000));
//             }
//             ros::shutdown();
//         }
//     });
// #elif USE_SOCKET_PUBLISHER
//     std::thread thread([&]() {
//         publisher.run();
//         if (SLAM.terminate_is_requested()) {
//             // wait until the loop BA is finished
//             while (SLAM.loop_BA_is_running()) {
//                 std::this_thread::sleep_for(std::chrono::microseconds(5000));
//             }
//             ros::shutdown();
//         }
//     });
// #endif

//     ros::spin();

//     // automatically close the viewer
// #ifdef USE_PANGOLIN_VIEWER
//     viewer.request_terminate();
//     thread.join();
// #elif USE_SOCKET_PUBLISHER
//     publisher.request_terminate();
//     thread.join();
// #endif

//     // shutdown the SLAM process
//     SLAM.shutdown();

//     auto& track_times = ros->track_times_;
//     if (eval_log) {
//         // output the trajectories for evaluation
//         SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
//         SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
//         // output the tracking times for evaluation
//         std::ofstream ofs("track_times.txt", std::ios::out);
//         if (ofs.is_open()) {
//             for (const auto track_time : track_times) {
//                 ofs << track_time << std::endl;
//             }
//             ofs.close();
//         }
//     }

//     if (!map_db_path.empty()) {
//         // output the map database
//         SLAM.save_map_database(map_db_path);
//     }

//     if (track_times.size()) {
//         std::sort(track_times.begin(), track_times.end());
//         const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
//         std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
//         std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
//     }
// }

std::vector<double> track_times_;
// void callback(const sensor_msgs::ImageConstPtr& msg)
// {

//     cv::Mat imRGB,imD;
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvShare(msg);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     cv::Mat temp = cv_ptr->image;
//     try {
//         imRGB = temp(cv::Rect(0,0,640,480));
//     }
//     catch (std::exception& e)
//     {
//         std::cout<<"RGB Error."<<std::endl;
//     }

//     cv::Mat bgrCombine;
//     try {
//         bgrCombine = temp(cv::Rect(640,0,640,480));
//     }
//     catch(std::exception& e){
//         std::cout<<"Depth Error."<<std::endl;
//     }

//     cv::Mat bgr[3];
//     split(bgrCombine , bgr);

//     imD = bgr[2];

//     const auto tp_1 = std::chrono::steady_clock::now();
//     const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();
// std::cout<<"rgb and d aaya na"<<"\n";
//     // input the current frame and estimate the camera pose
//     SLAM_.feed_RGBD_frame(imRGB, imD, timestamp);
// std::cout<<"feedback aaya na"<<"\n";
//     const auto tp_2 = std::chrono::steady_clock::now();

//     const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
//     track_times_.push_back(track_time);
// }


class ImageGrabber
{

public:
    cv::Mat imRGB,imD;
    decltype(std::chrono::steady_clock::now()) tp_0_;
    ImageGrabber(openvslam::system* pSLAM):mpSLAM(pSLAM){
        tp_0_=(std::chrono::steady_clock::now());
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    //void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    openvslam::system* mpSLAM;

    // double roll, pitch, yaw;
    // bool val;
    // ros::Publisher pos_pub, robotYaw, slam_pitch, Tcw_pos , camCenterPub;
};

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat temp = cv_ptr->image;
    try {
        imRGB = temp(cv::Rect(0,0,640,480));
    }
    catch (std::exception& e)
    {
        std::cout<<"RGB Error."<<std::endl;
    }

    cv::Mat bgrCombine;
    try {
        bgrCombine = temp(cv::Rect(640,0,640,480));
    }
    catch(std::exception& e){
        std::cout<<"Depth Error."<<std::endl;
    }

    cv::Mat bgr[3];
    split(bgrCombine , bgr);

    imD = bgr[2];
    if(imRGB.empty() || imD.empty()){
        return;
    }
    const auto tp_1 = std::chrono::steady_clock::now();
    // const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();
    std::cout<<"rgb and d aaya na"<<"\n";
    // input the current frame and estimate the camera pose
    // std::cout<<imRGB<<"\n\n\n\n\n\n"<<imD<<"\n\n\n";
    mpSLAM->feed_RGBD_frame(imRGB, imD,cv_ptr->header.stamp.toSec());
    std::cout<<"feedback aaya na"<<"\n";
    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
}

int main(int argc, char* argv[]) {


#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif



    ros::init(argc, argv, "run_slam");
    ros::start();
    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = "/home/meditab/work/openvslam/build/orb_vocab/orb_vocab.dbow2";
    auto setting_file_path = "/home/meditab/work/openvslam/example/tum_rgbd/TUM_RGBD_rgbd_1.yaml";
    auto mask_img_path=op.add<popl::Value<std::string>>("", "mask", "masking of the image", "");;
    auto debug_mode=false;
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
    
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(setting_file_path);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    // tracking(cfg, vocab_file_path->value(), mask_img_path->value(), eval_log->is_set(), map_db_path->value(), rectify->value());

    openvslam::system SLAM(cfg , vocab_file_path);
    std::shared_ptr<openvslam::config> cfg_(cfg);
    
    // cv::Mat mask_ = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
    SLAM.startup(true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/RGBD_Image", 1, &ImageGrabber::GrabImage,&igb);



    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    // run the viewer in another thread
#ifdef USE_PANGOLIN_VIEWER
    std::thread thread([&]() {
        viewer.run();
        if (SLAM.terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM.loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            ros::shutdown();
        }
    });
#elif USE_SOCKET_PUBLISHER
    std::thread thread([&]() {
        publisher.run();
        if (SLAM.terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM.loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            ros::shutdown();
        }
    });
#endif


if(!SLAM.terminate_is_requested()){
    ros::spin();
}


#ifdef USE_PANGOLIN_VIEWER
    viewer.request_terminate();
    thread.join();
#elif USE_SOCKET_PUBLISHER
    publisher.request_terminate();
    thread.join();
#endif

    // shutdown the SLAM process
    SLAM.shutdown();

    // auto& track_times = track_times_;
//     if (eval_log) {
//         // output the trajectories for evaluation
//         SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
//         SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
//         // output the tracking times for evaluation
//         std::ofstream ofs("track_times.txt", std::ios::out);
//         if (ofs.is_open()) {
//             for (const auto track_time : track_times_) {
//                 ofs << track_time << std::endl;
//             }
//             ofs.close();
//         }
//     }

//     if (!map_db_path->value().empty()) {
//         // output the map database
//         SLAM.save_map_database(map_db_path->value());
//     }

//     if (track_times_.size()) {
//         std::sort(track_times_.begin(), track_times_.end());
//         const auto total_track_time = std::accumulate(track_times_.begin(), track_times_.end(), 0.0);
//         std::cout << "median tracking time: " << track_times_.at(track_times_.size() / 2) << "[s]" << std::endl;
//         std::cout << "mean tracking time: " << total_track_time / track_times_.size() << "[s]" << std::endl;
//     }


// #ifdef USE_GOOGLE_PERFTOOLS
//     ProfilerStop();
// #endif

//     return EXIT_SUCCESS;
}
