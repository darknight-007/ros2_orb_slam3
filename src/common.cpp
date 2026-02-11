/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    //* SLAM output publishers
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/camera_pose", 10);
    map_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/map_points", 10);
    tracked_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/tracked_points", 10);
    trajectory_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/orb_slam3/trajectory", 10);
    tracking_state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/orb_slam3/tracking_state", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    trajectory_msg_.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "SLAM publishers initialized on /orb_slam3/*");
    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 

    // Guard: only publish if SLAM system is initialized
    if (!pAgent) return;
    
    //* Publish SLAM outputs
    rclcpp::Time stamp = this->get_clock()->now();
    
    // Publish tracking state
    publishTrackingState();
    
    // Only publish pose and map data if tracking is OK
    int trackingState = pAgent->GetTrackingState();
    if (trackingState == 2) // OK state
    {
        // Publish camera pose and TF
        publishCameraPose(Tcw, stamp);
        
        // Publish tracked map points (current frame)
        std::vector<ORB_SLAM3::MapPoint*> trackedPoints = pAgent->GetTrackedMapPoints();
        publishMapPoints(trackedPoints, stamp, tracked_points_publisher_);
        
        // Publish all map points (full map, throttled to reduce bandwidth)
        static int map_publish_counter = 0;
        if (++map_publish_counter % 5 == 0) // Every 5th frame
        {
            std::vector<ORB_SLAM3::MapPoint*> allPoints = pAgent->GetAllMapPoints();
            publishMapPoints(allPoints, stamp, map_points_publisher_);
            
            // Publish keyframe trajectory
            publishTrajectory(stamp);
        }
    }
}

//* Publish camera pose as PoseStamped and broadcast TF
void MonocularMode::publishCameraPose(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp)
{
    // Convert Tcw (camera-to-world) to Twc (world-to-camera) for pose in world frame
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f translation = Twc.translation();
    Eigen::Quaternionf quaternion = Twc.unit_quaternion();

    // Publish PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = translation.x();
    pose_msg.pose.position.y = translation.y();
    pose_msg.pose.position.z = translation.z();
    pose_msg.pose.orientation.x = quaternion.x();
    pose_msg.pose.orientation.y = quaternion.y();
    pose_msg.pose.orientation.z = quaternion.z();
    pose_msg.pose.orientation.w = quaternion.w();
    pose_publisher_->publish(pose_msg);

    // Broadcast TF: map -> camera_link
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "orb_slam3_camera";
    tf_msg.transform.translation.x = translation.x();
    tf_msg.transform.translation.y = translation.y();
    tf_msg.transform.translation.z = translation.z();
    tf_msg.transform.rotation.x = quaternion.x();
    tf_msg.transform.rotation.y = quaternion.y();
    tf_msg.transform.rotation.z = quaternion.z();
    tf_msg.transform.rotation.w = quaternion.w();
    tf_broadcaster_->sendTransform(tf_msg);
}

//* Publish map points as PointCloud2
void MonocularMode::publishMapPoints(
    const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
    const rclcpp::Time& stamp,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{
    if (mapPoints.empty()) return;

    // Count valid (non-null, non-bad) points
    std::vector<Eigen::Vector3f> validPoints;
    validPoints.reserve(mapPoints.size());
    for (auto* mp : mapPoints)
    {
        if (mp && !mp->isBad())
        {
            validPoints.push_back(mp->GetWorldPos());
        }
    }

    if (validPoints.empty()) return;

    // Build PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = "map";
    cloud_msg.height = 1;
    cloud_msg.width = validPoints.size();
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;

    // Define fields: x, y, z as float32
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(validPoints.size());

    // Fill the data
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& pt : validPoints)
    {
        *iter_x = pt.x();
        *iter_y = pt.y();
        *iter_z = pt.z();
        ++iter_x; ++iter_y; ++iter_z;
    }

    publisher->publish(cloud_msg);
}

//* Publish keyframe trajectory as Path
void MonocularMode::publishTrajectory(const rclcpp::Time& stamp)
{
    std::vector<Sophus::SE3f> keyframePoses = pAgent->GetAllKeyframePoses();
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = "map";

    for (const auto& kfPose : keyframePoses)
    {
        // kfPose is Twc (world-to-camera) from GetAllKeyframePoses
        Eigen::Vector3f t = kfPose.translation();
        Eigen::Quaternionf q = kfPose.unit_quaternion();

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = "map";
        ps.pose.position.x = t.x();
        ps.pose.position.y = t.y();
        ps.pose.position.z = t.z();
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        path_msg.poses.push_back(ps);
    }

    trajectory_publisher_->publish(path_msg);
}

//* Publish tracking state as Int32
void MonocularMode::publishTrackingState()
{
    auto state_msg = std_msgs::msg::Int32();
    state_msg.data = pAgent->GetTrackingState();
    tracking_state_publisher_->publish(state_msg);
}

