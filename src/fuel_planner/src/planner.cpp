/*****************************************************************************************
 * 本代码采用的mavros的速度控制进行跟踪
 ******************************************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "quadrotor_msgs/PositionCommand.h"

// 定义速度控制掩码
#define VELOCITY2D_CONTROL 0b101111000111

class FuelPlanner {
public:
    FuelPlanner(ros::NodeHandle& nh);
    ~FuelPlanner() = default;

    // 运行主循环
    void run();

private:
    // ROS节点句柄
    ros::NodeHandle nh_;

    // 订阅器
    ros::Subscriber state_sub_;
    ros::Subscriber rc_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber position_sub_;

    // 发布器
    ros::Publisher local_pos_pub_;
    ros::Publisher pub_marker_;

    // 服务客户端
    ros::ServiceClient arming_client_;
    ros::ServiceClient command_client_;
    ros::ServiceClient set_mode_client_;

    // 状态变量
    mavros_msgs::State current_state_;
    mavros_msgs::RCIn rc_;
    nav_msgs::Odometry position_msg_;
    geometry_msgs::PoseStamped target_pos_;
    quadrotor_msgs::PositionCommand ego_;

    // 控制相关变量
    mavros_msgs::PositionTarget current_goal_;
    unsigned short velocity_mask_;

    // 位置与航向相关变量
    float position_x_begin_, position_y_begin_, position_z_begin_, yaw_begin_;
    float position_x_, position_y_, position_z_, current_yaw_;
    float targetpos_x_, targetpos_y_;
    float ego_pos_x_, ego_pos_y_, ego_pos_z_;
    float ego_vel_x_, ego_vel_y_, ego_vel_z_;
    float ego_a_x_, ego_a_y_, ego_a_z_;
    float ego_yaw_, ego_yaw_rate_;

    // 标志位
    bool get_first_pos_;
    bool receive_;
    int rc_value_;
    int flag_, flag1_;

    // 常量
    const float PI = 3.14159265f;

    // 回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);
    void positionCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void twistCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);

    // 初始化函数
    void initializeSubscribers();
    void initializePublishers();
    void initializeServiceClients();
    void waitForConnection();
    void sendInitialSetpoints(int count, double rate_hz);

    // 控制逻辑函数
    void holdPosition(float target_x, float target_y, float target_z);
    void trackTrajectory();
    void publishControlCommand();
};

// 构造函数
FuelPlanner::FuelPlanner(ros::NodeHandle& nh) 
    : nh_(nh),
      velocity_mask_(VELOCITY2D_CONTROL),
      get_first_pos_(false),
      receive_(false),
      rc_value_(0),
      flag_(0),
      flag1_(0),
      position_x_begin_(0), position_y_begin_(0), position_z_begin_(0), yaw_begin_(0),
      position_x_(0), position_y_(0), position_z_(0), current_yaw_(0),
      targetpos_x_(0), targetpos_y_(0),
      ego_pos_x_(0), ego_pos_y_(0), ego_pos_z_(0),
      ego_vel_x_(0), ego_vel_y_(0), ego_vel_z_(0),
      ego_a_x_(0), ego_a_y_(0), ego_a_z_(0),
      ego_yaw_(0), ego_yaw_rate_(0)
{
    initializeSubscribers();
    initializePublishers();
    initializeServiceClients();
    
    ROS_INFO("FuelPlanner 初始化完成");
}

// 初始化订阅器
void FuelPlanner::initializeSubscribers() {
    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, &FuelPlanner::stateCallback, this);
    
    rc_sub_ = nh_.subscribe<mavros_msgs::RCIn>(
        "/mavros/rc/in", 10, &FuelPlanner::rcCallback, this);
    
    twist_sub_ = nh_.subscribe<quadrotor_msgs::PositionCommand>(
        "/planning/pos_cmd", 10, &FuelPlanner::twistCallback, this);
    
    target_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "move_base_simple/goal", 10, &FuelPlanner::targetCallback, this);
    
    position_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 10, &FuelPlanner::positionCallback, this);
}

// 初始化发布器
void FuelPlanner::initializePublishers() {
    local_pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 1);
    
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>(
        "/track_drone_point", 5);
}

// 初始化服务客户端
void FuelPlanner::initializeServiceClients() {
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    command_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

// 状态回调
void FuelPlanner::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

// RC回调
void FuelPlanner::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
    rc_ = *msg;
    rc_value_ = rc_.channels[4];
}

// 位置回调
void FuelPlanner::positionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    position_msg_ = *msg;
    
    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if (!get_first_pos_) {
        position_x_begin_ = position_msg_.pose.pose.position.x;
        position_y_begin_ = position_msg_.pose.pose.position.y;
        position_z_begin_ = position_msg_.pose.pose.position.z;
        yaw_begin_ = yaw;
        get_first_pos_ = true;
        ROS_INFO("初始位置已记录: [%.2f, %.2f, %.2f], 航向: %.2f", 
                 position_x_begin_, position_y_begin_, position_z_begin_, yaw_begin_);
    }
    
    position_x_ = position_msg_.pose.pose.position.x - position_x_begin_;
    position_y_ = position_msg_.pose.pose.position.y - position_y_begin_;
    position_z_ = position_msg_.pose.pose.position.z - position_z_begin_;
    current_yaw_ = yaw;
}

// 目标点回调
void FuelPlanner::targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    target_pos_ = *msg;
    targetpos_x_ = target_pos_.pose.position.x;
    targetpos_y_ = target_pos_.pose.position.y;
}

// EGO规划器回调
void FuelPlanner::twistCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    receive_ = true;
    ego_ = *msg;
    ego_pos_x_ = ego_.position.x;
    ego_pos_y_ = ego_.position.y;
    ego_pos_z_ = ego_.position.z;
    ego_vel_x_ = ego_.velocity.x;
    ego_vel_y_ = ego_.velocity.y;
    ego_vel_z_ = ego_.velocity.z;
    ego_a_x_ = ego_.acceleration.x;
    ego_a_y_ = ego_.acceleration.y;
    ego_a_z_ = ego_.acceleration.z;
    ego_yaw_ = ego_.yaw + yaw_begin_;
    ego_yaw_rate_ = ego_.yaw_dot;
}

// 等待飞控连接
void FuelPlanner::waitForConnection() {
    ros::Rate rate(10.0);
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("已连接到飞控");
}

// 发送初始设定点
void FuelPlanner::sendInitialSetpoints(int count, double rate_hz) {
    ros::Rate rate(rate_hz);
    for (int i = count; ros::ok() && i > 0; --i) {
        current_goal_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        local_pos_pub_.publish(current_goal_);
        ros::spinOnce();
        rate.sleep();
    }
}

// 保持位置（悬停控制）
void FuelPlanner::holdPosition(float target_x, float target_y, float target_z) {
    current_goal_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_goal_.header.stamp = ros::Time::now();
    current_goal_.type_mask = velocity_mask_;
    current_goal_.velocity.x = (target_x - position_x_) * 1.0f;
    current_goal_.velocity.y = (target_y - position_y_) * 1.0f;
    current_goal_.velocity.z = (target_z - position_z_) * 1.0f;
    current_goal_.yaw = current_yaw_;
    ROS_INFO("等待中... 当前位置: [%.2f, %.2f, %.2f]", position_x_, position_y_, position_z_);
}

// 轨迹跟踪
void FuelPlanner::trackTrajectory() {
    float yaw_error = ego_yaw_ - current_yaw_;
    
    current_goal_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_goal_.header.stamp = ros::Time::now();
    current_goal_.type_mask = velocity_mask_;
    current_goal_.velocity.x = (ego_pos_x_ - position_x_) * 2.0f;
    current_goal_.velocity.y = (ego_pos_y_ - position_y_) * 2.0f;
    current_goal_.velocity.z = (ego_pos_z_ - position_z_) * 2.0f;
    current_goal_.yaw = ego_yaw_;
    
    float velocity_magnitude = sqrt(pow(current_goal_.velocity.x, 2) + 
                                    pow(current_goal_.velocity.y, 2));
    ROS_INFO("planner规划速度: vel_x = %.2f", velocity_magnitude);
}

// 发布控制指令
void FuelPlanner::publishControlCommand() {
    local_pos_pub_.publish(current_goal_);
}

// 主运行循环
void FuelPlanner::run() {
    waitForConnection();
    
    // 发送初始设定点
    sendInitialSetpoints(100, 50.0);
    
    ros::Rate rate(50.0);
    
    while (ros::ok()) {
        if (!receive_) {
            // 未接收到轨迹，保持在 (0, 0, 1) 悬停
            holdPosition(0.0f, 0.0f, 1.0f);
        } else {
            // 接收到轨迹，进行跟踪
            trackTrajectory();
        }
        
        publishControlCommand();
        
        ros::spinOnce();
        rate.sleep();
    }
}

// 主函数
int main(int argc, char **argv) {
    ros::init(argc, argv, "fuel_planner");
    setlocale(LC_ALL, "");
    
    ros::NodeHandle nh;
    ROS_INFO("启动 FuelPlanner 节点");
    
    try {
        FuelPlanner planner(nh);
        planner.run();
    } catch (const std::exception& e) {
        ROS_ERROR("运行异常: %s", e.what());
        return 1;
    }
    
    return 0;
}
