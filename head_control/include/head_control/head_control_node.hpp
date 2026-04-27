#ifndef HEAD_CONTROL_NODE_HPP
#define HEAD_CONTROL_NODE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

class HeadControlNode : public rclcpp::Node
{
public:
    HeadControlNode();
    ~HeadControlNode();

private:
    void init_dynamixel();
    void shutdown_dynamixel();
    bool write_goal_positions(int32_t pitch_pos, int32_t yaw_pos);
    bool read_present_positions(int32_t & pitch_pos, int32_t & yaw_pos);

    static int32_t rad_to_pos(double rad, int32_t center);
    static double  pos_to_rad(int32_t pos, int32_t center);
    static int32_t clamp_pos(int32_t pos, int32_t lo, int32_t hi);

    void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish_state();

    dynamixel::PortHandler *    port_handler_   {nullptr};
    dynamixel::PacketHandler *  packet_handler_ {nullptr};
    dynamixel::GroupSyncWrite * sync_write_     {nullptr};
    dynamixel::GroupSyncRead *  sync_read_      {nullptr};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    state_pub_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    int32_t goal_pitch_pos_ {2048};
    int32_t goal_yaw_pos_   {2048};
    bool    torque_enabled_ {false};
};

#endif
