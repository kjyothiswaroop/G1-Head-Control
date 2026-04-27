#include "head_control/head_control_node.hpp"
#include <cmath>
#include <stdexcept>

// Control table addresses (X-series, Protocol 2.0)
static constexpr uint16_t ADDR_OPERATING_MODE   = 11;
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
static constexpr uint16_t ADDR_GOAL_POSITION    = 116;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
static constexpr uint16_t LEN_GOAL_POSITION     = 4;
static constexpr uint16_t LEN_PRESENT_POSITION  = 4;

// Operating mode values
static constexpr uint8_t POSITION_CONTROL_MODE = 4;
static constexpr uint8_t TORQUE_ENABLE         = 1;
static constexpr uint8_t TORQUE_DISABLE        = 0;

//Hardware config
static constexpr double   PROTOCOL_VERSION  = 2.0;
static constexpr int      BAUDRATE          = 57600;
static constexpr char     DEVICE_NAME[]     = "/dev/ttyUSB0";
static constexpr int      PITCH_ID          = 1;
static constexpr int      YAW_ID            = 2;
static constexpr uint32_t PROFILE_VELOCITY  = 100;

//Position encoding (4096 ticks per 360°)
static constexpr double  TICKS_PER_RAD    = 4096.0 / (2.0 * M_PI);
static constexpr int32_t PITCH_CENTER_POS = 3736;  // measured neutral forward-facing position
static constexpr int32_t YAW_CENTER_POS   = 3624;  // measured neutral forward-facing position

//Pitch limits — neutral (3736) is bottom of range; head can only tilt down (increasing ticks)
static constexpr int32_t PITCH_MIN_POS = 3736;
static constexpr int32_t PITCH_MAX_POS = 4518;  // -1.2 rad max down

//Yaw limits — remeasure from neutral; provisional ±90° around new center
static constexpr int32_t YAW_MIN_POS = 2000;  // provisional — remeasure physical limit
static constexpr int32_t YAW_MAX_POS = 5000;  // provisional — remeasure physical limit

HeadControlNode::HeadControlNode() : Node("head_control_node")
{
    init_dynamixel();

    target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "head/target",10,
        std::bind(&HeadControlNode::target_callback, this, std::placeholders::_1)
    );

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("head/state", 10);

    state_timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&HeadControlNode::publish_state, this));

    goal_pitch_pos_ = PITCH_CENTER_POS;
    goal_yaw_pos_   = YAW_CENTER_POS;
    write_goal_positions(goal_pitch_pos_, goal_yaw_pos_);
    RCLCPP_INFO(get_logger(), "Head centered at startup");
}

HeadControlNode::~HeadControlNode()
{
    shutdown_dynamixel();
}

int32_t HeadControlNode::rad_to_pos(double rad, int32_t center)
{
    return static_cast<int32_t>(std::round(center + rad * TICKS_PER_RAD));
}

double HeadControlNode::pos_to_rad(int32_t pos, int32_t center)
{
    return static_cast<double>(pos - center) / TICKS_PER_RAD;
}

int32_t HeadControlNode::clamp_pos(int32_t pos, int32_t lo, int32_t hi)
{
    return std::max(lo, std::min(hi, pos));
}

void HeadControlNode::init_dynamixel()
{
    port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    //Open port for communication with u2d2
    auto dxl_comm_res = port_handler_ -> openPort();
    if(dxl_comm_res)
    {
        RCLCPP_INFO(get_logger(), "Opened port succesfully.");
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to open port");
        throw std::runtime_error("Failed to open port");
    }

    // Set baudrate
    dxl_comm_res = port_handler_ -> setBaudRate(BAUDRATE);
    if(dxl_comm_res)
    {
        RCLCPP_INFO(get_logger(), "Baud rate set succesfully.");
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to set baudrate");
        throw std::runtime_error("Failed to set baudrate");
    }

    uint8_t dxl_error = 0;

    for(int id : {PITCH_ID, YAW_ID})
    {
        auto dxl_res = packet_handler_ ->write1ByteTxRx(
            port_handler_,
            id,
            ADDR_OPERATING_MODE,
            POSITION_CONTROL_MODE,
            &dxl_error
        );

        if(dxl_res != COMM_SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to set Position control for id %d", id);
            throw std::runtime_error("Failed to set Position control");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Position control set successfully for ID %d", id);
        }

        dxl_res = packet_handler_->write4ByteTxRx(
            port_handler_,
            id,
            ADDR_PROFILE_VELOCITY,
            PROFILE_VELOCITY,
            &dxl_error
        );

        if(dxl_res != COMM_SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to set profile velocity for ID %d", id);
            throw std::runtime_error("Failed to set profile velocity");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Profile velocity set successfully for ID %d", id);
        }

        dxl_res = packet_handler_->write1ByteTxRx(
            port_handler_,
            id,
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE,
            &dxl_error
        );

        if(dxl_res != COMM_SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to enable torque for ID %d", id);
            throw std::runtime_error("Failed to enable Torque");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Torque enabled successfully for ID %d", id);
        }

    }
    torque_enabled_ = true;
    sync_read_ = new dynamixel::GroupSyncRead(port_handler_, packet_handler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    sync_read_ ->addParam(PITCH_ID);
    sync_read_ ->addParam(YAW_ID);
    sync_write_ = new dynamixel::GroupSyncWrite(port_handler_, packet_handler_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

}

bool HeadControlNode::write_goal_positions(int32_t pitch_pos, int32_t yaw_pos)
{
    sync_write_->clearParam();

    uint8_t pitch_data[4] = {
        DXL_LOBYTE(DXL_LOWORD(pitch_pos)),
        DXL_HIBYTE(DXL_LOWORD(pitch_pos)),
        DXL_LOBYTE(DXL_HIWORD(pitch_pos)),
        DXL_HIBYTE(DXL_HIWORD(pitch_pos))
    };
    uint8_t yaw_data[4] = {
        DXL_LOBYTE(DXL_LOWORD(yaw_pos)),
        DXL_HIBYTE(DXL_LOWORD(yaw_pos)),
        DXL_LOBYTE(DXL_HIWORD(yaw_pos)),
        DXL_HIBYTE(DXL_HIWORD(yaw_pos))
    };

    sync_write_->addParam(PITCH_ID, pitch_data);
    sync_write_->addParam(YAW_ID, yaw_data);

    int result = sync_write_->txPacket();
    if (result != COMM_SUCCESS) {
        RCLCPP_WARN(get_logger(), "write_goal_positions failed: %s",
            packet_handler_->getTxRxResult(result));
        return false;
    }
    return true;
}

bool HeadControlNode::read_present_positions(int32_t & pitch_pos, int32_t & yaw_pos)
{
    int result = sync_read_->txRxPacket();
    if (result != COMM_SUCCESS) {
        RCLCPP_WARN(get_logger(), "read_present_positions failed: %s",
            packet_handler_->getTxRxResult(result));
        return false;
    }

    if (!sync_read_->isAvailable(PITCH_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) ||
        !sync_read_->isAvailable(YAW_ID,   ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
        RCLCPP_WARN(get_logger(), "read_present_positions: data not available");
        return false;
    }

    pitch_pos = static_cast<int32_t>(sync_read_->getData(PITCH_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
    yaw_pos   = static_cast<int32_t>(sync_read_->getData(YAW_ID,   ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
    return true;
}

void HeadControlNode::shutdown_dynamixel()
{
    if(torque_enabled_)
    {
        uint8_t dxl_error = 0;
        for(int id : {PITCH_ID, YAW_ID})
        {
            auto dxl_res = packet_handler_->write1ByteTxRx(
            port_handler_,
            id,
            ADDR_TORQUE_ENABLE,
            TORQUE_DISABLE,
            &dxl_error);
        }
        torque_enabled_ = false;
    }
    delete sync_read_;
    delete sync_write_;

    port_handler_->closePort();
}

void HeadControlNode::target_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    bool got_pitch = false, got_yaw = false;
    double pitch_rad = 0.0, yaw_rad = 0.0;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "pitch" && i < msg->position.size()) {
            pitch_rad = msg->position[i];
            got_pitch = true;
        } else if (msg->name[i] == "yaw" && i < msg->position.size()) {
            yaw_rad = msg->position[i];
            got_yaw = true;
        }
    }

    if (!got_pitch && !got_yaw) {
        RCLCPP_WARN(get_logger(), "Received JointState with no 'pitch' or 'yaw' fields");
        return;
    }

    if (got_pitch) {
        goal_pitch_pos_ = clamp_pos(rad_to_pos(-pitch_rad, PITCH_CENTER_POS), PITCH_MIN_POS, PITCH_MAX_POS);
    }
    if (got_yaw) {
        goal_yaw_pos_ = clamp_pos(rad_to_pos(yaw_rad, YAW_CENTER_POS), YAW_MIN_POS, YAW_MAX_POS);
    }

    write_goal_positions(goal_pitch_pos_, goal_yaw_pos_);
}

void HeadControlNode::publish_state()
{
    int32_t pitch_pos, yaw_pos;
    if (!read_present_positions(pitch_pos, yaw_pos)) 
    {
        return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name     = {"pitch", "yaw"};
    msg.position = {-pos_to_rad(pitch_pos, PITCH_CENTER_POS), pos_to_rad(yaw_pos, YAW_CENTER_POS)};
    state_pub_->publish(msg);
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadControlNode>());
    rclcpp::shutdown();
    return 0;
}