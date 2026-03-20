#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "dynamixel_control/SetPosition.h"
#include "dynamixel_control/GetPosition.h"
#include <algorithm>
#include <set>
#include <vector>

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default setting
#define BAUDRATE 57600
#define DEVICE_NAME "/dev/ttyUSB0"

using namespace dynamixel;

PortHandler *portHandler;
PacketHandler *packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result;
int32_t present_position = 0;

std::vector<uint8_t> controlled_ids;
std::set<uint8_t> multi_turn_ids;

bool containsId(const std::vector<uint8_t> &ids, uint8_t id)
{
    return std::find(ids.begin(), ids.end(), id) != ids.end();
}

std::vector<uint8_t> intListToUint8(const std::vector<int> &values)
{
    std::vector<uint8_t> out;
    out.reserve(values.size());
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (values[i] < 0 || values[i] > 252)
        {
            ROS_WARN("Ignore invalid motor id %d", values[i]);
            continue;
        }
        out.push_back(static_cast<uint8_t>(values[i]));
    }
    return out;
}

void setupDxl(uint8_t id)
{
    const uint8_t mode = multi_turn_ids.count(id) ? 4 : 3;

    // Make sure torque is off before changing operating mode.
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        ROS_ERROR("[ID %d] Failed disable torque before mode set: %s", id, packetHandler->getTxRxResult(dxl_comm_result));

    // set to position control mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        ROS_ERROR("[ID %d] Failed set mode: %s", id, packetHandler->getTxRxResult(dxl_comm_result));
    else
        ROS_INFO("[ID %d] Operating mode set to %s", id, mode == 4 ? "extended-position(4)" : "position(3)");

    // enable torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        ROS_ERROR("[ID %d] Failed enable torque: %s", id, packetHandler->getTxRxResult(dxl_comm_result));
    else
        ROS_INFO("[ID %d] Torque enabled", id);
}

void setPositionCallback(const dynamixel_control::SetPosition::ConstPtr &msg)
{
    uint8_t id = msg->id;
    if (!containsId(controlled_ids, id))
    {
        // ROS_WARN("ID %d is not in the controlled list", id);
        return;
    }
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, msg->position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        ROS_ERROR("Failed write pos: %s", packetHandler->getTxRxResult(dxl_comm_result));
}

bool getPositionService(dynamixel_control::GetPosition::Request &req,
                        dynamixel_control::GetPosition::Response &res)
{
    uint8_t id = req.id;
    if (!containsId(controlled_ids, id))
    {
        ROS_WARN("ID %d is not in the controlled list", id);
        res.position = -1;
        return true;
    }
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION,
                                                   (uint32_t *)&present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed read pos: %s", packetHandler->getTxRxResult(dxl_comm_result));
    }
    res.position = present_position;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_dxl_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string device_name = DEVICE_NAME;
    int baudrate = BAUDRATE;
    pnh.param<std::string>("device_name", device_name, DEVICE_NAME);
    pnh.param("baudrate", baudrate, BAUDRATE);

    std::vector<int> default_motor_ids;
    default_motor_ids.push_back(11);
    default_motor_ids.push_back(1);
    default_motor_ids.push_back(2);
    default_motor_ids.push_back(12);
    default_motor_ids.push_back(3);
    default_motor_ids.push_back(4);
    default_motor_ids.push_back(13);
    default_motor_ids.push_back(5);
    default_motor_ids.push_back(6);
    default_motor_ids.push_back(14);
    default_motor_ids.push_back(7);
    default_motor_ids.push_back(8);

    std::vector<int> default_multi_turn_ids;
    for (int i = 1; i <= 8; ++i)
        default_multi_turn_ids.push_back(i);

    std::vector<int> motor_ids_param;
    std::vector<int> multi_turn_ids_param;
    pnh.param("motor_ids", motor_ids_param, default_motor_ids);
    pnh.param("multi_turn_ids", multi_turn_ids_param, default_multi_turn_ids);

    controlled_ids = intListToUint8(motor_ids_param);
    std::vector<uint8_t> multi_turn_list = intListToUint8(multi_turn_ids_param);
    multi_turn_ids = std::set<uint8_t>(multi_turn_list.begin(), multi_turn_list.end());

    if (controlled_ids.empty())
    {
        ROS_ERROR("No valid motor IDs configured.");
        return 1;
    }

    ROS_INFO("Configured %zu motors", controlled_ids.size());

    portHandler = PortHandler::getPortHandler(device_name.c_str());
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort())
    {
        ROS_ERROR("Failed to open port %s", device_name.c_str());
        return 1;
    }
    if (!portHandler->setBaudRate(baudrate))
    {
        ROS_ERROR("Failed to set baudrate %d", baudrate);
        return 1;
    }

    // initialize each motor
    for (auto id : controlled_ids)
        setupDxl(id);

    ros::Subscriber sub = nh.subscribe("set_position", 10, setPositionCallback);
    ros::ServiceServer srv = nh.advertiseService("get_position", getPositionService);

    ros::spin();

    // disable torque on exit
    for (auto id : controlled_ids)
    {
        packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    }
    portHandler->closePort();

    return 0;
}
