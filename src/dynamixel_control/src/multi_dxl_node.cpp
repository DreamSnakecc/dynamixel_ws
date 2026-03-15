#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "dynamixel_control/SetPosition.h"
#include "dynamixel_control/GetPosition.h"

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

void setupDxl(uint8_t id)
{
    // set to position control mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        ROS_ERROR("[ID %d] Failed set mode: %s", id, packetHandler->getTxRxResult(dxl_comm_result));

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
    if (std::find(controlled_ids.begin(), controlled_ids.end(), id) == controlled_ids.end())
    {
        ROS_WARN("ID %d is not in the controlled list", id);
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
    if (std::find(controlled_ids.begin(), controlled_ids.end(), id) == controlled_ids.end())
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

    // build list of ids 1-8 and 11-14
    for (uint8_t i = 1; i <= 8; ++i)
        controlled_ids.push_back(i);
    for (uint8_t i = 11; i <= 14; ++i)
        controlled_ids.push_back(i);

    portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort())
    {
        ROS_ERROR("Failed to open port %s", DEVICE_NAME);
        return 1;
    }
    if (!portHandler->setBaudRate(BAUDRATE))
    {
        ROS_ERROR("Failed to set baudrate");
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
