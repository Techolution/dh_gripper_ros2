#include <iostream>
#include <unistd.h>
#include "dh_gripper_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "dh_gripper_msgs/msg/gripper_ctrl.hpp"
#include "dh_gripper_msgs/msg/gripper_state.hpp"
#include "dh_gripper_msgs/msg/gripper_rot_ctrl.hpp"
#include "dh_gripper_msgs/msg/gripper_rot_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

std::string _gripper_ID;
std::string _gripper_model;
std::string _gripper_connect_port;
std::string _gripper_Baudrate;
DH_Gripper *_gripper;

void update_gripper_control(const dh_gripper_msgs::msg::GripperCtrl::SharedPtr msg)
{
    if(msg->initialize)
    {
      _gripper->Initialization();  
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("dh_gripper_driver"), "speed:[%f],force: [%f], position: [%f]", msg->speed,msg->force, msg->position);
        _gripper->SetTargetSpeed((int)msg->speed);
        _gripper->SetTargetForce((int)msg->force);
        _gripper->SetTargetPosition((int)msg->position);
    }
}

void update_rotation_control(const dh_gripper_msgs::msg::GripperRotCtrl::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("dh_gripper_driver"), "r_speed:[%f],r_force: [%f], r_angle: [%f]", msg->speed,msg->force, msg->angle);
    if(_gripper_model.find("RGI")!= _gripper_model.npos)
    {
        dynamic_cast<DH_RGI *>(_gripper)->SetTargetRotationTorque((int)msg->force); 
        dynamic_cast<DH_RGI *>(_gripper)->SetTargetRotationSpeed((int)msg->speed); 
        dynamic_cast<DH_RGI *>(_gripper)->SetTargetRotation((int)msg->angle); 
    }
    else if(_gripper_model.find("DH3_CAN")!= _gripper_model.npos)
    {
        dynamic_cast<DH_DH3_CAN *>(_gripper)->SetTargetRotation((int)msg->angle); 
    }
}

void update_gripper_state(dh_gripper_msgs::msg::GripperState & msg)
{
    static long seq = 0;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "";
    msg.header.set__stamp(rclcpp::Clock().now());
    msg.header.set__frame_id("");
    // msg.header.seq = seq; // Not used in ROS2 std_msgs/Header
    int tmp_state[5] = {0};
    _gripper->GetRunStates(tmp_state);
    msg.is_initialized = (tmp_state[0] == 1);
    msg.grip_state      = tmp_state[1];
    msg.position        = tmp_state[2];
    msg.target_position = tmp_state[3];
    msg.target_force    = tmp_state[4];
    seq++;
}

void update_gripper_joint_state(sensor_msgs::msg::JointState & msg)
{
    static long seq = 0;
    msg.header.frame_id = "";
    msg.header.stamp = rclcpp::Clock().now();
    msg.name.resize(1);
    msg.position.resize(1);

    int tmp_pos = 0;
    _gripper->GetCurrentPosition(tmp_pos);

    msg.position[0] = (1000-tmp_pos)/1000.0 * 0.637;
    msg.name[0] = "gripper_finger1_joint"; 
    seq++;
}

void update_rotation_state(dh_gripper_msgs::msg::GripperRotState & msg)
{
    static long seq = 0;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "";
    int tmp_state[9] = {0};
    _gripper->GetRunStates(tmp_state); 
    msg.rot_state       = tmp_state[5];
    msg.angle           = tmp_state[6];
    msg.target_angle    = tmp_state[7];
    msg.target_force    = tmp_state[8];
    seq++;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dh_gripper_driver");

    node->declare_parameter<int>("Gripper_ID", 1);
    node->declare_parameter<std::string>("Gripper_Model", "AG95_MB");
    node->declare_parameter<std::string>("Connect_port", "/dev/ttyUSB0");
    node->declare_parameter<int>("BaudRate", 115200);

    int gripper_id_param = 1;
    int baudrate_param = 115200;
    node->get_parameter("Gripper_ID", gripper_id_param);
    _gripper_ID = std::to_string(gripper_id_param);
    node->get_parameter("Gripper_Model", _gripper_model);
    node->get_parameter("Connect_port", _gripper_connect_port);
    node->get_parameter("BaudRate", baudrate_param);
    _gripper_Baudrate = std::to_string(baudrate_param);

    RCLCPP_INFO(node->get_logger(), "Gripper_ID : %s", _gripper_ID.c_str());
    RCLCPP_INFO(node->get_logger(), "Gripper_model : %s", _gripper_model.c_str());
    RCLCPP_INFO(node->get_logger(), "Connect_port: %s", _gripper_connect_port.c_str());
    RCLCPP_INFO(node->get_logger(), "BaudRate : %s (In TCP/IP Mode , BaudRate is unuse)", _gripper_Baudrate.c_str());

    DH_Gripper_Factory* _gripper_Factory = new DH_Gripper_Factory();
    _gripper_Factory->Set_Parameter(atoi(_gripper_ID.c_str()), _gripper_connect_port, atoi(_gripper_Baudrate.c_str()));

    _gripper = _gripper_Factory->CreateGripper(_gripper_model);
    if(_gripper == NULL)
    {
        RCLCPP_ERROR(node->get_logger(), "No this Model :%s", _gripper_model.c_str());
        return -1;
    }   

    if(_gripper->open()<0)
    {
        RCLCPP_ERROR(node->get_logger(), "Unable to open commport to %s", _gripper_connect_port.c_str());
        return -1;
    }

    int initstate = 0;
    _gripper->GetInitState(initstate);
    if(initstate != DH_Gripper::S_INIT_FINISHED)
    {
        _gripper->Initialization();
        std::cout<< " Send grip init " << std::endl;
        initstate = 0;
        std::cout<< " wait grip initialized " << std::endl;
        while(initstate != DH_Gripper::S_INIT_FINISHED )
            _gripper->GetInitState(initstate); 
        std::cout<< "GetInitState "<< initstate << std::endl;
    }

    auto _sub_grip = node->create_subscription<dh_gripper_msgs::msg::GripperCtrl>(
        "/gripper/ctrl", 50, update_gripper_control);
    auto _gripper_state_pub = node->create_publisher<dh_gripper_msgs::msg::GripperState>("/gripper/states", 50);

    rclcpp::Subscription<dh_gripper_msgs::msg::GripperRotCtrl>::SharedPtr _sub_rot;
    rclcpp::Publisher<dh_gripper_msgs::msg::GripperRotState>::SharedPtr _rot_state_pub;

    if(_gripper->GetGripperAxiNumber()==2)
    {
        _sub_rot = node->create_subscription<dh_gripper_msgs::msg::GripperRotCtrl>(
            "/gripper/rot_ctrl", 50, update_rotation_control);
        _rot_state_pub = node->create_publisher<dh_gripper_msgs::msg::GripperRotState>("/gripper/rot_states", 50);
    }

    auto _gripper_joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/gripper/joint_states", 50); 

    rclcpp::Rate loop_rate(50);
    while (rclcpp::ok())
    {
        dh_gripper_msgs::msg::GripperState msg_g_state;
        update_gripper_state(msg_g_state);
        _gripper_state_pub->publish(msg_g_state);

        sensor_msgs::msg::JointState msg_g_joint_state;
        update_gripper_joint_state(msg_g_joint_state);
        _gripper_joint_state_pub->publish(msg_g_joint_state);

        if(_gripper->GetGripperAxiNumber()==2 && _rot_state_pub)
        {
            dh_gripper_msgs::msg::GripperRotState msg_r_state;
            update_rotation_state(msg_r_state);
            _rot_state_pub->publish(msg_r_state);
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    _gripper->close();
    rclcpp::shutdown();
    return 0;
}