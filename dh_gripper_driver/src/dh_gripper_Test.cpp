#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "dh_gripper_msgs/msg/gripper_ctrl.hpp"
#include "dh_gripper_msgs/msg/gripper_state.hpp"
#include "dh_gripper_msgs/msg/gripper_rot_ctrl.hpp"
#include "dh_gripper_msgs/msg/gripper_rot_state.hpp"


dh_gripper_msgs::msg::GripperState _g_state;
dh_gripper_msgs::msg::GripperRotState _r_state;

void _update_gripper_state(const dh_gripper_msgs::msg::GripperState::SharedPtr msg)
{
    _g_state = *msg;
}

void _update_gripper_rot_state(const dh_gripper_msgs::msg::GripperRotState::SharedPtr msg)
{
    _r_state = *msg;
}

void AG95_Test(rclcpp::Node::SharedPtr node, rclcpp::Publisher<dh_gripper_msgs::msg::GripperCtrl>::SharedPtr gripper_ctrl_pub)
{
    dh_gripper_msgs::msg::GripperCtrl msg_g_ctrl;
    msg_g_ctrl.initialize = true;
    msg_g_ctrl.position = 100;
    msg_g_ctrl.force = 100;
    msg_g_ctrl.speed = 100;
    gripper_ctrl_pub->publish(msg_g_ctrl);

    while(!_g_state.is_initialized)
    {
        rclcpp::spin_some(node);
    }

    int test_state = 0;
    while (rclcpp::ok())
    {
        switch(test_state)
        {
            case 0:
                msg_g_ctrl.initialize = false;
                msg_g_ctrl.position = 100;
                msg_g_ctrl.force = 100;
                msg_g_ctrl.speed = 100;
                gripper_ctrl_pub->publish(msg_g_ctrl);
                test_state = 1;
                break;
            case 1:
                if(_g_state.grip_state == 0)
                    test_state = 2;
                break;
            case 2:
                if(_g_state.grip_state != 0)
                    test_state = 3;
                break;
            case 3:
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                test_state = 4;
                break;
            case 4:
                msg_g_ctrl.initialize = false;
                msg_g_ctrl.position = 0;
                msg_g_ctrl.force = 100;
                msg_g_ctrl.speed = 100;
                gripper_ctrl_pub->publish(msg_g_ctrl);
                test_state = 5;
                break;
            case 5:
                if(_g_state.grip_state == 0)
                    test_state = 6;
                break;
            case 6:
                if(_g_state.grip_state != 0)
                    test_state = 7;
                break;
            case 7:
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                test_state = 0;
                break;
        }
        rclcpp::spin_some(node);
    }
}

void DH3_Test(rclcpp::Node::SharedPtr node, rclcpp::Publisher<dh_gripper_msgs::msg::GripperCtrl>::SharedPtr gripper_ctrl_pub, rclcpp::Publisher<dh_gripper_msgs::msg::GripperCtrl>::SharedPtr rot_ctrl_pub)
{
    dh_gripper_msgs::msg::GripperCtrl msg_g_ctrl;
    msg_g_ctrl.initialize = true;
    msg_g_ctrl.position = 1000;
    msg_g_ctrl.force = 100;
    msg_g_ctrl.speed = 100;
    gripper_ctrl_pub->publish(msg_g_ctrl);

    while(!_g_state.is_initialized)
    {
        rclcpp::spin_some(node);
    }

    int test_state = 0;
    while (rclcpp::ok())
    {
        switch(test_state)
        {
            case 0:
                msg_g_ctrl.initialize = false;
                msg_g_ctrl.position = 1000;
                msg_g_ctrl.force = 100;
                msg_g_ctrl.speed = 100;
                gripper_ctrl_pub->publish(msg_g_ctrl);
                test_state = 1;
                break;
            case 1:
                if(_g_state.grip_state == 0)
                    test_state = 2;
                break;
            case 2:
                if(_g_state.grip_state != 0)
                    test_state = 3;
                break;
            case 3:
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                test_state = 4;
                break;
            case 4:
                msg_g_ctrl.initialize = false;
                msg_g_ctrl.position = 0;
                msg_g_ctrl.force = 100;
                msg_g_ctrl.speed = 100;
                gripper_ctrl_pub->publish(msg_g_ctrl);
                test_state = 5;
                break;
            case 5:
                if(_g_state.grip_state == 0)
                    test_state = 6;
                break;
            case 6:
                if(_g_state.grip_state != 0)
                    test_state = 7;
                break;
            case 7:
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                test_state = 0;
                break;
        }
        rclcpp::spin_some(node);
    }
}

void ModbusGripper_test(rclcpp::Node::SharedPtr node, rclcpp::Publisher<dh_gripper_msgs::msg::GripperCtrl>::SharedPtr gripper_ctrl_pub)
{
    dh_gripper_msgs::msg::GripperCtrl msg_g_ctrl;
    msg_g_ctrl.initialize = true;
    msg_g_ctrl.position = 1000;
    msg_g_ctrl.force = 100;
    msg_g_ctrl.speed = 100;
    gripper_ctrl_pub->publish(msg_g_ctrl);

    while(!_g_state.is_initialized)
    {
        rclcpp::spin_some(node);
    }

    int test_state = 0;
    while (rclcpp::ok())
    {
        switch(test_state)
        {
            case 0:
                msg_g_ctrl.initialize = false;
                msg_g_ctrl.position = 1000;
                msg_g_ctrl.force = 100;
                msg_g_ctrl.speed = 100;
                gripper_ctrl_pub->publish(msg_g_ctrl);
                test_state = 1;
                break;
            case 1:
                if(_g_state.grip_state == 0)
                    test_state = 2;
                break;
            case 2:
                if(_g_state.grip_state != 0)
                    test_state = 3;
                break;
            case 3:
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                test_state = 4;
                break;
            case 4:
                msg_g_ctrl.initialize = false;
                msg_g_ctrl.position = 0;
                msg_g_ctrl.force = 100;
                msg_g_ctrl.speed = 100;
                gripper_ctrl_pub->publish(msg_g_ctrl);
                test_state = 5;
                break;
            case 5:
                if(_g_state.grip_state == 0)
                    test_state = 6;
                break;
            case 6:
                if(_g_state.grip_state != 0)
                    test_state = 7;
                break;
            case 7:
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                test_state = 0;
                break;
        }
        rclcpp::spin_some(node);
    }
}

void RGI_test(rclcpp::Node::SharedPtr node, rclcpp::Publisher<dh_gripper_msgs::msg::GripperCtrl>::SharedPtr gripper_ctrl_pub, rclcpp::Publisher<dh_gripper_msgs::msg::GripperCtrl>::SharedPtr rot_ctrl_pub)
{
    dh_gripper_msgs::msg::GripperCtrl msg_g_ctrl;
    msg_g_ctrl.initialize = true;
    msg_g_ctrl.position = 1000;
    msg_g_ctrl.force = 100;
    msg_g_ctrl.speed = 100;
    gripper_ctrl_pub->publish(msg_g_ctrl);

    while(!_g_state.is_initialized)
    {
        rclcpp::spin_some(node);
    }

    int test_state = 0;
    while (rclcpp::ok())
    {
        switch(test_state)
        {
            case 0:
                msg_g_ctrl.initialize = false;
                msg_g_ctrl.position = 1000;
                msg_g_ctrl.force = 100;
                msg_g_ctrl.speed = 100;
                gripper_ctrl_pub->publish(msg_g_ctrl);
                test_state = 1;
                break;
            case 1:
                if(_g_state.grip_state == 0)
                    test_state = 2;
                break;
            case 2:
                if(_g_state.grip_state != 0)
                    test_state = 3;
                break;
            case 3:
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                test_state = 4;
                break;
            case 4:
                msg_g_ctrl.initialize = false;
                msg_g_ctrl.position = 0;
                msg_g_ctrl.force = 100;
                msg_g_ctrl.speed = 100;
                gripper_ctrl_pub->publish(msg_g_ctrl);
                test_state = 5;
                break;
            case 5:
                if(_g_state.grip_state == 0)
                    test_state = 6;
                break;
            case 6:
                if(_g_state.grip_state != 0)
                    test_state = 7;
                break;
            case 7:
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                test_state = 0;
                break;
        }
        rclcpp::spin_some(node);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dh_gripper_tester");
    node->declare_parameter<std::string>("Gripper_Model", "AG95_MB");
    std::string _gripper_model;
    node->get_parameter("Gripper_Model", _gripper_model);

    RCLCPP_INFO(node->get_logger(), "Gripper_model : %s", _gripper_model.c_str());
    auto _sub_grip_state = node->create_subscription<dh_gripper_msgs::msg::GripperState>(
        "/gripper/states", 50, _update_gripper_state);
    auto _gripper_ctrl_pub = node->create_publisher<dh_gripper_msgs::msg::GripperCtrl>("/gripper/ctrl", 50);

    if(_gripper_model.find("AG95_CAN")!=_gripper_model.npos)
    {
        AG95_Test(node, _gripper_ctrl_pub);
    }
    else if(_gripper_model.find("DH3")!=_gripper_model.npos)
    {
        auto _sub_rot_state = node->create_subscription<dh_gripper_msgs::msg::GripperRotState>(
            "/gripper/rot_states", 50, _update_gripper_rot_state);
        auto _rot_ctrl_pub = node->create_publisher<dh_gripper_msgs::msg::GripperCtrl>("/gripper/rot_ctrl", 50);
        DH3_Test(node, _gripper_ctrl_pub, _rot_ctrl_pub);
    }
    else if(_gripper_model.find("AG95_MB")!=_gripper_model.npos
                ||_gripper_model.find("PGE")!=_gripper_model.npos
                ||_gripper_model.find("PGC")!=_gripper_model.npos
                ||_gripper_model.find("CGC")!=_gripper_model.npos)
    {
        ModbusGripper_test(node, _gripper_ctrl_pub);
    }
    else if(_gripper_model.find("RGI")!=_gripper_model.npos)
    {
        auto _sub_rot_state = node->create_subscription<dh_gripper_msgs::msg::GripperRotState>(
            "/gripper/rot_states", 50, _update_gripper_rot_state);
        auto _rot_ctrl_pub = node->create_publisher<dh_gripper_msgs::msg::GripperCtrl>("/gripper/rot_ctrl", 50);
        RGI_test(node, _gripper_ctrl_pub, _rot_ctrl_pub);
    }
    else
    {
        return -1;
    }
    rclcpp::shutdown();
    return 0;
}