#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <my_interfaces/msg/msg_info_led_cmd.hpp>

#include "serial_ModbusRTU_slavery/modbus_master.hpp"

namespace serial_ModbusRTU_slavery
{

/// serial_ModbusRTU_slavery_node
///  - 配置 /dev/ttyTHS1 115200 8N1（-crtscts），基于 modbus 协议实现 RTU slavery 管理
///  - 订阅 /led_state/torso（my_interfaces::msg::MsgInfoLedCmd），解析 runmode/color 组成数据帧
///  - 可配置从站数量与 ID；队列模式 + cycle task 集中收发；超时重试，重试达上限后默认掉站
class serial_ModbusRTU_slavery_node : public rclcpp::Node
{
public:
    serial_ModbusRTU_slavery_node();
    ~serial_ModbusRTU_slavery_node() override;

    bool init();  // 读取参数、初始化 Modbus 主站、创建订阅
    void start(); // 启动 cycle task
    void stop();  // 停止 cycle task

private:
    void led_state_callback(const my_interfaces::msg::MsgInfoLedCmd::SharedPtr msg);

    ModbusMasterConfig cfg_;
    std::shared_ptr<ModbusMaster> master_;
    rclcpp::Subscription<my_interfaces::msg::MsgInfoLedCmd>::SharedPtr led_state_sub_;
};

} // namespace serial_ModbusRTU_slavery