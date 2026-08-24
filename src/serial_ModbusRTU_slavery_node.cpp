#include "serial_ModbusRTU_slavery/serial_ModbusRTU_slavery_node.hpp"

#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include "../include/log.hpp"

namespace serial_ModbusRTU_slavery
{

serial_ModbusRTU_slavery_node::serial_ModbusRTU_slavery_node()
    : Node("serial_ModbusRTU_slavery_node")
{
}

serial_ModbusRTU_slavery_node::~serial_ModbusRTU_slavery_node()
{
    stop();
}

bool serial_ModbusRTU_slavery_node::init()
{
    // ===================== 声明并读取 ROS 参数 =====================
    declare_parameter("serial_device", "/dev/ttyTHS1");
    declare_parameter("baudrate", 115200);
    declare_parameter("parity", "N");
    declare_parameter("data_bits", 8);
    declare_parameter("stop_bits", 1);
    declare_parameter("rs485_mode", false);
    declare_parameter("slave_ids", std::vector<int64_t>{1});
    declare_parameter("cycle_period_ms", 100);
    declare_parameter("response_timeout_ms", 100);
    declare_parameter("max_retries", 3);
    declare_parameter("config_period_cycles", 20);
    declare_parameter("offline_probe_cycles", 100);
    declare_parameter("send_to_all_slaves", true);
    declare_parameter("led_info_mode", 1);
    declare_parameter("led_time_coefficient", 2);
    declare_parameter("debug", false);

    cfg_.serial_device = get_parameter("serial_device").as_string();
    cfg_.baudrate      = static_cast<int>(get_parameter("baudrate").as_int());
    cfg_.data_bits     = static_cast<int>(get_parameter("data_bits").as_int());
    cfg_.stop_bits     = static_cast<int>(get_parameter("stop_bits").as_int());
    cfg_.rs485_mode    = get_parameter("rs485_mode").as_bool();
    cfg_.debug         = get_parameter("debug").as_bool();

    std::string parity = get_parameter("parity").as_string();
    cfg_.parity = parity.empty() ? 'N'
        : static_cast<char>(std::toupper(static_cast<unsigned char>(parity[0])));

    cfg_.cycle_period         = std::chrono::milliseconds(get_parameter("cycle_period_ms").as_int());
    cfg_.response_timeout     = std::chrono::milliseconds(get_parameter("response_timeout_ms").as_int());
    cfg_.max_retries          = static_cast<int>(get_parameter("max_retries").as_int());
    cfg_.config_period_cycles = static_cast<int>(get_parameter("config_period_cycles").as_int());
    cfg_.offline_probe_cycles = static_cast<int>(get_parameter("offline_probe_cycles").as_int());
    cfg_.send_to_all_slaves   = get_parameter("send_to_all_slaves").as_bool();
    cfg_.led_info_mode        = static_cast<uint16_t>(get_parameter("led_info_mode").as_int());
    cfg_.led_time_coefficient = static_cast<uint16_t>(get_parameter("led_time_coefficient").as_int());

    cfg_.slave_ids.clear(); // 防止 cfg_ 被复用/意外残留，先清空
    {
        // 先取到本地数组再遍历，避免对临时对象 range-for 的潜在问题
        const auto arr = get_parameter("slave_ids").as_integer_array();
        for (const auto v : arr)
            cfg_.slave_ids.push_back(static_cast<int>(v));
    }
    if (cfg_.slave_ids.empty())
        cfg_.slave_ids.push_back(1); // 未配置任何从站 ID 时回退为默认 1 号从站

    LOG_INFO("params: dev=%s baud=%d parity=%c data=%d stop=%d rs485=%d slaves=%zu "
             "cycle=%lldms timeout=%lldms retry=%d",
             cfg_.serial_device.c_str(), cfg_.baudrate, cfg_.parity, cfg_.data_bits,
             cfg_.stop_bits, cfg_.rs485_mode ? 1 : 0, cfg_.slave_ids.size(),
             static_cast<long long>(cfg_.cycle_period.count()),
             static_cast<long long>(cfg_.response_timeout.count()),
             cfg_.max_retries);

    // ===================== 初始化 Modbus 主站（打开串口）=====================
    master_ = std::make_shared<ModbusMaster>(cfg_);
    if (!master_->init())
    {
        RCLCPP_ERROR(get_logger(), "ModbusMaster init failed: %s", master_->last_error().c_str());
        return false;
    }

    // ===================== 订阅 /led_state/torso =====================
    // QoS 与 robot_ctrol_node 的发布端保持一致（默认 QoS：reliable + volatile + keep_last 10）
    led_state_sub_ = create_subscription<my_interfaces::msg::MsgInfoLedCmd>(
        "/led_state/torso", rclcpp::QoS(10),
        [this](const my_interfaces::msg::MsgInfoLedCmd::SharedPtr msg)
        {
            led_state_callback(msg);
        });

    LOG_INFO("subscribed /led_state/torso (my_interfaces::msg::MsgInfoLedCmd)");
    return true;
}

void serial_ModbusRTU_slavery_node::start()
{
    if (master_)
        master_->start();
}

void serial_ModbusRTU_slavery_node::stop()
{
    if (master_)
        master_->stop();
}

void serial_ModbusRTU_slavery_node::led_state_callback(
    const my_interfaces::msg::MsgInfoLedCmd::SharedPtr msg)
{
    // 解析 runmode/color，由 ModbusMaster 组成数据帧并入队（cycle task 集中发送）
    if (master_)
        master_->push_led_command(msg->runmode, msg->color, msg->id);
}

} // namespace serial_ModbusRTU_slavery

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<serial_ModbusRTU_slavery::serial_ModbusRTU_slavery_node>();
    if (!node->init())
    {
        RCLCPP_ERROR(node->get_logger(), "serial_ModbusRTU_slavery_node init failed");
        rclcpp::shutdown();
        return -1;
    }

    node->start();
    rclcpp::spin(node);
    node->stop();
    rclcpp::shutdown();
    return 0;
}
