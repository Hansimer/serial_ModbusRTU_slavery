#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <modbus/modbus.h>

#include "serial_ModbusRTU_slavery/led_protocol.hpp"

namespace serial_ModbusRTU_slavery
{

/// 单个从站运行状态
struct SlaveState
{
    int id = 1;                              // Modbus 从站地址（1~247）
    bool online = true;                      // 在线标志；连续重试失败后置为掉站
    int consecutive_failures = 0;            // 连续失败次数
    std::deque<ModbusWriteRequest> tx_queue; // 待发送队列（队列模式，FIFO）
    std::optional<ModbusWriteRequest> in_flight;      // 在途帧（失败时下一周期重试同一帧）
    std::optional<ModbusWriteRequest> last_desired;   // 最近一次期望/成功的帧（掉站恢复探测用）
    int cycles_since_config = 0;             // 距上次下发配置帧的周期数
    bool config_sent_once = false;           // 是否已至少成功下发过一次配置帧
};

/// 主站配置（由 ROS 参数填充）
struct ModbusMasterConfig
{
    std::string serial_device = "/dev/ttyTHS1"; // ttyTHS1 115200 8N1 -crtscts
    int baudrate = 115200;
    char parity = 'N'; // 'N' / 'E' / 'O'
    int data_bits = 8;
    int stop_bits = 1;
    bool rs485_mode = false;

    std::vector<int> slave_ids;         // 可配置从站地址列表（slavery 数量与 ID 均可配置），空则由节点回退为 {1}
    bool send_to_all_slaves = true;     // true: LED 指令广播到所有从站; false: 仅发给与 msg.id 匹配的从站

    std::chrono::milliseconds cycle_period{100};     // cycle task 周期
    std::chrono::milliseconds response_timeout{100}; // 单次等待从站应答的超时时间
    int max_retries = 3;                             // 最大连续重试次数（超过后默认掉站）

    int config_period_cycles = 20;   // 每 N 个周期下发一次配置信息帧（需求4），0=禁用
    int offline_probe_cycles = 100;  // 每 N 个周期对掉站设备做一次恢复探测，0=禁用（严格按"掉站后不再读写"）

    uint16_t led_info_mode = 1;         // 配置信息模式（需求4：01=呼吸）
    uint16_t led_time_coefficient = 2;  // 时间系数（需求4：02）

    bool debug = false; // libmodbus 调试输出
};

/// Modbus RTU 主站
///
/// 串口(tty)为公共资源，因此所有收发集中到 cycle task 单线程内完成，
/// 并使用 serial_mutex_ 互斥保护；外部（订阅回调）只允许通过 push_led_command
/// 向各从站队列入队，绝不直接触碰串口。
class ModbusMaster
{
public:
    explicit ModbusMaster(const ModbusMasterConfig& cfg);
    ~ModbusMaster();

    ModbusMaster(const ModbusMaster&) = delete;
    ModbusMaster& operator=(const ModbusMaster&) = delete;

    bool init();   // 打开串口并创建 libmodbus 上下文
    void start();  // 启动 cycle task 线程
    void stop();   // 停止 cycle task 线程

    /// 订阅回调入口：解析 runmode/color 后入队（线程安全）
    void push_led_command(int16_t runmode, int16_t color, int8_t src_id = 1);

    bool is_running() const { return running_.load(); }
    const std::string& last_error() const { return last_error_; }

private:
    void cycle_task();                       // 周期任务：集中发送 + 应答核对 + 重试
    void process_slave(size_t idx);          // 处理单个从站的一个待发帧
    bool try_send(const ModbusWriteRequest& req); // 单次发送并核对应答（串口互斥）
    void mark_online(SlaveState& s);
    void mark_offline(SlaveState& s);

    ModbusMasterConfig cfg_;
    modbus_t* ctx_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<int> cycle_count_{0};
    std::thread cycle_thread_;
    std::mutex serial_mutex_; // 串口 / ctx 互斥（公共资源保护）
    std::mutex queue_mutex_;  // 队列与从站状态互斥
    std::vector<SlaveState> slaves_;
    size_t round_robin_index_ = 0; // 仅 cycle task 线程访问
    std::string last_error_;
};

} // namespace serial_ModbusRTU_slavery
