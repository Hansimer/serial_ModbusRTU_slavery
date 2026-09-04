#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <modbus/modbus.h>

#include "serial_modbus_rtu_slavery/led_protocol.hpp"

namespace serial_modbus_rtu_slavery
{

/// 单个从站运行状态
/// 工作方式（先读后写、读回确认）：
///   1. 每周期用功能码 0x03 读取寄存器 1002（从站“当前正在执行的指令”）；
///   2. 将 1002 数据与期望指令值（来自 /led_state/torso 的最新 runmode+color）比对；
///   3. 一致则不做 0x06 写入；不同则用 0x06 向寄存器 2199 写入新指令，下周期读回确认。
struct SlaveState
{
    int id = 1;              // Modbus 从站地址（1~247）
    bool online = true;      // 在线标志；连续读/写失败达到上限后置为掉站

    // ===== 期望指令（0x06 写寄存器 2199 的值：2010~2033）=====
    bool     has_desired = false;   // 是否已收到过有效 LED 指令
    uint16_t desired_value = 0;     // 最新期望指令值（runmode/color 的编码值）
    bool     desired_changed = true;// 期望值刚更新、尚未被 1002 确认同步（触发立即写入）

    // ===== 从站当前执行状态（0x03 读寄存器 1002）=====
    uint16_t last_status = 0;   // 最近读到的 1002 数据（当前正在执行的指令）
    bool     status_valid = false; // 是否已成功读过一次 1002

    // ===== 通信健壮性 =====
    int consecutive_failures = 0; // 连续读/写失败次数，达到 max_retries 后掉站
    int last_write_cycle = -1;    // 最近一次写 2199 成功的周期序号；-1=尚未写过（用于写后冷却，避免反复刷写）

    // ===== 需求4 周期配置帧（仅 config_period_cycles>0 时启用）=====
    bool config_sent_once = false; // 是否已至少成功下发过一次配置帧
    int  cycles_since_config = 0;  // 距上次下发配置帧的周期数
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
    std::chrono::milliseconds frame_interval{4};     // 相邻两帧发送的最小时间间隔（默认 4ms，避免总线过载）
    int max_retries = 3;                             // 最大连续失败次数（读/写，超过后默认掉站）

    int config_period_cycles = 20;   // 每 N 个周期下发一次配置信息帧（需求4），0=禁用
    int offline_probe_cycles = 100;  // 每 N 个周期对掉站设备做一次恢复探测（0x03 读 1002），0=禁用

    uint16_t led_info_mode = 1;         // 配置信息模式（需求4：01=呼吸）
    uint16_t led_time_coefficient = 2;  // 时间系数（需求4：02）

    bool debug = false; // libmodbus 调试输出
};

/// Modbus RTU 主站
///
/// 串口(tty)为公共资源，因此所有收发集中到 cycle task 单线程内完成，
/// 并使用 serial_mutex_ 互斥保护；外部（订阅回调）只允许通过 push_led_command
/// 更新各从站的“期望指令值”，绝不直接触碰串口。
///
/// 每个周期对在线从站执行：0x03 读寄存器 1002 -> 与期望指令值比对
/// （一致则不写；不同则 0x06 写寄存器 2199 下发新指令）。
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

    /// 订阅回调入口：解析 runmode/color，更新期望指令值（线程安全）
    /// 实际是否下发由 cycle_task 中比对寄存器 1002 决定
    void push_led_command(int16_t runmode, int16_t color, int8_t src_id = 1);

    bool is_running() const { return running_.load(); }
    const std::string& last_error() const { return last_error_; }

private:
    void cycle_task();                       // 周期任务：读 1002 -> 比对 -> 按需 0x06 写 2199
    void process_slave(size_t idx);          // 处理单个从站的一个周期（读 + 可能的写）
    bool read_status(size_t idx, uint16_t& value); // 功能码 0x03 读寄存器 1002（串口互斥）
    bool write_led_value(size_t idx, uint16_t value); // 功能码 0x06 写寄存器 2199（串口互斥）
    bool try_send(const ModbusWriteRequest& req); // 通用单次写帧发送并核对应答（串口互斥）
    void throttle_frame_interval();               // 相邻两帧最小时间间隔控制（须在 serial_mutex_ 锁内调用）
    void mark_online(SlaveState& s);
    void mark_offline(SlaveState& s);

    ModbusMasterConfig cfg_;
    modbus_t* ctx_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<int> cycle_count_{0};
    std::thread cycle_thread_;
    std::mutex serial_mutex_; // 串口 / ctx 互斥（公共资源保护）
    std::mutex state_mutex_;  // 各从站期望/状态字段互斥
    std::vector<SlaveState> slaves_;
    size_t round_robin_index_ = 0; // 仅 cycle task 线程访问
    std::string last_error_;
    std::chrono::steady_clock::time_point last_frame_end_{}; // 上一次帧收发结束时刻（仅 serial_mutex_ 保护下访问）
};

} // namespace serial_modbus_rtu_slavery
