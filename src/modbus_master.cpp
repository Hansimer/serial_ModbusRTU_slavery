#include "serial_modbus_rtu_slavery/modbus_master.hpp"

#include <cerrno>
#include <cstring>

#include "../include/log.hpp"

namespace serial_modbus_rtu_slavery
{

ModbusMaster::ModbusMaster(const ModbusMasterConfig& cfg)
  : cfg_(cfg)
{
}

ModbusMaster::~ModbusMaster()
{
    stop();
    if (ctx_)
    {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
}

bool ModbusMaster::init()
{
    // 串口参数: /dev/ttyTHS1 115200 8N1，无硬件流控(-crtscts)
    ctx_ = modbus_new_rtu(cfg_.serial_device.c_str(), cfg_.baudrate,
                          cfg_.parity, cfg_.data_bits, cfg_.stop_bits);
    if (!ctx_)
    {
        last_error_ = "modbus_new_rtu failed";
        return false;
    }

    if (cfg_.rs485_mode)
    {
        if (modbus_rtu_set_serial_mode(ctx_, MODBUS_RTU_RS485) == -1)
        {
            last_error_ = std::string("modbus_rtu_set_serial_mode(RS485) failed: ") + modbus_strerror(errno);
            modbus_free(ctx_);
            ctx_ = nullptr;
            return false;
        }
    }

    if (cfg_.debug)
        modbus_set_debug(ctx_, 1);

    // 应答超时：等待从站回应的最大时间
    const long long to_ms = cfg_.response_timeout.count();
    modbus_set_response_timeout(ctx_,
                                static_cast<uint32_t>(to_ms / 1000),
                                static_cast<uint32_t>((to_ms % 1000) * 1000));

    if (modbus_connect(ctx_) == -1)
    {
        last_error_ = std::string("modbus_connect(") + cfg_.serial_device + ") failed: " +
                      modbus_strerror(errno);
        modbus_free(ctx_);
        ctx_ = nullptr;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (const int id : cfg_.slave_ids)
        {
            if (id < 1 || id > 247)
            {
                LOG_WARN("invalid slave id %d, skip", id);
                continue;
            }
            SlaveState s;
            s.id = id;
            slaves_.push_back(std::move(s));
        }
    }
    if (slaves_.empty())
    {
        last_error_ = "no valid slave id configured";
        return false;
    }

    for (const auto& s : slaves_)
        LOG_INFO("slave registered: id=%d", s.id);
    return true;
}

void ModbusMaster::start()
{
    if (running_.load())
        return;
    running_.store(true);
    cycle_thread_ = std::thread(&ModbusMaster::cycle_task, this);
}

void ModbusMaster::stop()
{
    if (!running_.load())
        return;
    running_.store(false);
    if (cycle_thread_.joinable())
        cycle_thread_.join();
}

void ModbusMaster::push_led_command(int16_t runmode, int16_t color, int8_t src_id)
{
    auto led = make_led_request(runmode, color);
    if (!led)
    {
        LOG_WARN("ignore invalid led cmd: runmode=%d color=%d",
                 static_cast<int>(runmode), static_cast<int>(color));
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    for (auto& s : slaves_)
    {
        // 是否将指令下发到该从站
        if (!cfg_.send_to_all_slaves && s.id != static_cast<int>(src_id))
            continue;

        // 期望指令值发生变化：标记待同步，cycle_task 会读取寄存器 1002 比对，
        // 一致则不再 0x06 写入，不同则写入新指令
        if (!s.has_desired || s.desired_value != led->value)
        {
            LOG_INFO("slave[%d] new desired led cmd value=%u (runmode=%d color=%d)",
                     s.id, led->value, static_cast<int>(runmode), static_cast<int>(color));
            s.has_desired = true;
            s.desired_value = led->value;
            s.desired_changed = true; // 强制下个周期立即比对并（如有需要）写入
        }
    }
}

void ModbusMaster::cycle_task()
{
    const auto period = cfg_.cycle_period;
    LOG_INFO("cycle task started, period=%lldms, slaves=%zu",
             static_cast<long long>(period.count()), slaves_.size());

    while (running_.load())
    {
        const auto cycle_start = std::chrono::steady_clock::now();
        ++cycle_count_;

        if (!slaves_.empty())
        {
            // 单周期时间预算（9/10 周期），避免读/写超时拖垮 100ms 周期
            const auto budget = period * 9 / 10;
            for (size_t k = 0; k < slaves_.size(); ++k)
            {
                const size_t idx = (round_robin_index_ + k) % slaves_.size();
                const auto spent = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - cycle_start);
                if (spent >= budget)
                    break;
                process_slave(idx); // 内部仅在读写状态时短锁，串口发送在锁外
            }
            round_robin_index_ = (round_robin_index_ + 1) % slaves_.size();
        }

        // 补偿休眠到下一个周期边界，保持稳定节拍
        const auto spent = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - cycle_start);
        if (spent < period)
            std::this_thread::sleep_for(period - spent);
    }

    LOG_INFO("cycle task stopped");
}

void ModbusMaster::process_slave(size_t idx)
{
    // ---- 第0步（锁内）：掉站设备的恢复探测判定 ----
    bool probe = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const SlaveState& s = slaves_[idx];
        if (!s.online)
        {
            if (cfg_.offline_probe_cycles > 0 &&
                cycle_count_.load() % cfg_.offline_probe_cycles == 0)
                probe = true; // 仅做 0x03 读探测（比写更安全）
            else
                return;
        }
    }

    if (probe)
    {
        uint16_t st = 0;
        if (read_status(idx, st))
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            SlaveState& s = slaves_[idx];
            s.status_valid = true;
            s.last_status = st;
            mark_online(s);
        }
        return;
    }

    // ---- 第1步（锁外串口）：功能码 0x03 读取寄存器 1002（当前正在执行的指令）----
    uint16_t status = 0;
    const bool read_ok = read_status(idx, status);

    bool do_write = false;   // 是否需要 0x06 写新指令
    bool config_due = false; // 周期配置信息帧到期（需求4，仅参数开启时）
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        SlaveState& s = slaves_[idx];

        if (!read_ok)
        {
            ++s.consecutive_failures;
            if (s.consecutive_failures >= cfg_.max_retries)
                mark_offline(s); // 连续读失败达到上限 -> 掉站
            return;
        }
        s.consecutive_failures = 0;
        s.status_valid = true;
        s.last_status = status;

        // ---- 第2步：比对：一致则不进行 0x06 写入；不同则写新指令 ----
        if (!s.has_desired || s.desired_value == s.last_status)
        {
            // 无指令需求，或 1002 已等于期望指令 -> 无需写入
            s.desired_changed = false;
        }
        else
        {
            // 1002 != 期望指令：需要下发。期望值刚更新则立即写；
            // 否则按 max_retries 周期冷却重试，避免从站已应答但迟迟未执行时每周期刷写总线
            const int cur = cycle_count_.load();
            const bool cooldown_ok = s.last_write_cycle < 0 ||
                (cur - s.last_write_cycle) >= cfg_.max_retries;
            do_write = s.desired_changed || cooldown_ok;
        }

        // 需求4 周期配置帧：仅当本周期无需写 LED 指令且参数开启时下发
        // if (!do_write && cfg_.config_period_cycles > 0)
        // {
        //     ++s.cycles_since_config;
        //     if (!s.config_sent_once || s.cycles_since_config >= cfg_.config_period_cycles)
        //         config_due = true;
        // }
    }

    if (do_write)
    {
        // ---- 第3步（锁外串口）：功能码 0x06 写寄存器 2199 = 新指令 ----
        uint16_t desired = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            desired = slaves_[idx].desired_value;
        }
        const bool wok = write_led_value(idx, desired);

        std::lock_guard<std::mutex> lock(state_mutex_);
        SlaveState& s = slaves_[idx];
        if (wok)
        {
            s.consecutive_failures = 0;
            s.last_write_cycle = cycle_count_.load();
            s.desired_changed = false; // 已下发，等待下轮 1002 读回确认同步
        }
        else
        {
            ++s.consecutive_failures;
            if (s.consecutive_failures >= cfg_.max_retries)
                mark_offline(s);
        }
    }
    else if (config_due)
    {
        ModbusWriteRequest req = make_config_request(cfg_.led_info_mode, cfg_.led_time_coefficient);
        req.slave_id = static_cast<uint8_t>(slaves_[idx].id);
        const bool cok = try_send(req);

        std::lock_guard<std::mutex> lock(state_mutex_);
        SlaveState& s = slaves_[idx];
        if (cok)
        {
            s.config_sent_once = true;
            s.cycles_since_config = 0;
            s.consecutive_failures = 0;
        }
        else
        {
            ++s.consecutive_failures;
            if (s.consecutive_failures >= cfg_.max_retries)
                mark_offline(s);
        }
    }
}

void ModbusMaster::throttle_frame_interval()
{
    const auto delay = cfg_.frame_interval;
    if (delay <= delay.zero())
        return; // 配置为 0 时表示不限制帧间隔

    // 距上一帧结束的时间不足 delay 时，休眠补齐，确保相邻两帧间隔 >= delay
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = now - last_frame_end_;
    if (elapsed < delay)
        std::this_thread::sleep_for(delay - elapsed);
}

bool ModbusMaster::read_status(size_t idx, uint16_t& value)
{
    // tty 为公共资源：所有串口收发必须经此互斥，防止并发写
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!ctx_)
        return false;

    const int slave_id = slaves_[idx].id; // 从站 id 注册后不再变化
    if (modbus_set_slave(ctx_, slave_id) == -1)
    {
        LOG_WARN("slave[%d] modbus_set_slave failed", slave_id);
        return false;
    }

    modbus_flush(ctx_); // 清空残留字节，避免污染本次应答核对

    // 发送前确保与上一帧间隔 >= cfg_.frame_interval（默认 4ms）
    throttle_frame_interval();

    uint16_t reg = 0;
    // 功能码 0x03 读保持寄存器 1002（1 个寄存器）
    const int rc = modbus_read_registers(ctx_, kLedStatusRegAddr, 1, &reg);

    // 帧事务结束（收到应答或超时），记录为下一帧间隔的起点
    last_frame_end_ = std::chrono::steady_clock::now();

    if (rc == 1) // 成功返回读取的寄存器个数
    {
        value = reg;
        LOG_INFO("slave[%d] RX reg%d = %u", slave_id, kLedStatusRegAddr, reg);
        return true;
    }

    LOG_WARN("slave[%d] RX reg%d FAIL(%s)", slave_id, kLedStatusRegAddr, modbus_strerror(errno));
    return false;
}

bool ModbusMaster::write_led_value(size_t idx, uint16_t value)
{
    ModbusWriteRequest req;
    req.slave_id = static_cast<uint8_t>(slaves_[idx].id); // 从站 id 注册后不再变化
    req.function = kFuncWriteRegister;
    req.address  = kLedRegAddr;
    req.value    = value;
    req.role     = FrameRole::LedCommand;
    return try_send(req);
}

bool ModbusMaster::try_send(const ModbusWriteRequest& req)
{
    // tty 为公共资源：所有串口收发必须经此互斥，防止并发写
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!ctx_)
        return false;

    if (modbus_set_slave(ctx_, req.slave_id) == -1)
    {
        LOG_WARN("slave[%d] modbus_set_slave failed", req.slave_id);
        return false;
    }

    modbus_flush(ctx_); // 清空残留字节，避免污染本次应答核对

    const std::string hex = frame_to_hex_string(req);
    int rc = -1;
    if (req.function == kFuncWriteCoil || req.function == kFuncWriteRegister)
    {
        // 发送前确保与上一帧间隔 >= cfg_.frame_interval（默认 4ms）
        throttle_frame_interval();
        if (req.function == kFuncWriteCoil)
            rc = modbus_write_bit(ctx_, req.address, (req.value == kCoilValueOn) ? 1 : 0);
        else
            rc = modbus_write_register(ctx_, req.address, req.value);

        // 帧事务结束（收到应答/回显或超时），记录为下一帧间隔的起点
        last_frame_end_ = std::chrono::steady_clock::now();
    }
    else
    {
        LOG_WARN("unsupported function 0x%02X", req.function);
        return false;
    }

    // modbus_write_bit / modbus_write_register 成功返回正数（本机 libmodbus 3.1.6 返回 1）
    if (rc > 0)
    {
        LOG_INFO("slave[%d] TX %s -> ACK", req.slave_id, hex.c_str());
        return true;
    }

    LOG_WARN("slave[%d] TX %s -> FAIL(%s)", req.slave_id, hex.c_str(), modbus_strerror(errno));
    return false;
}

void ModbusMaster::mark_online(SlaveState& s)
{
    if (!s.online)
    {
        LOG_INFO("slave[%d] recovered, back online", s.id);
        s.config_sent_once = false; // 恢复后立即补发一次配置帧
        s.cycles_since_config = 0;
    }
    s.online = true;
    s.consecutive_failures = 0;
}

void ModbusMaster::mark_offline(SlaveState& s)
{
    if (s.online)
    {
        LOG_WARN("slave[%d] offline after %d consecutive failures, stop R/W for it",
                 s.id, cfg_.max_retries);
    }
    s.online = false;
}

} // namespace serial_modbus_rtu_slavery
