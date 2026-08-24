#include "serial_ModbusRTU_slavery/modbus_master.hpp"

#include <cerrno>
#include <cstring>

#include "../include/log.hpp"

namespace serial_ModbusRTU_slavery
{

namespace
{
/// 队列长度上限（防止指令积压导致总线拥堵）
constexpr size_t kMaxQueueSize = 16;
} // namespace

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
        std::lock_guard<std::mutex> lock(queue_mutex_);
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

    std::lock_guard<std::mutex> lock(queue_mutex_);
    for (auto& s : slaves_)
    {
        // 是否将指令发送到该从站
        if (!cfg_.send_to_all_slaves && s.id != static_cast<int>(src_id))
            continue;

        ModbusWriteRequest req = *led;
        req.slave_id = static_cast<uint8_t>(s.id);

        // 去重：与在途帧或队尾帧相同则不入队（避免重复刷总线）
        if (s.in_flight && *s.in_flight == req)
            continue;
        if (!s.tx_queue.empty() && s.tx_queue.back() == req)
            continue;

        // 无论在线/掉站都记录"期望状态"，供掉站恢复探测使用
        s.last_desired = req;

        if (s.tx_queue.size() >= kMaxQueueSize)
            s.tx_queue.pop_front(); // 队列积压保护：丢弃最旧指令，保留最新
        s.tx_queue.push_back(req);
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
            // 单周期时间预算（9/10 周期），避免超时/重试拖垮 100ms 周期
            const auto budget = period * 9 / 10;
            for (size_t k = 0; k < slaves_.size(); ++k)
            {
                const size_t idx = (round_robin_index_ + k) % slaves_.size();
                const auto spent = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - cycle_start);
                if (spent >= budget)
                    break;
                process_slave(idx); // 内部仅在取帧/回写状态时短锁，串口发送在外
            }
            round_robin_index_ = (round_robin_index_ + 1) % slaves_.size();
        }

        // 补偿休眠到下一个周期边界，保持 100ms 节拍
        const auto spent = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - cycle_start);
        if (spent < period)
            std::this_thread::sleep_for(period - spent);
    }

    LOG_INFO("cycle task stopped");
}

void ModbusMaster::process_slave(size_t idx)
{
    std::optional<ModbusWriteRequest> job;
    bool is_probe = false;

    // ---- 第1步：在锁内取一个待发送的帧 ----
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        SlaveState& s = slaves_[idx];

        // 掉站设备：不进行正常读写，仅按周期做一次恢复探测
        if (!s.online)
        {
            if (cfg_.offline_probe_cycles > 0 && (cycle_count_.load() % cfg_.offline_probe_cycles) == 0)
            {
                if (s.last_desired)
                    job = *s.last_desired;
                else
                {
                    job = make_config_request(cfg_.led_info_mode, cfg_.led_time_coefficient);
                    job->slave_id = static_cast<uint8_t>(s.id);
                }
                is_probe = true;
            }
            if (!job)
                return;
        }
        else
        {
            // 在途帧失败 -> 下一周期重试同一帧
            if (s.in_flight)
            {
                job = *s.in_flight;
            }
            else if (!s.tx_queue.empty())
            {
                job = s.tx_queue.front();
                s.tx_queue.pop_front();
                s.in_flight = *job;
            }
            else if (cfg_.config_period_cycles > 0)
            {
                // 队列空时周期下发配置信息帧（需求4）
                ++s.cycles_since_config;
                if (!s.config_sent_once || s.cycles_since_config >= cfg_.config_period_cycles)
                {
                    job = make_config_request(cfg_.led_info_mode, cfg_.led_time_coefficient);
                    job->slave_id = static_cast<uint8_t>(s.id);
                }
            }
            if (!job)
                return;
        }
    }

    // ---- 第2步：在锁外发送（串口公共资源，内部 serial_mutex_ 互斥）----
    const bool ok = try_send(*job);

    // ---- 第3步：在锁内根据应答结果更新状态 ----
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        SlaveState& s = slaves_[idx];

        if (is_probe)
        {
            if (ok)
                mark_online(s);
            return;
        }

        if (ok) // 应答核对通过（libmodbus 已完成 CRC 与回显校验）
        {
            if (job->function == kFuncWriteCoil)
                s.in_flight.reset();
            else // 配置帧
            {
                s.config_sent_once = true;
                s.cycles_since_config = 0;
            }
            s.last_desired = *job;
            s.consecutive_failures = 0;
        }
        else // 超时/校验失败 -> 累计重试，达到上限后默认掉站
        {
            ++s.consecutive_failures;
            if (s.consecutive_failures >= cfg_.max_retries)
            {
                if (job->function == kFuncWriteCoil)
                    s.in_flight.reset(); // 丢弃该帧，停止对该从站的读写
                mark_offline(s);
            }
        }
    }
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
    if (req.function == kFuncWriteCoil)
    {
        rc = modbus_write_bit(ctx_, req.address, (req.value == kCoilValueOn) ? 1 : 0);
    }
    else if (req.function == kFuncWriteRegister)
    {
        rc = modbus_write_register(ctx_, req.address, req.value);
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
    s.in_flight.reset();
}

} // namespace serial_ModbusRTU_slavery

