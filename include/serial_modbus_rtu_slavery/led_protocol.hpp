#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace serial_modbus_rtu_slavery
{

// ===================== LED 运行模式（对应 MsgInfoLedCmd.runmode）=====================
// 当前协议仅支持三种显示模式：常亮 / 呼吸 / 闪烁
// （runmode=0 即旧协议的“关闭”，新协议未定义对应的写入值，收到后忽略）
enum LedRunMode : int16_t
{
    LED_RUNMODE_CONSTANT = 1, // 常亮
    LED_RUNMODE_BREATH   = 2, // 呼吸
    LED_RUNMODE_BLINK    = 3, // 闪烁
};

// ===================== LED 颜色（对应 MsgInfoLedCmd.color）=====================
// 当前协议仅支持四种颜色：红 / 绿 / 蓝 / 黄（其余颜色值收到后忽略）
enum LedColor : int16_t
{
    LED_COLOR_RED    = 1, // 红色
    LED_COLOR_GREEN  = 2, // 绿色
    LED_COLOR_BLUE   = 3, // 蓝色
    LED_COLOR_YELLOW = 4, // 黄色
};

// ===================== 新协议：功能码 0x06 写单寄存器，固定寄存器地址 2199 =====================
/// 躯干 LED 控制寄存器地址（十进制 2199）
constexpr uint16_t kLedRegAddr = 2199;

/// 各 runmode 的写入值基址：实际写入值 = 基址 + (color - LED_COLOR_RED)
///   常亮: 红色2010 / 绿色2011 / 蓝色2012 / 黄色2013
///   呼吸: 红色2020 / 绿色2021 / 蓝色2022 / 黄色2023
///   闪烁: 红色2030 / 绿色2031 / 蓝色2032 / 黄色2033
constexpr uint16_t kLedValueBaseConstant = 2010; // 常亮
constexpr uint16_t kLedValueBaseBreath   = 2020; // 呼吸
constexpr uint16_t kLedValueBaseBlink    = 2030; // 闪烁

// Modbus 功能码
constexpr uint8_t kFuncReadRegisters  = 0x03; // 读保持寄存器（读取当前执行指令）
constexpr uint8_t kFuncWriteCoil      = 0x05; // 写单线圈（保留，通用写帧支持）
constexpr uint8_t kFuncWriteRegister  = 0x06; // 写单寄存器

/// 从站当前正在执行的指令（寄存器 1002，功能码 0x03 定时读取；数值与写入 2199 的指令值一致）
constexpr uint16_t kLedStatusRegAddr = kLedRegAddr;

// 线圈值（仅当功能码为 0x05 时使用）
constexpr uint16_t kCoilValueOn = 0xFF00;

// 配置信息寄存器（需求4：01 06 00 04 <信息模式> <时间系数>）
constexpr uint16_t kConfigRegAddr = 0x0004;

/// 帧用途：LED 指令与周期配置帧同为功能码 0x06，需据此区分应答后的处理逻辑
enum class FrameRole : uint8_t
{
    LedCommand,  // LED 指令帧（0x06 写寄存器 kLedRegAddr=2199）
    ConfigFrame, // 配置信息帧（需求4，0x06 写寄存器 kConfigRegAddr=0x0004）
};

/// 一条 Modbus 写请求（写线圈 / 写寄存器）
struct ModbusWriteRequest
{
    uint8_t   slave_id = 1; // 从站地址 1~247
    uint8_t   function = kFuncWriteRegister;
    uint16_t  address  = 0;
    uint16_t  value    = 0;
    FrameRole role     = FrameRole::LedCommand;

    bool operator==(const ModbusWriteRequest& other) const
    {
        return slave_id == other.slave_id &&
               function  == other.function  &&
               address   == other.address   &&
               value     == other.value;
    }
};

/// 由 (runmode, color) 生成 LED 写寄存器请求（0x06 写寄存器 2199）
/// 写入值 = 2010/2020/2030(常亮/呼吸/闪烁) + (color - 1)，如：常亮红=2010、闪烁黄=2033
/// @return 非法 runmode/color 时返回空 optional（忽略该指令）
std::optional<ModbusWriteRequest> make_led_request(int16_t runmode, int16_t color);

/// 生成配置信息帧（func=0x06, addr=0x0004, value=(info_mode<<8)|time_coefficient）
ModbusWriteRequest make_config_request(uint16_t info_mode, uint16_t time_coefficient);

/// Modbus RTU CRC16（多项式 0xA001）
uint16_t modbus_crc16(const uint8_t* data, size_t len);

/// 将请求序列化为完整 RTU 帧（含 CRC，CRC 低字节在前）
std::vector<uint8_t> serialize_frame(const ModbusWriteRequest& req);

/// 将请求格式化为 "01 06 08 97 07 DA B9 ED"（0x06 写寄存器 2199=常亮红）十六进制字符串（日志/调试用）
std::string frame_to_hex_string(const ModbusWriteRequest& req);

} // namespace serial_modbus_rtu_slavery
