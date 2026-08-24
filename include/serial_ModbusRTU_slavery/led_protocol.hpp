#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace serial_ModbusRTU_slavery
{

// ===================== LED 运行模式（对应 MsgInfoLedCmd.runmode）=====================
enum LedRunMode : int16_t
{
    LED_RUNMODE_STOP     = 0, // 关闭显示
    LED_RUNMODE_CONSTANT = 1, // 常亮
    LED_RUNMODE_BREATH   = 2, // 呼吸
    LED_RUNMODE_BLINK    = 3, // 闪烁
};

// ===================== LED 颜色（对应 MsgInfoLedCmd.color）=====================
enum LedColor : int16_t
{
    LED_COLOR_RED        = 1, // 红色
    LED_COLOR_GREEN      = 2, // 绿色
    LED_COLOR_BLUE       = 3, // 蓝色
    LED_COLOR_YELLOW     = 4, // 黄色
    LED_COLOR_PURPLE     = 5, // 紫色
    LED_COLOR_CYAN       = 6, // 青色
    LED_COLOR_WHITE      = 7, // 白色
    LED_COLOR_WARM_WHITE = 8, // 暖白色
};

// 各 runmode 的寄存器地址基址（实际地址 = 基址 + color - 1）
constexpr uint16_t kLedBaseConstant = 0x0000; // 常亮  0x00~0x07
constexpr uint16_t kLedBaseBreath   = 0x0030; // 呼吸  0x30~0x37
constexpr uint16_t kLedBaseBlink    = 0x0040; // 闪烁  0x40~0x47
constexpr uint16_t kLedAddrOff      = 0x0004; // 关闭显示（线圈写 0）

// Modbus 功能码
constexpr uint8_t kFuncWriteCoil     = 0x05; // 写单线圈
constexpr uint8_t kFuncWriteRegister = 0x06; // 写单寄存器

// 线圈值
constexpr uint16_t kCoilValueOn  = 0xFF00;
constexpr uint16_t kCoilValueOff = 0x0000;

// 配置信息寄存器（需求4：01 06 00 04 <信息模式> <时间系数>）
constexpr uint16_t kConfigRegAddr = 0x0004;

/// 一条 Modbus 写请求（写线圈 / 写寄存器）
struct ModbusWriteRequest
{
    uint8_t  slave_id = 1; // 从站地址 1~247
    uint8_t  function = kFuncWriteCoil;
    uint16_t address  = 0;
    uint16_t value    = 0;

    bool operator==(const ModbusWriteRequest& other) const
    {
        return slave_id == other.slave_id &&
               function  == other.function  &&
               address   == other.address   &&
               value     == other.value;
    }
};

/// 由 (runmode, color) 生成 LED 写线圈请求
/// @return 非法 runmode/color 时返回空 optional（忽略该指令）
std::optional<ModbusWriteRequest> make_led_request(int16_t runmode, int16_t color);

/// 生成配置信息帧（func=0x06, addr=0x0004, value=(info_mode<<8)|time_coefficient）
ModbusWriteRequest make_config_request(uint16_t info_mode, uint16_t time_coefficient);

/// Modbus RTU CRC16（多项式 0xA001）
uint16_t modbus_crc16(const uint8_t* data, size_t len);

/// 将请求序列化为完整 RTU 帧（含 CRC，CRC 低字节在前）
std::vector<uint8_t> serialize_frame(const ModbusWriteRequest& req);

/// 将请求格式化为 "01 05 00 00 FF 00 8C 3A" 十六进制字符串（日志/调试用）
std::string frame_to_hex_string(const ModbusWriteRequest& req);

} // namespace serial_ModbusRTU_slavery
