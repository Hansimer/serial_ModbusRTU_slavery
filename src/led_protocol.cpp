#include "serial_modbus_rtu_slavery/led_protocol.hpp"

#include <iomanip>
#include <sstream>

namespace serial_modbus_rtu_slavery
{

uint16_t modbus_crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

std::optional<ModbusWriteRequest> make_led_request(int16_t runmode, int16_t color)
{
    // 新协议：功能码 0x06 写单寄存器，固定写寄存器地址 2199
    if (color < LED_COLOR_RED || color > LED_COLOR_YELLOW)
        return std::nullopt; // 新协议仅支持 红/绿/蓝/黄 四种颜色

    uint16_t base = 0;
    switch (runmode)
    {
        case LED_RUNMODE_CONSTANT: base = kLedValueBaseConstant; break; // 常亮 2010~2013
        case LED_RUNMODE_BREATH:   base = kLedValueBaseBreath;   break; // 呼吸 2020~2023
        case LED_RUNMODE_BLINK:    base = kLedValueBaseBlink;    break; // 闪烁 2030~2033
        default:
            return std::nullopt; // runmode 仅支持 常亮/呼吸/闪烁
    }

    ModbusWriteRequest req;
    req.function = kFuncWriteRegister;
    req.address  = kLedRegAddr;
    req.value    = static_cast<uint16_t>(base + static_cast<uint16_t>(color - LED_COLOR_RED));
    req.role     = FrameRole::LedCommand;
    return req;
}

ModbusWriteRequest make_config_request(uint16_t info_mode, uint16_t time_coefficient)
{
    ModbusWriteRequest req;
    req.function = kFuncWriteRegister;
    req.address  = kConfigRegAddr;
    req.value    = static_cast<uint16_t>(((info_mode & 0xFF) << 8) | (time_coefficient & 0xFF));
    req.role     = FrameRole::ConfigFrame;
    return req;
}

std::vector<uint8_t> serialize_frame(const ModbusWriteRequest& req)
{
    std::vector<uint8_t> data = {
        req.slave_id,
        req.function,
        static_cast<uint8_t>((req.address >> 8) & 0xFF),
        static_cast<uint8_t>(req.address & 0xFF),
        static_cast<uint8_t>((req.value >> 8) & 0xFF),
        static_cast<uint8_t>(req.value & 0xFF),
    };
    const uint16_t crc = modbus_crc16(data.data(), data.size());
    data.push_back(static_cast<uint8_t>(crc & 0xFF));        // CRC 低字节在前
    data.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF)); // 高字节在后
    return data;
}

std::string frame_to_hex_string(const ModbusWriteRequest& req)
{
    std::ostringstream oss;
    const auto bytes = serialize_frame(req);
    for (size_t i = 0; i < bytes.size(); ++i)
    {
        if (i)
            oss << ' ';
        oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(bytes[i]);
    }
    return oss.str();
}

} // namespace serial_modbus_rtu_slavery
