#pragma once

#include "data_stream.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace reanaut
{

//
// Requests
//

//
// # Bytestream
//
// [1] 0xAA
// [1] 0x55
// [1] Length
// [N] Payload
// [1] Checksum
//

//
// # Payload
//
// [N] Sub-Payload 0
// [N] Sub-Payload 1
// [N] Sub-Payload 2
// ...
// [N] Sub-Payload M-1
//
// # Sub-Payload
//
// [1] Header
// [1] Length
// [N] Data
//

//
// (BaseControl)
// [1] 0x01
// [1] 0x04
// [2] speed (mm/s)
// [2] radius (mm)
//
// 1. pure translation => {speed}, {0}
// 2. pure rotation => {w * b / 2}, {1}
// 3. translation + rotation => {speed * (radius + b / 2); radius >  1}, {radius}
//                              {speed * (radius - b / 2); radius < -1}
//
// b (wheelbase in mm) = 230
// w (rotation speed in rad/s)
//

//
// (Sound)
// [1] 0x03
// [1] 0x03
// [2] note
// [1] duration (ms)
//
// note = 1 / (f * a)
// a = 0.00000275
//

//
// (SoundSequence)
// [1] 0x04
// [1] 0x01
// [1] sequence
//
// 0 = ON
// 1 = OFF
// 2 = RECHARGE
// 3 = BUTTON
// 4 = ERROR
// 5 = CLEANINGSTART
// 6 = CLEANINGEND
//

//
// (RequestExtra)
// [1] 0x09
// [1] 0x02
// [2] request flags
//
// 0x01 = hardware version
// 0x02 = firmware version
// 0x08 = unique device id
//

//
// (GeneralPurposeOutput)
// [1] 0x0C
// [1] 0x02
// [2] output flags
//
// {digital outputs}
// 0x0001 = ch 0
// 0x0002 = ch 1
// 0x0004 = ch 2
// 0x0008 = ch 3
//
// {external powers}
// 0x0010 = 3.3V
// 0x0020 = 5V
// 0x0040 = 12V/5A
// 0x0080 = 12V/1.5A
//
// {LEDs}
// 0x0100 = red LED1
// 0x0200 = green LED1
// 0x0400 = red LED2
// 0x0800 = green LED2
//

//
// (SetControllerGain)
// [1] 0x01
// [1] 0x0D
// [1] type
// [4] P gain
// [4] I gain
// [4] D gain
//
// 0x00 = factory default PID
// 0x01 = user configured PID
//
// P = Kp * 1000
// I = Ki * 1000
// D = Kd * 1000
//

//
// (GetControllerGain)
// [ 1] 0x01
// [ 1] 0x0E
// [14] unused
//

class Command
{
public:

    static constexpr uint8_t  kMagicByte1               = 0xAA;
    static constexpr uint8_t  kMagicByte2               = 0x55;
    static constexpr uint32_t kOffsetByte1              = 0 * 8;
    static constexpr uint32_t kOffsetByte2              = 1 * 8;
    static constexpr uint32_t kOffsetByte3              = 2 * 8;
    static constexpr uint32_t kOffsetByte4              = 3 * 8;
    static constexpr uint8_t  kBaseControlSize          = 4;
    static constexpr uint8_t  kSoundSize                = 3;
    static constexpr uint8_t  kSoundSequenceSize        = 1;
    static constexpr uint8_t  kRequestExtraSize         = 2;
    static constexpr uint8_t  kGeneralPurposeOutputSize = 2;
    static constexpr uint8_t  kSetControllerGainSize    = 13;
    static constexpr uint8_t  kGetControllerGainSize    = 14;

    enum CommandId : uint8_t
    {
        BaseControl          = 1,
        Sound                = 3,
        SoundSequence        = 4,
        RequestExtra         = 9,
        GeneralPurposeOutput = 12,
        SetControllerGain    = 13,
        GetControllerGain    = 14,
    };

    enum SoundSequenceType : uint8_t
    {
        On            = 0,
        Off           = 1,
        Recharge      = 2,
        Button        = 3,
        Error         = 4,
        CleaningStart = 5,
        CleaningEnd   = 6,
    };

    // NOLINTNEXTLINE(performance-enum-size)
    enum RequestExtraFlag : uint16_t
    {
        HardwareVersion = 0x01,
        FirmwareVersion = 0x02,
        UniqueDeviceId  = 0x08,
    };

    // NOLINTNEXTLINE(performance-enum-size)
    enum GpioDigitalFlag : uint16_t
    {
        DigitalOuT0 = 0x0001,
        DigitalOuT1 = 0x0002,
        DigitalOuT2 = 0x0004,
        DigitalOuT3 = 0x0008,
    };

    // NOLINTNEXTLINE(performance-enum-size)
    enum GpioPowerFlag : uint16_t
    {
        Power33V    = 0x0010,
        Power5V     = 0x0020,
        Power12V5A  = 0x0040,
        Power12V15A = 0x0080,
    };

    enum GpioLedFlag : uint16_t
    {
        Led1Red   = 0x0100,
        Led1Green = 0x0200,
        Led2Red   = 0x0400,
        Led2Green = 0x0800,
    };

    enum ControllerGainType : uint8_t
    {
        FactoryDefault = 0x00,
        UserConfigured = 0x01,
    };

    auto baseControl(uint16_t speed, uint16_t radius) -> Command&;
    auto sound(uint16_t note, uint8_t duration) -> Command&;
    auto soundSequence(SoundSequenceType sequence) -> Command&;
    auto requestExtra(uint16_t flags) -> Command&;
    auto generalPurposeOutput(uint16_t flags) -> Command&;
    auto setControllerGain(uint8_t type, uint32_t kp, uint32_t ki, uint32_t kd) -> Command&;
    auto getControllerGain() -> Command&;

    [[nodiscard]]
    auto buildPacket() const -> std::vector<uint8_t>;
    void buildPacket(std::vector<uint8_t>& out) const;
    void reset();
    auto size() -> size_t;

protected:

    void append(uint8_t value);
    void append(size_t count, uint8_t value);
    void append(uint16_t value);
    void append(uint32_t value);

    static void buildPacket(const std::vector<uint8_t>& payload, std::vector<uint8_t>& out);

private:

    std::vector<uint8_t> m_payload;
};

class Feedback
{
public:

    enum FeedbackId : uint8_t
    {
        BasicSensorData     = 1,
        DockingIr           = 3,
        InertialSensor      = 4,
        CliffSensor         = 5,
        CurrentInfo         = 6,
        HardwareVersion     = 10,
        FirmwareVersion     = 11,
        RawDataOf3DGyro     = 13,
        GeneralPurposeInput = 16,
        UniqueDeviceId      = 19,
        ControllerInfo      = 21,
    };

    struct BasicSensors
    {
        uint16_t timestamp{};        //
        uint8_t  bumper{};           // Right (0x01), Center (0x02), Left (0x04)
        uint8_t  wheelDrop{};        // Right (0x01), Left (0x02)
        uint8_t  cliff{};            // Right (0x01), Center (0x02), Left (0x04)
        uint16_t leftEncoder{};      //
        uint16_t rightEncoder{};     //
        int8_t   leftPwm{};          //
        int8_t   rightPwm{};         //
        uint8_t  buttons{};          // B0 (0x01), B1 (0x02), B2 (0x04)
        uint8_t  charger{};          // 0: Disconnected, 1: Adapter, 2: Dock
        uint8_t  battery{};          // Voltage in 0.1V units
        uint8_t  overcurrentFlags{}; // Right wheel (0x01), Left wheel (0x02)
    };

    struct DockingStation
    {
        uint8_t rightSignal{};
        uint8_t centerSignal{};
        uint8_t leftSignal{};
    };

    struct Inertial
    {
        int16_t angle{};
        int16_t angleRate{};
    };

    struct Cliff
    {
        uint16_t rightCliffSignal{};
        uint16_t centerCliffSignal{};
        uint16_t leftCliffSignal{};
    };

    struct Current
    {
        uint8_t leftMotor{};  // current in 10mA
        uint8_t rightMotor{}; // current in 10mA
    };

    struct Version
    {
        uint8_t patch{};
        uint8_t minor{};
        uint8_t major{};
    };

    struct Gpio
    {
        uint16_t digitalInput{}; // 4-bit value representing DI_0 to DI_3
        uint16_t analogInputCh0{};
        uint16_t analogInputCh1{};
        uint16_t analogInputCh2{};
        uint16_t analogInputCh3{};
    };

    struct Udid
    {
        uint32_t udid0{};
        uint32_t udid1{};
        uint32_t udid2{};
    };

    struct ControllerGains
    {
        uint8_t  type{};
        uint32_t p{};
        uint32_t i{};
        uint32_t d{};
    };

    // reset flags and parse packet data
    auto parse(std::span<std::uint8_t> packet) -> bool;
    void resetFlags();

    [[nodiscard]] auto getBasicSensors() const noexcept -> const BasicSensors& { return m_basicSensors; }
    [[nodiscard]] auto getDockingIr() const noexcept -> const DockingStation& { return m_dockingIr; }
    [[nodiscard]] auto getInertial() const noexcept -> const Inertial& { return m_inertial; }
    [[nodiscard]] auto getHardwareVersion() const noexcept -> const Version& { return m_hardwareVersion; }
    [[nodiscard]] auto getFirmwareVersion() const noexcept -> const Version& { return m_firmwareVersion; }
    [[nodiscard]] auto getGpio() const noexcept -> const Gpio& { return m_gpio; }
    [[nodiscard]] auto getUdid() const noexcept -> const Udid& { return m_udid; }
    [[nodiscard]] auto getControllerGains() const noexcept -> const ControllerGains& { return m_controllerGains; }

    [[nodiscard]] auto hasBasicSensors() const noexcept -> bool { return m_hasBasicSensors; }
    [[nodiscard]] auto hasDockingIr() const noexcept -> bool { return m_hasDockingIr; }
    [[nodiscard]] auto hasInertial() const noexcept -> bool { return m_hasInertial; }
    [[nodiscard]] auto hasHardwareVersion() const noexcept -> bool { return m_hasHardwareVersion; }
    [[nodiscard]] auto hasFirmwareVersion() const noexcept -> bool { return m_hasFirmwareVersion; }
    [[nodiscard]] auto hasGpio() const noexcept -> bool { return m_hasGpio; }
    [[nodiscard]] auto hasUuid() const noexcept -> bool { return m_hasUuid; }
    [[nodiscard]] auto hasControllerGains() const noexcept -> bool { return m_hasControllerGains; }

protected:

    void parseBasicSensorData(DataStream& stream);
    void parseDockingIr(DataStream& stream);
    void parseInertialSensor(DataStream& stream);
    void parseGpio(DataStream& stream);
    void parseUdid(DataStream& stream);
    void parseControllerInfo(DataStream& stream);

    static void parseVersion(DataStream& stream, Version& version);

private:

    BasicSensors    m_basicSensors;
    DockingStation  m_dockingIr;
    Inertial        m_inertial;
    Version         m_hardwareVersion;
    Version         m_firmwareVersion;
    Gpio            m_gpio;
    Udid            m_udid;
    ControllerGains m_controllerGains;

    bool m_hasBasicSensors    : 1 {};
    bool m_hasDockingIr       : 1 {};
    bool m_hasInertial        : 1 {};
    bool m_hasHardwareVersion : 1 {};
    bool m_hasFirmwareVersion : 1 {};
    bool m_hasGpio            : 1 {};
    bool m_hasUuid            : 1 {};
    bool m_hasControllerGains : 1 {};
};

} // namespace reanaut
