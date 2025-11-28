#include "constants.hpp"
#include "data_stream.hpp"
#include "kobuki.hpp"

#include <boost/endian.hpp>
#include <boost/endian/conversion.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <span>
#include <vector>

namespace reanaut
{

auto Command::baseControl(uint16_t speed, uint16_t radius) -> Command&
{
    append(BaseControl);
    append(kBaseControlSize);
    append(speed);
    append(radius);
    return *this;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
auto Command::sound(uint16_t note, uint8_t duration) -> Command&
{
    append(Sound);
    append(kSoundSize);
    append(note);
    append(duration);
    return *this;
}

auto Command::soundSequence(SoundSequenceType sequence) -> Command&
{
    append(SoundSequence);
    append(kSoundSequenceSize);
    append(sequence);
    return *this;
}

auto Command::requestExtra(uint16_t flags) -> Command&
{
    append(RequestExtra);
    append(kRequestExtraSize);
    append(flags);
    return *this;
}

auto Command::generalPurposeOutput(uint16_t flags) -> Command&
{
    append(GeneralPurposeOutput);
    append(kGeneralPurposeOutputSize);
    append(flags);
    return *this;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
auto Command::setControllerGain(uint8_t type, uint32_t kp, uint32_t ki, uint32_t kd) -> Command&
{
    append(SetControllerGain);
    append(kSetControllerGainSize);
    append(type);
    append(kp);
    append(ki);
    append(kd);
    return *this;
}

auto Command::getControllerGain() -> Command&
{
    append(GetControllerGain);
    append(kGetControllerGainSize);
    append(kGetControllerGainSize, 0);
    return *this;
}

auto Command::buildPacket() const -> std::vector<uint8_t>
{
    std::vector<uint8_t> packet;
    buildPacket(m_payload, packet);
    return packet;
}

void Command::buildPacket(std::vector<uint8_t>& out) const { buildPacket(m_payload, out); }

void Command::reset() { m_payload.clear(); }

auto Command::size() -> size_t { return m_payload.size(); }

void Command::buildPacket(const std::vector<uint8_t>& payload, std::vector<uint8_t>& out)
{
    if (payload.size() > kMaskU8) {
        std::cerr << "Payload size exceeds 255 bytes. Packet will be malformed.\n";
    }

    out.clear();
    out.reserve(payload.size() + 4);

    out.emplace_back(kMagicByte1);
    out.emplace_back(kMagicByte2);
    out.emplace_back(static_cast<uint8_t>(payload.size()));
    out.insert(out.end(), payload.begin(), payload.end());

    uint8_t checksum = 0;
    for (size_t i = 2; i < out.size(); ++i) {
        checksum ^= static_cast<uint8_t>(out.at(i));
    }

    out.emplace_back(checksum);
}

void Command::append(uint8_t value) { m_payload.emplace_back(value); }

void Command::append(size_t count, uint8_t value) { m_payload.insert(m_payload.end(), count, value); }

void Command::append(uint16_t value)
{
    m_payload.emplace_back(static_cast<uint8_t>(value & kMaskU8));
    m_payload.emplace_back(static_cast<uint8_t>(value >> 8U));
}

void Command::append(uint32_t value)
{
    m_payload.emplace_back(static_cast<uint8_t>((value >> kOffsetByte1) & kMaskU8));
    m_payload.emplace_back(static_cast<uint8_t>((value >> kOffsetByte2) & kMaskU8));
    m_payload.emplace_back(static_cast<uint8_t>((value >> kOffsetByte3) & kMaskU8));
    m_payload.emplace_back(static_cast<uint8_t>((value >> kOffsetByte4) & kMaskU8));
}

auto Feedback::parse(std::span<std::uint8_t> packet) -> bool
{
    uint8_t checksum{};
    uint8_t size{};

    // if (packet.size() < 4) {
    //     qWarning() << "Packet too small: " << packet.size();
    //     return false;
    // }

    // if (static_cast<uint8_t>(packet.at(0)) != 0xAA || static_cast<uint8_t>(packet.at(1)) != 0x55) {
    //     qWarning() << "Invalid header";
    //     return false;
    // }

    // NOTE: This is does not use official protocol

    // Checksum validation

    DataStream stream(packet);
    stream >> size;

    for (size_t i = 0; i < static_cast<size_t>(size) + 2; ++i) {
        checksum ^= packet[i];
    }

    if (checksum != 0) {
        return false;
    }

    // if (checksum != static_cast<uint8_t>(packet.back())) {
    //     qWarning() << "Invalid checksum";
    //     return false;
    // }

    stream.limit(size);
    while (!stream.atEnd()) {
        uint8_t headerId{};
        uint8_t length{};

        stream >> headerId;
        stream >> length;

        switch (headerId) {
            case BasicSensorData:
                parseBasicSensorData(stream);
                break;
            case DockingIr:
                parseDockingIr(stream);
                break;
            case InertialSensor:
                parseInertialSensor(stream);
                break;
            case HardwareVersion:
                parseVersion(stream, m_hardwareVersion);
                m_hasHardwareVersion = true;
                break;
            case FirmwareVersion:
                parseVersion(stream, m_firmwareVersion);
                m_hasFirmwareVersion = true;
                break;
            case GeneralPurposeInput:
                parseGpio(stream);
                break;
            case UniqueDeviceId:
                parseUdid(stream);
                break;
            case ControllerInfo:
                parseControllerInfo(stream);
                break;
            default:
                stream.skip(length);
                break;
        }
    }
    return true;
}

void Feedback::resetFlags()
{
    m_hasBasicSensors    = false;
    m_hasDockingIr       = false;
    m_hasInertial        = false;
    m_hasHardwareVersion = false;
    m_hasFirmwareVersion = false;
    m_hasGpio            = false;
    m_hasUuid            = false;
    m_hasControllerGains = false;
}

void Feedback::parseBasicSensorData(DataStream& stream)
{
    stream >> m_basicSensors.timestamp;
    stream >> m_basicSensors.bumper;
    stream >> m_basicSensors.wheelDrop;
    stream >> m_basicSensors.cliff;
    stream >> m_basicSensors.leftEncoder;
    stream >> m_basicSensors.rightEncoder;
    stream >> m_basicSensors.leftPwm;
    stream >> m_basicSensors.rightPwm;
    stream >> m_basicSensors.buttons;
    stream >> m_basicSensors.charger;
    stream >> m_basicSensors.battery;
    stream >> m_basicSensors.overcurrentFlags;
    m_hasBasicSensors = true;
}

void Feedback::parseDockingIr(DataStream& stream)
{
    stream >> m_dockingIr.rightSignal;
    stream >> m_dockingIr.centerSignal;
    stream >> m_dockingIr.leftSignal;
    m_hasDockingIr = true;
}

void Feedback::parseInertialSensor(DataStream& stream)
{
    stream >> m_inertial.angle;
    stream >> m_inertial.angleRate;
    stream.skip(3);
    m_hasInertial = true;
}

void Feedback::parseGpio(DataStream& stream)
{
    stream >> m_gpio.digitalInput;
    stream >> m_gpio.analogInputCh0;
    stream >> m_gpio.analogInputCh1;
    stream >> m_gpio.analogInputCh2;
    stream >> m_gpio.analogInputCh3;
    stream.skip(6); // NOLINT
    m_hasGpio = true;
}

void Feedback::parseUdid(DataStream& stream)
{
    stream >> m_udid.udid0;
    stream >> m_udid.udid1;
    stream >> m_udid.udid2;
    m_hasUuid = true;
}

void Feedback::parseControllerInfo(DataStream& stream)
{
    stream >> m_controllerGains.type;
    stream >> m_controllerGains.p;
    stream >> m_controllerGains.i;
    stream >> m_controllerGains.d;
    m_hasControllerGains = true;
}

void Feedback::parseVersion(DataStream& stream, Version& version)
{
    stream >> version.patch;
    stream >> version.minor;
    stream >> version.major;
}

} // namespace reanaut
