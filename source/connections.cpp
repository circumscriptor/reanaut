#include "connections.hpp"
#include "kobuki.hpp"
#include "laser.hpp"

#include <boost/asio/buffer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/system/detail/error_code.hpp>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <print>
#include <span>
#include <string>
#include <vector>

namespace reanaut
{

KobukiConnection::KobukiConnection(boost::asio::io_context& ioContext, uint16_t port, const std::string& robotIp, uint16_t robotPort)
    : m_socket(ioContext, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)),
      m_remoteEndpoint(boost::asio::ip::make_address(robotIp), robotPort), m_recvBuffer(1024) // Allocate a 1KB buffer, ample for Kobuki packets
{
    std::println("kobuki_connection listening on port {}", port);
    std::println("kobuki_connection set to send commands to robot at {}", robotIp);
}

auto KobukiConnection::getLatestFeedback(Feedback& feedback) -> bool
{
    if (m_hasNewData) {
        feedback     = m_feedback;
        m_hasNewData = false; // Mark the data as "read"
        return true;
    }
    return false;
}

void KobukiConnection::asyncRecv()
{
    m_socket.async_receive_from(boost::asio::buffer(m_recvBuffer), m_remoteEndpoint,
                                [this](const boost::system::error_code& error, std::size_t bytesTransferred) { handleReceive(error, bytesTransferred); });
}

void KobukiConnection::asyncSend(const Command& command)
{
    command.buildPacket(m_sendBuffer);
    m_socket.async_send_to(boost::asio::buffer(m_sendBuffer), m_remoteEndpoint, [](const boost::system::error_code& error, std::size_t /*bytes_sent*/) {
        if (error) {
            std::println(stderr, "Send Error: {}", error.message());
        }
    });
}

void KobukiConnection::handleReceive(const boost::system::error_code& error, std::size_t bytesTransferred)
{
    if (error == boost::asio::error::operation_aborted) {
        std::println(stderr, "operation (async_receive_from) aborted0");
        return;
    }

    if (!error && bytesTransferred > 0) {
        if (m_feedback.parse({m_recvBuffer.begin(), std::next(m_recvBuffer.begin(), static_cast<ptrdiff_t>(bytesTransferred))})) {
            m_hasNewData = true;
        } else {
            std::println(stderr, "Warning: Malformed Kobuki packet received.");
        }
    }

    asyncRecv();
}

LaserConnection::LaserConnection(boost::asio::io_context& ioContext, uint16_t port, const std::string& robotIp, uint16_t robotPort)
    : m_socket(ioContext, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)),
      m_remoteEndpoint(boost::asio::ip::make_address(robotIp), robotPort),
      m_recvBuffer(sizeof(LaserScan) * kMaxLaserScans) // Allocate a 1KB buffer, ample for Kobuki packets
{
    std::println("Laser connection listening on port {}", port);
    std::println("Laser connection set to send commands to robot at {}", robotIp);

    uint8_t command = 0x00;
    m_socket.async_send_to(boost::asio::buffer(&command, 1), m_remoteEndpoint, [](const boost::system::error_code& error, std::size_t /*bytes_sent*/) {
        if (error) {
            std::println(stderr, "Send Error: {}", error.message());
        } else {
            std::println("SENT LASER COMMAND");
        }
    });
}

auto LaserConnection::getLatestSweep() const -> std::span<const LaserScan> { return m_scans; }

auto LaserConnection::getLatestSweep(std::vector<LaserScan>& scan) -> bool
{
    if (m_hasNewData) {
        scan         = m_scans;
        m_hasNewData = false;
        return true;
    }
    return false;
}

void LaserConnection::asyncRecv()
{
    m_socket.async_receive_from(boost::asio::buffer(m_recvBuffer), m_remoteEndpoint,
                                [this](const boost::system::error_code& error, std::size_t bytesTransferred) { handleReceive(error, bytesTransferred); });
}

void LaserConnection::handleReceive(const boost::system::error_code& error, std::size_t bytesTransferred)
{
    if (error == boost::asio::error::operation_aborted) {
        std::println(stderr, "operation (async_receive_from) aborted");
        return;
    }

    if (!error && bytesTransferred > 0) {
        const size_t count = bytesTransferred / sizeof(LaserScan);
        m_scans.resize(count);
        std::memcpy(m_scans.data(), m_recvBuffer.data(), bytesTransferred);
        m_hasNewData = true;
    }

    asyncRecv();
}

} // namespace reanaut
