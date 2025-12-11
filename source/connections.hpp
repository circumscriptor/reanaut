#pragma once

#include "kobuki.hpp"
#include "laser.hpp"

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/system/detail/error_code.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace reanaut
{

class KobukiConnection
{
public:

    KobukiConnection(boost::asio::io_context& ioContext, uint16_t port, const std::string& robotIp, uint16_t robotPort);

    auto getLatestFeedback(Feedback& feedback) -> bool;
    void asyncRecv();
    void asyncSend(const Command& command);

protected:

    void handleReceive(const boost::system::error_code& error, std::size_t bytesTransferred);

private:

    boost::asio::ip::udp::socket   m_socket;
    boost::asio::ip::udp::endpoint m_remoteEndpoint;
    std::vector<uint8_t>           m_recvBuffer;
    std::vector<uint8_t>           m_sendBuffer;
    Feedback                       m_feedback{};
    bool                           m_hasNewData{};
};

class LaserConnection
{
public:

    LaserConnection(boost::asio::io_context& ioContext, uint16_t port, const std::string& robotIp, uint16_t robotPort);

    [[nodiscard]]
    auto getLastSweep() const -> std::span<const LaserScan>;
    auto getLatestSweep(std::vector<LaserScan>& scan) -> bool;
    void asyncRecv();

protected:

    void handleReceive(const boost::system::error_code& error, std::size_t bytesTransferred);

private:

    boost::asio::ip::udp::socket   m_socket;
    boost::asio::ip::udp::endpoint m_remoteEndpoint;
    std::vector<uint8_t>           m_recvBuffer;
    std::vector<LaserScan>         m_scans;
    bool                           m_hasNewData{};
};

} // namespace reanaut
