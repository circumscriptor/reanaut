#include "data_stream.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

DataStream::DataStream(std::span<uint8_t> buffer) : m_stream{buffer} {}

auto DataStream::atEnd() const noexcept -> bool { return m_stream.empty(); }

void DataStream::skip(size_t count) { m_stream = m_stream.subspan(count); }

void DataStream::limit(size_t count) { m_stream = m_stream.subspan(0, count); }

auto DataStream::consume(size_t size) -> std::span<uint8_t>
{
    auto result = m_stream.subspan(0, size);
    m_stream    = m_stream.subspan(size);
    return result;
}
