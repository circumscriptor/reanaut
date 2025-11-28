#pragma once

#include <boost/endian/conversion.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

class DataStream
{
public:

    explicit DataStream(std::span<uint8_t> buffer);

    [[nodiscard]]
    auto atEnd() const noexcept -> bool;
    void skip(size_t count);
    void limit(size_t count);

    template <typename T>
    void operator>>(T& output);

    template <typename Type>
    [[nodiscard]]
    auto read() -> Type;

protected:

    auto consume(size_t size) -> std::span<uint8_t>;

private:

    std::span<uint8_t> m_stream;
};

template <typename Type>
inline auto DataStream::read() -> Type
{
    auto buffer = consume(sizeof(Type));
    Type value{};
    std::memcpy(&value, buffer.data(), buffer.size());
    return boost::endian::little_to_native(value);
}

template <typename T>
inline void DataStream::operator>>(T& output)
{
    output = read<T>();
}
