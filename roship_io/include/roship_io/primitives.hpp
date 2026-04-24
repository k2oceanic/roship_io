#pragma once

#include "package_defs.hpp"

#include <cstdint>
#include <cstddef>

ROSHIP_IO_NS_HEAD

/**
 * @brief Fundamental type aliases and byte-order utilities used throughout roship_io.
 */
namespace primitives {

using byte = uint8_t;   ///< Byte type, compatible with the `io_interfaces::msg::RawPacket` `data` field.
using word = uint16_t;  ///< 16-bit word type, used as the Modbus register unit.

typedef uint8_t  u8;  ///< Unsigned 8-bit integer.
typedef uint16_t u16; ///< Unsigned 16-bit integer.
typedef uint32_t u32; ///< Unsigned 32-bit integer.
typedef int8_t   s8;  ///< Signed 8-bit integer.
typedef int16_t  s16; ///< Signed 16-bit integer.
typedef int32_t  s32; ///< Signed 32-bit integer.
typedef float    f32; ///< 32-bit IEEE 754 float.
typedef double   f64; ///< 64-bit IEEE 754 double.

/**
 * @brief Reverse the byte order of primitive.
 */
template <typename T>
T revPrimitive(const T u)
{
    union
    {
        T u;
        unsigned char u8[sizeof(T)];
    } source, dest;

    source.u = u;

    for (size_t k = 0; k < sizeof(T); k++)
        dest.u8[k] = source.u8[sizeof(T) - k - 1];

    return dest.u;
}

/**
 * @brief A big-endian primitive stored as a single raw value with byte-swap accessors.
 *
 */
template <typename PrimT>
struct BigEndianPrimative {
  PrimT raw; ///< Raw big-endian storage.

  /**
   * @brief Return the machine-endian value.
   * @return Native (host) endian value.
   */
  PrimT get() const { return revPrimitive(raw); }

  /**
   * @brief Store a machine-endian value in big-endian byte order.
   * @param val Native value to store.
   */
  void set(PrimT val) { raw = revPrimitive(val); }

  operator int8_t()   const { return get(); } ///< Implicit cast to int8_t.
  operator int16_t()  const { return get(); } ///< Implicit cast to int16_t.
  operator int32_t()  const { return get(); } ///< Implicit cast to int32_t.

  operator uint8_t()  const { return get(); } ///< Implicit cast to uint8_t.
  operator uint16_t() const { return get(); } ///< Implicit cast to uint16_t.
  operator uint32_t() const { return get(); } ///< Implicit cast to uint32_t.

  operator float()  const { return get(); }   ///< Implicit cast to float.
  operator double() const { return get(); }   ///< Implicit cast to double.
};

typedef BigEndianPrimative<u8>  BE_u8;  ///< Big-endian unsigned 8-bit integer.
typedef BigEndianPrimative<u16> BE_u16; ///< Big-endian unsigned 16-bit integer.
typedef BigEndianPrimative<u32> BE_u32; ///< Big-endian unsigned 32-bit integer.
typedef BigEndianPrimative<s8>  BE_s8;  ///< Big-endian signed 8-bit integer.
typedef BigEndianPrimative<s16> BE_s16; ///< Big-endian signed 16-bit integer.
typedef BigEndianPrimative<s32> BE_s32; ///< Big-endian signed 32-bit integer.
typedef BigEndianPrimative<f32> BE_f32; ///< Big-endian 32-bit float.
typedef BigEndianPrimative<f64> BE_f64; ///< Big-endian 64-bit double.

}
using namespace primitives;

NS_FOOT
