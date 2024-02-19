#pragma once

#include "package_defs.hpp"

#include <cstdint>
#include <cstddef>


NS_HEAD



namespace primitives {

using   byte =      uint8_t;  //!<  this covers the ROSmessage definition of a byte
using   word =      uint16_t;

typedef uint8_t     u8;
typedef uint16_t    u16;
typedef uint32_t    u32;
typedef int8_t      s8;
typedef int16_t     s16;
typedef int32_t     s32;
typedef float       f32;
typedef double      f64;

template <typename T>
T revPrimitive(const T u)
{
    //static_assert (CHAR_BIT == 8, "CHAR_BIT != 8");

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

/*!
 * \brief The BigEndianPrimative struct represents a big endian version
 * of various primitives.  Also includes overloaded casting operators
 * so it can be assigned to standard primatives with minimal extra syntax.
 */
template <typename PrimT>
struct BigEndianPrimative{
  PrimT raw;
  /*!
   * \brief returns the machine-endian version of the variable in
   * the specified (PrimT) type.
   * \return a machine endian version fo the variable
   */
  PrimT get() const{
    return revPrimitive(raw);
  }
  void set(PrimT val){
    raw = revPrimitive(val);
  }

  operator int8_t() const { return get(); }
  operator int16_t() const { return get(); }
  operator int32_t() const { return get(); }

  operator uint8_t() const { return get(); }
  operator uint16_t() const { return get(); }
  operator uint32_t() const { return get(); }

  operator float() const {return get(); }
  operator double() const {return get(); }
};

typedef BigEndianPrimative<u8>      BE_u8;
typedef BigEndianPrimative<u16>     BE_u16;
typedef BigEndianPrimative<u32>     BE_u32;
typedef BigEndianPrimative<s8>      BE_s8;
typedef BigEndianPrimative<s16>     BE_s16;
typedef BigEndianPrimative<s32>     BE_s32;
typedef BigEndianPrimative<f32>     BE_f32;
typedef BigEndianPrimative<f64>     BE_f64;

}
using namespace primitives;

NS_FOOT
