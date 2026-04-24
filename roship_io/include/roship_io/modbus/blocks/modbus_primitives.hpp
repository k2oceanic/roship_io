#pragma once

#include "../modbus_defs.hpp"
#include "../../primitives.hpp"

#include <cstdint>
#include <stddef.h>

ROSHIP_IO_NS_HEAD
namespace primitives {

/** @brief Pointer-to-word alias used as the Modbus register buffer type. */
typedef word* WordBuffer;

/**
 * @brief A big-endian primitive stored as an array of 16-bit Modbus words.
 */
template <typename PrimT>
struct BigEndianPrimativeByWord {
  union prim_union {
    PrimT u;
    uint16_t words[sizeof(PrimT)/2];
  } raw; ///< Raw word-order storage for the primitive value.

  /**
   * @brief Return the native machine-endian value.
   *
   * Reverses the word order in `raw` to produce the correct host-endian value.
   *
   * @return Machine-endian representation of the stored value.
   */
  PrimT get() const {
    prim_union output = raw;
    auto words = sizeof(PrimT)/2;
    for (int i = 0; i < words; i++) {
      output.words[i] = raw.words[words - 1 - i];
    }
    return output.u;
  }

  /**
   * @brief Store a machine-endian value in word-swapped big-endian order.
   *
   * @param val Native value to store.
   */
  void set(PrimT val) {
    prim_union input;
    input.u = val;
    auto words = sizeof(PrimT)/2;
    for (int i = 0; i < words; i++) {
      raw.words[i] = input.words[words - 1 - i];
    }
  }

  operator int8_t()  const { return get(); }  ///< Implicit cast to int8_t.
  operator int16_t() const { return get(); }  ///< Implicit cast to int16_t.
  operator int32_t() const { return get(); }  ///< Implicit cast to int32_t.

  operator uint8_t()  const { return get(); } ///< Implicit cast to uint8_t.
  operator uint16_t() const { return get(); } ///< Implicit cast to uint16_t.
  operator uint32_t() const { return get(); } ///< Implicit cast to uint32_t.

  operator float()  const { return get(); }   ///< Implicit cast to float.
  operator double() const { return get(); }   ///< Implicit cast to double.

  /**
   * @brief Assignment from any compatible type T.
   * @tparam T Source numeric type.
   * @param other Value to store (will be word-swapped).
   */
  template <typename T>
  BigEndianPrimativeByWord& operator=(T other) { this->set(other); }
};

typedef BigEndianPrimativeByWord<u32> BE_WORD_u32; ///< Big-endian 32-bit unsigned integer (2 words).
typedef BigEndianPrimativeByWord<s8>  BE_WORD_s8;  ///< Big-endian signed 8-bit integer.
typedef BigEndianPrimativeByWord<s16> BE_WORD_s16; ///< Big-endian signed 16-bit integer (1 word).
typedef BigEndianPrimativeByWord<s32> BE_WORD_s32; ///< Big-endian signed 32-bit integer (2 words).
typedef BigEndianPrimativeByWord<f32> BE_WORD_f32; ///< Big-endian 32-bit float (2 words).
typedef BigEndianPrimativeByWord<f64> BE_WORD_f64; ///< Big-endian 64-bit double (4 words).

}
NS_FOOT
