/**
 * File Name: alu.cpp
 * Author: Vishank Singh
 * Updated for RISC-V Compliance (Fixed compilation errors)
 */

#include "vm/alu.h"
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <limits>

namespace alu {

// Define aliases for 128-bit types to handle __extension__ cleanly
__extension__ using int128_t = __int128;
__extension__ using uint128_t = unsigned __int128;

// Note: FCSR_* constants and Alu constructor are defined in the header.

// Execute Integer ALU Operations
[[nodiscard]] std::pair<uint64_t, bool> Alu::execute(AluOp op, uint64_t a, uint64_t b) {
  // Force 64-bit inputs
  a &= 0xFFFFFFFFFFFFFFFFULL;
  b &= 0xFFFFFFFFFFFFFFFFULL;

  switch (op) {
    case AluOp::kAdd: {
      uint64_t result = a + b;
      // Signed overflow check
      bool overflow = ((static_cast<int64_t>(a) > 0 && static_cast<int64_t>(b) > 0 && static_cast<int64_t>(result) < 0) ||
                       (static_cast<int64_t>(a) < 0 && static_cast<int64_t>(b) < 0 && static_cast<int64_t>(result) > 0));
      return {result, overflow};
    }
    case AluOp::kAddw: {
      int32_t sa = static_cast<int32_t>(a);
      int32_t sb = static_cast<int32_t>(b);
      int32_t result32 = sa + sb;
      bool overflow = ((sa > 0 && sb > 0 && result32 < 0) ||
                       (sa < 0 && sb < 0 && result32 > 0));
      return {static_cast<uint64_t>(static_cast<int64_t>(result32)), overflow};
    }
    case AluOp::kSub: {
      uint64_t result = a - b;
      bool overflow = ((static_cast<int64_t>(a) > 0 && static_cast<int64_t>(b) < 0 && static_cast<int64_t>(result) < 0) ||
                       (static_cast<int64_t>(a) < 0 && static_cast<int64_t>(b) > 0 && static_cast<int64_t>(result) > 0));
      return {result, overflow};
    }
    case AluOp::kSubw: {
      int32_t sa = static_cast<int32_t>(a);
      int32_t sb = static_cast<int32_t>(b);
      int32_t result32 = sa - sb;
      bool overflow = ((sa > 0 && sb < 0 && result32 < 0) ||
                       (sa < 0 && sb > 0 && result32 > 0));
      return {static_cast<uint64_t>(static_cast<int64_t>(result32)), overflow};
    }

    // Bitwise
    case AluOp::kAnd:  return {a & b, false};
    case AluOp::kOr:   return {a | b, false};
    case AluOp::kXor:  return {a ^ b, false};

    // Shifts
    case AluOp::kSll:  return {a << (b & 0x3F), false};
    case AluOp::kSrl:  return {a >> (b & 0x3F), false};
    case AluOp::kSra: {
      int64_t sa = static_cast<int64_t>(a);
      return {static_cast<uint64_t>(sa >> (b & 0x3F)), false};
    }
    case AluOp::kSllw: {
      int32_t sa = static_cast<int32_t>(a);
      int32_t result = sa << (b & 0x1F);
      return {static_cast<uint64_t>(static_cast<int64_t>(result)), false};
    }
    case AluOp::kSrlw: {
      uint32_t ua = static_cast<uint32_t>(a);
      uint32_t result = ua >> (b & 0x1F);
      return {static_cast<uint64_t>(static_cast<int64_t>(result)), false};
    }
    case AluOp::kSraw: {
      int32_t sa = static_cast<int32_t>(a);
      int32_t result = sa >> (b & 0x1F);
      return {static_cast<uint64_t>(static_cast<int64_t>(result)), false};
    }

    // Comparisons
    case AluOp::kSlt:
      return {static_cast<uint64_t>(static_cast<int64_t>(a) < static_cast<int64_t>(b)), false};
    case AluOp::kSltu:
      return {static_cast<uint64_t>(a < b), false};

    // Multiplication
    case AluOp::kMul: {
      auto sa = static_cast<int64_t>(a);
      auto sb = static_cast<int64_t>(b);
      int64_t result = sa * sb;
      bool overflow = __builtin_mul_overflow(sa, sb, &result);
      return {static_cast<uint64_t>(result), overflow};
    }
    case AluOp::kMulh: {
      auto sa = static_cast<int64_t>(a);
      auto sb = static_cast<int64_t>(b);
      int128_t wide = static_cast<int128_t>(sa) * static_cast<int128_t>(sb);
      return {static_cast<uint64_t>(wide >> 64), false};
    }
    case AluOp::kMulhsu: {
      auto sa = static_cast<int64_t>(a);
      auto ub = static_cast<uint64_t>(b);
      int128_t wide = static_cast<int128_t>(sa) * static_cast<int128_t>(ub);
      return {static_cast<uint64_t>(wide >> 64), false};
    }
    case AluOp::kMulhu: {
      auto ua = static_cast<uint64_t>(a);
      auto ub = static_cast<uint64_t>(b);
      uint128_t wide = static_cast<uint128_t>(ua) * static_cast<uint128_t>(ub);
      return {static_cast<uint64_t>(wide >> 64), false};
    }
    case AluOp::kMulw: {
      auto sa = static_cast<int32_t>(a);
      auto sb = static_cast<int32_t>(b);
      int64_t result = static_cast<int64_t>(sa) * static_cast<int64_t>(sb);
      return {static_cast<uint64_t>(static_cast<int32_t>(result)), false};
    }

    // Division
    case AluOp::kDiv: {
      auto sa = static_cast<int64_t>(a);
      auto sb = static_cast<int64_t>(b);
      if (sb == 0) return {static_cast<uint64_t>(-1), true};
      if (sa == std::numeric_limits<int64_t>::min() && sb == -1) return {static_cast<uint64_t>(sa), true};
      return {static_cast<uint64_t>(sa / sb), false};
    }
    case AluOp::kDivu: {
      if (b == 0) return {static_cast<uint64_t>(-1), true};
      return {a / b, false};
    }
    case AluOp::kDivw: {
      auto sa = static_cast<int32_t>(a);
      auto sb = static_cast<int32_t>(b);
      if (sb == 0) return {static_cast<uint64_t>(-1), true};
      if (sa == std::numeric_limits<int32_t>::min() && sb == -1) return {static_cast<uint64_t>(sa), true};
      return {static_cast<uint64_t>(static_cast<int64_t>(sa / sb)), false};
    }
    case AluOp::kDivuw: {
      auto ua = static_cast<uint32_t>(a);
      auto ub = static_cast<uint32_t>(b);
      if (ub == 0) return {static_cast<uint64_t>(-1), true};
      return {static_cast<uint64_t>(static_cast<int64_t>(ua / ub)), false};
    }

    // Remainder
    case AluOp::kRem: {
      auto sa = static_cast<int64_t>(a);
      auto sb = static_cast<int64_t>(b);
      if (sb == 0) return {static_cast<uint64_t>(sa), true};
      if (sa == std::numeric_limits<int64_t>::min() && sb == -1) return {0, false};
      return {static_cast<uint64_t>(sa % sb), false};
    }
    case AluOp::kRemu: {
      if (b == 0) return {a, true};
      return {a % b, false};
    }
    case AluOp::kRemw: {
      auto sa = static_cast<int32_t>(a);
      auto sb = static_cast<int32_t>(b);
      if (sb == 0) return {static_cast<uint64_t>(sa), true};
      if (sa == std::numeric_limits<int32_t>::min() && sb == -1) return {0, false};
      return {static_cast<uint64_t>(static_cast<int64_t>(sa % sb)), false};
    }
    case AluOp::kRemuw: {
      auto ua = static_cast<uint32_t>(a);
      auto ub = static_cast<uint32_t>(b);
      if (ub == 0) return {static_cast<uint64_t>(ua), true};
      return {static_cast<uint64_t>(static_cast<int64_t>(ua % ub)), false};
    }

    default: return {0, false};
  }
}

// Execute Single-Precision Floating Point (RV64F)
[[nodiscard]] std::pair<uint64_t, uint8_t>
Alu::fpexecute(AluOp op, uint64_t ina, uint64_t inb, uint64_t inc, uint8_t rm) {
  float a, b, c;
  // Read only the lower 32-bits for single-precision
  std::memcpy(&a, &ina, sizeof(float));
  std::memcpy(&b, &inb, sizeof(float));
  std::memcpy(&c, &inc, sizeof(float));

  float result = 0.0f;
  uint8_t fcsr = 0;

  int original_rm = std::fegetround();
  switch (rm) {
    case 0: std::fesetround(FE_TONEAREST); break;
    case 1: std::fesetround(FE_TOWARDZERO); break;
    case 2: std::fesetround(FE_DOWNWARD); break;
    case 3: std::fesetround(FE_UPWARD); break;
    default: break;
  }

  std::feclearexcept(FE_ALL_EXCEPT);

  switch (op) {
    case AluOp::kFmadd_s:  result = std::fma(a, b, c); break;
    case AluOp::kFmsub_s:  result = std::fma(a, b, -c); break;
    case AluOp::kFnmadd_s: result = std::fma(-a, b, -c); break;
    case AluOp::kFnmsub_s: result = std::fma(-a, b, c); break;
    case AluOp::FADD_S:    result = a + b; break;
    case AluOp::FSUB_S:    result = a - b; break;
    case AluOp::FMUL_S:    result = a * b; break;

    case AluOp::FDIV_S:
      if (b == 0.0f) {
        result = std::numeric_limits<float>::quiet_NaN();
        fcsr |= FCSR_DIV_BY_ZERO;
      } else {
        result = a / b;
      }
      break;

    case AluOp::FSQRT_S:
      if (a < 0.0f) {
        result = std::numeric_limits<float>::quiet_NaN();
        fcsr |= FCSR_INVALID_OP;
      } else {
        result = std::sqrt(a);
      }
      break;

    // Conversions: Float -> Int
    case AluOp::FCVT_W_S:
    case AluOp::FCVT_WU_S:
    case AluOp::FCVT_L_S:
    case AluOp::FCVT_LU_S: {
      std::fesetround(original_rm);
      if (op == AluOp::FCVT_W_S) {
        if (!std::isfinite(a) || a > static_cast<float>(INT32_MAX) || a < static_cast<float>(INT32_MIN)) {
          fcsr |= FCSR_INVALID_OP;
          int32_t clipped = (a > 0) ? INT32_MAX : INT32_MIN;
          return {static_cast<uint64_t>(static_cast<int64_t>(clipped)), fcsr};
        }
        return {static_cast<uint64_t>(static_cast<int64_t>(std::nearbyint(a))), fcsr};
      }
      int64_t res_int = 0;
      if (op == AluOp::FCVT_WU_S) res_int = static_cast<int64_t>(static_cast<uint32_t>(std::nearbyint(a)));
      else if (op == AluOp::FCVT_L_S) res_int = static_cast<int64_t>(std::nearbyint(a));
      else res_int = static_cast<int64_t>(static_cast<uint64_t>(std::nearbyint(a)));
      return {static_cast<uint64_t>(res_int), fcsr};
    }

    // Conversions: Int -> Float
    case AluOp::FCVT_S_W:  result = static_cast<float>(static_cast<int32_t>(ina)); break;
    case AluOp::FCVT_S_WU: result = static_cast<float>(static_cast<uint32_t>(ina)); break;
    case AluOp::FCVT_S_L:  result = static_cast<float>(static_cast<int64_t>(ina)); break;
    case AluOp::FCVT_S_LU: result = static_cast<float>(static_cast<uint64_t>(ina)); break;

    case AluOp::FSGNJ_S: {
      uint32_t ia = static_cast<uint32_t>(ina), ib = static_cast<uint32_t>(inb);
      uint32_t res = (ia & 0x7FFFFFFF) | (ib & 0x80000000);
      std::memcpy(&result, &res, sizeof(float));
      break;
    }
    case AluOp::FSGNJN_S: {
      uint32_t ia = static_cast<uint32_t>(ina), ib = static_cast<uint32_t>(inb);
      uint32_t res = (ia & 0x7FFFFFFF) | (~ib & 0x80000000);
      std::memcpy(&result, &res, sizeof(float));
      break;
    }
    case AluOp::FSGNJX_S: {
      uint32_t ia = static_cast<uint32_t>(ina), ib = static_cast<uint32_t>(inb);
      uint32_t res = (ia & 0x7FFFFFFF) | ((ia ^ ib) & 0x80000000);
      std::memcpy(&result, &res, sizeof(float));
      break;
    }

    case AluOp::FMIN_S:
      if (std::isnan(a) && std::isnan(b)) result = std::numeric_limits<float>::quiet_NaN();
      else if (std::isnan(a)) result = b;
      else if (std::isnan(b)) result = a;
      else if (std::signbit(a) != std::signbit(b) && a == b) result = -0.0f;
      else result = std::fmin(a, b);
      break;

    case AluOp::FMAX_S:
      if (std::isnan(a) && std::isnan(b)) result = std::numeric_limits<float>::quiet_NaN();
      else if (std::isnan(a)) result = b;
      else if (std::isnan(b)) result = a;
      else if (std::signbit(a) != std::signbit(b) && a == b) result = 0.0f;
      else result = std::fmax(a, b);
      break;

    case AluOp::FEQ_S: std::fesetround(original_rm); return {(a == b) ? 1ULL : 0ULL, fcsr};
    case AluOp::FLT_S: std::fesetround(original_rm); return {(a < b) ? 1ULL : 0ULL, fcsr};
    case AluOp::FLE_S: std::fesetround(original_rm); return {(a <= b) ? 1ULL : 0ULL, fcsr};

    case AluOp::FCLASS_S: {
      uint32_t bits = static_cast<uint32_t>(ina);
      float f;
      std::memcpy(&f, &bits, sizeof(float));
      uint16_t res = 0;
      if (std::isinf(f)) res |= std::signbit(f) ? 1 : 128;
      else if (std::fpclassify(f) == FP_NORMAL) res |= std::signbit(f) ? 2 : 64;
      else if (std::fpclassify(f) == FP_SUBNORMAL) res |= std::signbit(f) ? 4 : 32;
      else if (f == 0.0f) res |= std::signbit(f) ? 8 : 16;
      else if (std::isnan(f)) res |= (bits & 0x00400000) ? 512 : 256;
      std::fesetround(original_rm);
      return {res, fcsr};
    }

    // Double to Single
    case AluOp::FCVT_S_D: {
      double d;
      std::memcpy(&d, &ina, sizeof(double));
      float res_float = static_cast<float>(d);
      uint32_t raw;
      std::memcpy(&raw, &res_float, sizeof(float));
      std::fesetround(original_rm);
      return {static_cast<uint64_t>(raw), fcsr};
    }

    default: break;
  }

  int raised = std::fetestexcept(FE_ALL_EXCEPT);
  if (raised & FE_INVALID)   fcsr |= FCSR_INVALID_OP;
  if (raised & FE_DIVBYZERO) fcsr |= FCSR_DIV_BY_ZERO;
  if (raised & FE_OVERFLOW)  fcsr |= FCSR_OVERFLOW;
  if (raised & FE_UNDERFLOW) fcsr |= FCSR_UNDERFLOW;
  if (raised & FE_INEXACT)   fcsr |= FCSR_INEXACT;

  std::fesetround(original_rm);

  // Zero-Extension for Single Precision
  uint32_t result_bits = 0;
  std::memcpy(&result_bits, &result, sizeof(result));
  return {static_cast<uint64_t>(result_bits), fcsr};
}

// Execute Double-Precision Floating Point (RV64D)
[[nodiscard]] std::pair<uint64_t, uint8_t>
Alu::dfpexecute(AluOp op, uint64_t ina, uint64_t inb, uint64_t inc, uint8_t rm) {
  double a, b, c;
  std::memcpy(&a, &ina, sizeof(double));
  std::memcpy(&b, &inb, sizeof(double));
  std::memcpy(&c, &inc, sizeof(double));

  double result = 0.0;
  uint8_t fcsr = 0;

  int original_rm = std::fegetround();
  switch (rm) {
    case 0: std::fesetround(FE_TONEAREST); break;
    case 1: std::fesetround(FE_TOWARDZERO); break;
    case 2: std::fesetround(FE_DOWNWARD); break;
    case 3: std::fesetround(FE_UPWARD); break;
    default: break;
  }

  std::feclearexcept(FE_ALL_EXCEPT);

  switch (op) {
    // FMA (double)
    case AluOp::FMADD_D:  result = std::fma(a, b, c); break;
    case AluOp::FMSUB_D:  result = std::fma(a, b, -c); break;
    case AluOp::FNMADD_D: result = std::fma(-a, b, -c); break;
    case AluOp::FNMSUB_D: result = std::fma(-a, b, c); break;

    case AluOp::FADD_D:   result = a + b; break;
    case AluOp::FSUB_D:   result = a - b; break;
    case AluOp::FMUL_D:   result = a * b; break;
    case AluOp::FDIV_D:
      if (b == 0.0) {
        result = std::numeric_limits<double>::quiet_NaN();
        fcsr |= FCSR_DIV_BY_ZERO;
      } else {
        result = a / b;
      }
      break;
    case AluOp::FSQRT_D:
      if (a < 0.0) {
        result = std::numeric_limits<double>::quiet_NaN();
        fcsr |= FCSR_INVALID_OP;
      } else {
        result = std::sqrt(a);
      }
      break;

    // Double -> Int
    case AluOp::FCVT_W_D:
    case AluOp::FCVT_WU_D:
    case AluOp::FCVT_L_D:
    case AluOp::FCVT_LU_D: {
      std::fesetround(original_rm);
      if (op == AluOp::FCVT_W_D) {
        if (!std::isfinite(a) || a > static_cast<double>(INT32_MAX) || a < static_cast<double>(INT32_MIN)) {
          fcsr |= FCSR_INVALID_OP;
          int32_t clipped = (a > 0) ? INT32_MAX : INT32_MIN;
          return {static_cast<uint64_t>(static_cast<int64_t>(clipped)), fcsr};
        }
        return {static_cast<uint64_t>(static_cast<int64_t>(std::nearbyint(a))), fcsr};
      }
      int64_t res_int = 0;
      if (op == AluOp::FCVT_WU_D) res_int = static_cast<int64_t>(static_cast<uint32_t>(std::nearbyint(a)));
      else if (op == AluOp::FCVT_L_D) res_int = static_cast<int64_t>(std::nearbyint(a));
      else res_int = static_cast<int64_t>(static_cast<uint64_t>(std::nearbyint(a)));
      return {static_cast<uint64_t>(res_int), fcsr};
    }

    // Int -> Double
    case AluOp::FCVT_D_W:  result = static_cast<double>(static_cast<int32_t>(ina)); break;
    case AluOp::FCVT_D_WU: result = static_cast<double>(static_cast<uint32_t>(ina)); break;
    case AluOp::FCVT_D_L:  result = static_cast<double>(static_cast<int64_t>(ina)); break;
    case AluOp::FCVT_D_LU: result = static_cast<double>(static_cast<uint64_t>(ina)); break;

    case AluOp::FSGNJ_D: {
      uint64_t ia = ina, ib = inb;
      uint64_t res = (ia & 0x7FFFFFFFFFFFFFFFULL) | (ib & 0x8000000000000000ULL);
      std::memcpy(&result, &res, sizeof(double));
      break;
    }
    case AluOp::FSGNJN_D: {
      uint64_t ia = ina, ib = inb;
      uint64_t res = (ia & 0x7FFFFFFFFFFFFFFFULL) | (~ib & 0x8000000000000000ULL);
      std::memcpy(&result, &res, sizeof(double));
      break;
    }
    case AluOp::FSGNJX_D: {
      uint64_t ia = ina, ib = inb;
      uint64_t res = (ia & 0x7FFFFFFFFFFFFFFFULL) | ((ia ^ ib) & 0x8000000000000000ULL);
      std::memcpy(&result, &res, sizeof(double));
      break;
    }

    case AluOp::FMIN_D:
      if (std::isnan(a) && std::isnan(b)) result = std::numeric_limits<double>::quiet_NaN();
      else if (std::isnan(a)) result = b;
      else if (std::isnan(b)) result = a;
      else if (std::signbit(a) != std::signbit(b) && a == b) result = -0.0;
      else result = std::fmin(a, b);
      break;

    case AluOp::FMAX_D:
      if (std::isnan(a) && std::isnan(b)) result = std::numeric_limits<double>::quiet_NaN();
      else if (std::isnan(a)) result = b;
      else if (std::isnan(b)) result = a;
      else if (std::signbit(a) != std::signbit(b) && a == b) result = 0.0;
      else result = std::fmax(a, b);
      break;

    case AluOp::FEQ_D: std::fesetround(original_rm); return {(a == b) ? 1ULL : 0ULL, fcsr};
    case AluOp::FLT_D: std::fesetround(original_rm); return {(a < b) ? 1ULL : 0ULL, fcsr};
    case AluOp::FLE_D: std::fesetround(original_rm); return {(a <= b) ? 1ULL : 0ULL, fcsr};

    case AluOp::FCLASS_D: {
      uint64_t bits = ina;
      double d;
      std::memcpy(&d, &bits, sizeof(double));
      uint16_t res = 0;
      if (std::isinf(d)) res |= std::signbit(d) ? 1 : 128;
      else if (std::fpclassify(d) == FP_NORMAL) res |= std::signbit(d) ? 2 : 64;
      else if (std::fpclassify(d) == FP_SUBNORMAL) res |= std::signbit(d) ? 4 : 32;
      else if (d == 0.0) res |= std::signbit(d) ? 8 : 16;
      else if (std::isnan(d)) res |= (bits & (1ULL << 51)) ? 512 : 256;
      std::fesetround(original_rm);
      return {res, fcsr};
    }

    // Single to Double
    case AluOp::FCVT_D_S: {
      uint32_t raw_input = static_cast<uint32_t>(ina);
      float f;
      std::memcpy(&f, &raw_input, sizeof(float));
      result = static_cast<double>(f);
      break;
    }

    default: break;
  }

  int raised = std::fetestexcept(FE_ALL_EXCEPT);
  if (raised & FE_INVALID)   fcsr |= FCSR_INVALID_OP;
  if (raised & FE_DIVBYZERO) fcsr |= FCSR_DIV_BY_ZERO;
  if (raised & FE_OVERFLOW)  fcsr |= FCSR_OVERFLOW;
  if (raised & FE_UNDERFLOW) fcsr |= FCSR_UNDERFLOW;
  if (raised & FE_INEXACT)   fcsr |= FCSR_INEXACT;

  std::fesetround(original_rm);

  uint64_t result_bits = 0;
  std::memcpy(&result_bits, &result, sizeof(result));
  return {result_bits, fcsr};
}

void Alu::setFlags(bool carry, bool zero, bool negative, bool overflow) {
  carry_ = carry;
  zero_ = zero;
  negative_ = negative;
  overflow_ = overflow;
}

} // namespace alu
