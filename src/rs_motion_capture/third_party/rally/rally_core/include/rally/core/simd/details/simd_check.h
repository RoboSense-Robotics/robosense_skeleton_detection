/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef RALLY_CORE_SIMD_SIMD_CHECK_H
#define RALLY_CORE_SIMD_SIMD_CHECK_H

#include <array>
#include <iostream>
#include "rally/utils/logger/log.h"

#if defined(_MSC_VER)
#ifdef __AVX__
#ifndef RALLY_HAVE_SSE2
#define RALLY_HAVE_SSE2
#endif
#ifndef RALLY_HAVE_SSE3
#define RALLY_HAVE_SSE3
#endif
#ifndef RALLY_HAVE_SSE41
#define RALLY_HAVE_SSE41
#endif
#ifndef RALLY_HAVE_AVX
#define RALLY_HAVE_AVX
#endif
#endif
#if (defined( _M_X64) || defined(_M_IX86_FP) && _M_IX86_FP >= 2) && !defined(RALLY_HAVE_SSE2)
#define RALLY_HAVE_SSE2
#endif
#else
#ifdef __SSE2__
#ifndef RALLY_HAVE_SSE2
#define RALLY_HAVE_SSE2
#endif
#endif
#ifdef __SSSE3__
#ifndef RALLY_HAVE_SSE3
#define RALLY_HAVE_SSE3
#endif
#endif
#ifdef __SSE4_1__
#ifndef RALLY_HAVE_SSE41
#define RALLY_HAVE_SSE41
#endif
#endif
#ifdef __AVX__
#ifndef RALLY_HAVE_AVX
#define RALLY_HAVE_AVX
#endif
#endif
#ifdef __AVX2__
#ifndef RALLY_HAVE_AVX2
#define RALLY_HAVE_AVX2
#endif
#endif
#ifdef __ALTIVEC__
#ifndef RALLY_HAVE_ALTIVEC
#define RALLY_HAVE_ALTIVEC
#endif
#endif
#ifdef __VSX__
#ifndef RALLY_HAVE_VSX
#define RALLY_HAVE_VSX
#endif
#endif
#ifdef __VEC__ // __VEC__ = 10206
#ifndef RALLY_HAVE_POWER_VEC	// vector and vec_ intrinsics
#define RALLY_HAVE_POWER_VEC
#endif
#endif
#ifdef __ARM_NEON
#ifndef RALLY_HAVE_NEON
#define RALLY_HAVE_NEON
#endif
#endif
#endif

// ----------------------------------------------------------------------------------------


#ifdef RALLY_HAVE_ALTIVEC
#include <altivec.h>
#endif

#ifdef RALLY_HAVE_SSE2

#include <xmmintrin.h>
#include <emmintrin.h>
#include <mmintrin.h>

#endif
#ifdef RALLY_HAVE_SSE3
#include <pmmintrin.h> // SSE3
#include <tmmintrin.h>
#endif
#ifdef RALLY_HAVE_SSE41
#include <smmintrin.h> // SSE4
#endif
#ifdef RALLY_HAVE_AVX
#include <immintrin.h> // AVX
#endif
#ifdef RALLY_HAVE_AVX2
#include <immintrin.h> // AVX
//    #include <avx2intrin.h>
#endif
#ifdef RALLY_HAVE_NEON
#include <arm_neon.h> // ARM NEON
#endif

// ----------------------------------------------------------------------------------------
// Define functions to check, at runtime, what instructions are available

#if defined(_MSC_VER) && (defined(_M_I86) || defined(_M_IX86) || defined(_M_X64) || defined(_M_AMD64))
#include <intrin.h>

namespace rally {
inline std::array<unsigned int,4> cpuid(int function_id) {
    std::array<unsigned int,4> info;
    // Load EAX, EBX, ECX, EDX into info
    __cpuid((int*)info.data(), function_id);
    return info;
}
}  // namespace rally

#elif (defined(__GNUC__) || defined(__clang__)) && (defined(__i386__) || defined(__i686__) || defined(__amd64__) || defined(__x86_64__))

#include <cpuid.h>

namespace rally {
inline std::array<unsigned int, 4> cpuid(int function_id) {
    std::array<unsigned int, 4> info;
    // Load EAX, EBX, ECX, EDX into info
    __cpuid_count(function_id, 0, info[0], info[1], info[2], info[3]);
    return info;
}
}  // namespace rally

#else

namespace rally {
inline std::array<unsigned int,4> cpuid(int) {
    return std::array<unsigned int,4>{};
}
}  // namespace rally

#endif

namespace rally {
inline bool cpu_has_sse2_instructions() { return 0 != (cpuid(1)[3] & (1 << 26)); }

inline bool cpu_has_sse3_instructions() { return 0 != (cpuid(1)[2] & (1 << 0)); }

inline bool cpu_has_sse41_instructions() { return 0 != (cpuid(1)[2] & (1 << 19)); }

inline bool cpu_has_sse42_instructions() { return 0 != (cpuid(1)[2] & (1 << 20)); }

inline bool cpu_has_avx_instructions() { return 0 != (cpuid(1)[2] & (1 << 28)); }

inline bool cpu_has_avx2_instructions() { return 0 != (cpuid(7)[1] & (1 << 5)); }

inline bool cpu_has_avx512_instructions() { return 0 != (cpuid(7)[1] & (1 << 16)); }

inline void warn_about_unavailable_but_used_cpu_instructions() {
#if defined(RALLY_HAVE_AVX2)
    if (!cpu_has_avx2_instructions()) {
        RERROR << "Rally was compiled to use AVX2 instructions, but these aren't available on your machine.";
    }
#elif defined(RALLY_HAVE_AVX)
    if (!cpu_has_avx_instructions()) {
        RERROR << "Rally was compiled to use AVX instructions, but these aren't available on your machine.";
    }
#elif defined(RALLY_HAVE_SSE41)
    if (!cpu_has_sse41_instructions()) {
        RERROR << "Rally was compiled to use SSE41 instructions, but these aren't available on your machine.";
    }
#elif defined(RALLY_HAVE_SSE3)
    if (!cpu_has_sse3_instructions()) {
        RERROR << "Rally was compiled to use SSE3 instructions, but these aren't available on your machine.";
    }
#elif defined(RALLY_HAVE_SSE2)
    if (!cpu_has_sse2_instructions()) {
        RERROR << "Rally was compiled to use SSE2 instructions, but these aren't available on your machine.";
    }
#endif
}
}  // namespace rally

// ---------

#endif  // RALLY_CORE_SIMD_SIMD_CHECK_H
