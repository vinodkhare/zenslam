/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt-headers.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


@file
@brief Assertions used in the project
*/

#pragma once

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

// MSVC compatibility: __builtin_expect is not available in MSVC
#if defined(_MSC_VER) && !defined(__builtin_expect)
#define __builtin_expect(expr, expected) (expr)
#endif

// Compiler-specific branch prediction hints
#if defined(_MSC_VER)
// MSVC doesn't support __builtin_expect, so we define these as pass-through
#define BASALT_LIKELY(x) (x)
#define BASALT_UNLIKELY(x) (x)
#elif defined(__GNUC__) || defined(__clang__)
// GCC and Clang support __builtin_expect
#define BASALT_LIKELY(x) __builtin_expect(x, 1)
#define BASALT_UNLIKELY(x) __builtin_expect(x, 0)
#else
// Fallback for other compilers
#define BASALT_LIKELY(x) (x)
#define BASALT_UNLIKELY(x) (x)
#endif

namespace basalt {

#define UNUSED(x) (void)(x)

#define BASALT_ATTRIBUTE_NORETURN [[noreturn]]

// Compiler-specific function name macro
#if defined(_MSC_VER)
    #define BASALT_FUNCTION_NAME __FUNCSIG__
#else
    #define BASALT_FUNCTION_NAME __PRETTY_FUNCTION__
#endif

inline void assertionFailed(char const* expr,
                                                      char const* function,
                                                      char const* file,
                                                      long line) {
  std::cerr << "***** Assertion (" << expr << ") failed in " << function
            << ":\n"
            << file << ':' << line << ":" << std::endl;
  std::abort();
}

inline void assertionFailedMsg(char const* expr,
                                                         char const* msg,
                                                         char const* function,
                                                         char const* file,
                                                         long line) {
  std::cerr << "***** Assertion (" << expr << ") failed in " << function
            << ":\n"
            << file << ':' << line << ": " << msg << std::endl;
  std::abort();
}

inline void logFatal(char const* function,
                                               char const* file, long line) {
  std::cerr << "***** Fatal error in " << function << ":\n"
            << file << ':' << line << ":" << std::endl;
  std::abort();
}

inline void logFatalMsg(char const* msg,
                                                  char const* function,
                                                  char const* file, long line) {
  std::cerr << "***** Fatal error in " << function << ":\n"
            << file << ':' << line << ": " << msg << std::endl;
  std::abort();
}

}  // namespace basalt

#if defined(BASALT_DISABLE_ASSERTS)

#define BASALT_ASSERT(expr) ((void)0)

#define BASALT_ASSERT_MSG(expr, msg) ((void)0)

#define BASALT_ASSERT_STREAM(expr, msg) ((void)0)

#else

#define BASALT_ASSERT(expr)                                              \
  (BASALT_LIKELY(!!(expr))                                               \
       ? ((void)0)                                                       \
       : ::basalt::assertionFailed(#expr, BASALT_FUNCTION_NAME, __FILE__, \
                                   __LINE__))

#define BASALT_ASSERT_MSG(expr, msg)                                   \
  (BASALT_LIKELY(!!(expr))                                             \
       ? ((void)0)                                                     \
       : ::basalt::assertionFailedMsg(#expr, msg, BASALT_FUNCTION_NAME, \
                                      __FILE__, __LINE__))

#define BASALT_ASSERT_STREAM(expr, msg)                                   \
  (BASALT_LIKELY(!!(expr))                                                \
       ? ((void)0)                                                        \
       : (std::cerr << msg << std::endl,                                  \
          ::basalt::assertionFailed(#expr, BASALT_FUNCTION_NAME, __FILE__, \
                                    __LINE__)))

#endif
