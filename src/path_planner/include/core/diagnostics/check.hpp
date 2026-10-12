/**
 * @file check.hpp
 * @author Seif ElShafey
 * @brief Always-on runtime validation macro for external/untrusted input at module boundaries.
 * @version 0.1
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <stdexcept>
#include <string>

/**
 * @brief Validate a precondition on external input. Unlike assert(), CHECK is compiled into every
 *        build (including release) and reports failure by throwing std::invalid_argument, so the
 *        caller can surface a configuration error instead of aborting.
 *
 * Use CHECK for input that can legitimately be wrong at runtime (user config, file contents).
 * Use assert() for internal invariants that can only fail due to a programming error.
 *
 * @param cond Expression that must be true. If false, an exception is thrown.
 * @param msg  Human-readable description of the violated precondition (anything convertible to std::string).
 */
#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (!(cond))                                                           \
            throw std::invalid_argument(                                       \
                std::string(__FILE__ ":") + std::to_string(__LINE__) +         \
                " CHECK failed: " + (msg));                                    \
    } while (0)
