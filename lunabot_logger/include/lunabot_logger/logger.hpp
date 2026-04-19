#pragma once

#include "rclcpp/rclcpp.hpp"

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define MAGENTA "\033[1;35m"
#define CYAN "\033[1;36m"

// Logging macros
#define LOGGER_INFO(logger, fmt, ...) RCLCPP_INFO(logger, fmt, ##__VA_ARGS__)
#define LOGGER_DEBUG(logger, fmt, ...) RCLCPP_DEBUG(logger, fmt, ##__VA_ARGS__)
#define LOGGER_SUCCESS(logger, fmt, ...) RCLCPP_INFO(logger, GREEN fmt RESET, ##__VA_ARGS__)
#define LOGGER_ACTION(logger, fmt, ...) RCLCPP_INFO(logger, CYAN fmt RESET, ##__VA_ARGS__)
#define LOGGER_WARN(logger, fmt, ...) RCLCPP_WARN(logger, YELLOW fmt RESET, ##__VA_ARGS__)
#define LOGGER_FAILURE(logger, fmt, ...) RCLCPP_ERROR(logger, RED fmt RESET, ##__VA_ARGS__)

// Throttled / once variants
#define LOGGER_WARN_ONCE(logger, fmt, ...) RCLCPP_WARN_ONCE(logger, YELLOW fmt RESET, ##__VA_ARGS__)
#define LOGGER_INFO_THROTTLE(logger, clock, ms, fmt, ...) \
  RCLCPP_INFO_THROTTLE(logger, clock, ms, fmt, ##__VA_ARGS__)
#define LOGGER_WARN_THROTTLE(logger, clock, ms, fmt, ...) \
  RCLCPP_WARN_THROTTLE(logger, clock, ms, YELLOW fmt RESET, ##__VA_ARGS__)
