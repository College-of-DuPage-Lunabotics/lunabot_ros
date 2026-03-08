# lunabot_logger

Shared logging library used by all packages in this workspace. Provides colored terminal output for both C++ and Python nodes.

## Color Convention

| Color | Meaning | C++ Macro | Python Method |
|-------|---------|-----------|---------------|
| Default | General info | `LOGGER_INFO` | `log.info()` |
| Green | Success | `LOGGER_SUCCESS` | `log.success()` |
| Cyan | Action / in-progress | `LOGGER_ACTION` | `log.action()` |
| Yellow | Warning | `LOGGER_WARN` | `log.warning()` |
| Red | Failure / error | `LOGGER_FAILURE` | `log.failure()` |

## C++ Usage

Include the header and call the macros with the node's logger:

```cpp
#include "lunabot_logger/logger.hpp"

LOGGER_INFO(this->get_logger(), "Starting up");
LOGGER_SUCCESS(this->get_logger(), "Hardware initialized");
LOGGER_ACTION(this->get_logger(), "Sending goal");
LOGGER_WARN(this->get_logger(), "Low battery: %.1f V", voltage);
LOGGER_FAILURE(this->get_logger(), "Service call failed");
```

Additional macros for throttled and once-only logging:

```cpp
LOGGER_DEBUG(this->get_logger(), "Debug value: %d", val);
LOGGER_WARN_ONCE(this->get_logger(), "This warning appears once");
LOGGER_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Throttled info");
LOGGER_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Throttled warning");
```

## Python Usage

Instantiate `Logger` with the node and call its methods:

```python
from lunabot_logger import Logger

self.log = Logger(self)  # pass the node, or an rclpy Logger directly

self.log.info("Starting up")
self.log.success("Hardware initialized")
self.log.action("Sending goal")
self.log.warning(f"Low battery: {voltage:.1f} V")
self.log.failure("Service call failed")
self.log.debug(f"Debug value: {val}")
```
