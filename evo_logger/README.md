# evo_logger
C++ library for custom logging without ROS

Include this in your CATKIN_DEPENDENCIES to use the custom logger.

Usage:

```cpp
// include Logger class
#include "log/Logger.h"

// initialize logger instance
evo::log::init("");

// log output
evo::log::get() << "DEBUG LEVEL" << evo::debug;
evo::log::get() << "INFO  LEVEL" << evo::info;
evo::log::get() << "WARN  LEVEL" << evo::warn;
evo::log::get() << "ERROR LEVEL" << evo::error;

```
