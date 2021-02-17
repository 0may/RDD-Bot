//###############################################################
//# Copyright (C) 2016, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE.txt.            #
//###############################################################

#ifndef LOGTYPE_H_
#define LOGTYPE_H_

#include <vector>
#include <string>

#include "evo_logger/time/Time.h"

namespace evo {

using LogType = unsigned int; ///< type for enum, alias for flexible usage

namespace Log {
/**
 * enums For Logging
 */
enum Log : LogType
{
   INFO  = 1, ///< INFO
   DEBUG = 2, ///< DEBUG
   WARN  = 4, ///< WARN
   ERROR = 8, ///< ERROR

   ALL = 0xFFFFFFFF ///< ALL
};
} // namespace Log

static std::vector<std::string> LEVEL_STR = {
    "0", "INFO ", "DEBUG", "3",    "WARN ",
    "5", "6",     "7",     "ERROR"}; ///< string corresponding to LogLevel for easy
                                     ///< output

/**
 * Struct for Log message containing Timestamp, Loglevel and Message
 *
 * @author MSC
 */
struct LogObj
{
   evo::Time stamp;        ///< Timestamp for log
   Log::Log level;         ///< Loglevel for log
   const std::string text; ///< Logmessage for log

   /**
    * Parse function to convert LogObj to String (for terminal and file output)
    *
    * @param[in] obj object to parse
    * @return string parsed from Logobj
    */
   static std::string parse(const LogObj& obj)
   {
      std::string str = std::string("[") + evo::Time::toString(obj.stamp) +
                        std::string("]-[") +
                        std::string(LEVEL_STR[static_cast<LogType>(obj.level)]) +
                        std::string("]  ") + obj.text;
      return str;
   }
};

} // namespace evo

#endif /* LOGTYPE_H_ */
