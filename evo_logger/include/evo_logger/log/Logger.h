//###############################################################
//# Copyright (C) 2016, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE.txt.            #
//###############################################################

#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>
#include <cstdio>
#include <chrono>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <memory>
#include <stdexcept>
#include <thread>
#include <mutex>

#include "evo_logger/log/LogType.h"
#include "evo_logger/log/Writer.h"
#include "evo_logger/log/OstreamColor.h"
#include "evo_logger/time/Time.h"
#include "evo_logger/base/Utility.h"

namespace evo {

static const std::string LOG_FOLDER =
    ".evocortex"; ///< Folder in Homedir where log-file are stored

/**
 * @brief Class for Logging as Singleton.
 *
 * This Logger stores all logs and writes all Logs in to a file (append). The log
 * files are stored in "homdir/.evocortex/". The class evo::log is a simple wrapper
 * for the Logger for easy usage. Log levels can only be changed for terminal output.
 *
 * Logger uses ostream for stream logging, so every object with overloaded ostream is
 * accepted.
 *
 * Recommended usage:
 *
 * log initialize:
 * @code
 * evo::log::init("Logger-name");
 * @endcode
 *
 * log stream:
 * @code
 * evo::log::get() << "log message" << someData << evo::info;  //for info stream
 * evo::log::get() << "log message" << someData << evo::error;  //for error stream
 * ...
 * @endcode
 *
 * printf syntax:
 * @code
 * evo::info("printf-%s", "style");
 * evo::debug("printf-syntax %d", 4);
 * @endcode
 *
 * strings as parameter:
 * @code
 * evo::warn("warn-msg");
 * evo::error(std::string("warn-msg");
 * @endcode
 *
 * write log-file
 * @code
 * evo::log::writeLog();
 * @endcode
 *
 * @todo thread safe impl (thread c++11)
 * @todo preprocessor exclude for debug in release mode
 * @todo safe before program exit -> save after each log or something... maybe extra
 * mode...
 * @todo add kind of __pretty_function__ style in logger output (origin-> line
 * file... )
 * @todo sublogger
 *
 * @author MSC
 */
class Logger : public std::basic_stringstream<char>
{
 public:
   /**
    * Copy-Constructor deleted -> Singleton
    */
   Logger(const Logger&) = delete;

   /**
    * Move-Constructor deleted -> Singleton
    */
   Logger(Logger&&) = delete;

   /**
    * =operator(copy) deleted -> Singleton
    */
   Logger& operator=(const Logger&) = delete;

   /**
    * =operator(move) deleted -> Singleton
    */
   Logger& operator=(Logger&&) = delete;

   /**
    * Singleton pattern
    *
    * @return Instance from Logger
    */
   static inline Logger& instance()
   {
      static Logger instance;
      return instance;
   }

   /**
    * Initialize Logger, has only on first call an effect
    *
    * @param[in] name name of program or user defined name
    */
   void initialize(const std::string& name)
   {
      // set only once
      if(_writer)
      {
         return;
      }

      _name = name;

      std::string log_folder = Utility::getHomeDir() + "/" + LOG_FOLDER + "/";

      // prove folder exists
      if(!Utility::directoryExists(log_folder))
      {
         // create log folder
         // linux
         if(mkdir(log_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0 ||
            (!Utility::directoryExists(log_folder)))
         {
            _os << "LOG_FOLDER could not be set.... take current Direktory"
                << std::endl;
            log_folder = "";
         }
      }

      // get name for logfile:
      std::string log_file(evo::Time::toString(evo::Time::now()) + std::string("-") +
                           _name + std::string(".log"));

      _writer = std::unique_ptr<Writer>(new Writer(log_folder + log_file));
   }

 private:
   /**
    * Default private Constructor -> Singleton
    */
   Logger() :
       _os(std::cout), _color_def_f(OSColor(Color::F_DEFAULT)),
       _color_def_b(OSColor(Color::B_DEFAULT)),
       _color_info_f(OSColor(Color::F_DEFAULT)),
       _color_info_b(OSColor(Color::B_DEFAULT)),
       _color_debug_f(OSColor(Color::F_LIGHT_BLUE)),
       _color_debug_b(OSColor(Color::B_DEFAULT)),
       _color_warn_f(OSColor(Color::F_LIGHT_RED)),
       _color_warn_b(OSColor(Color::B_DEFAULT)),
       _color_error_f(OSColor(Color::F_DEFAULT)),
       _color_error_b(OSColor(Color::B_RED))
   {
      _current_log_level = static_cast<LogType>(Log::ALL);
   }

   /**
    * Destructor writes Logs to file
    */
   ~Logger()
   {
      if(!_writer)
      {
         this->initialize("EVO");
      }
      _writer->write(_logs);
   }

   /**
    * forces ERROR output, even if it is disabled
    *
    * @todo check if needed
    */
   void forceOutput() { _current_log_level |= static_cast<LogType>(Log::ERROR); }

   std::vector<LogObj> _logs; ///< Container for logs

   LogType _current_log_level; ///< current Log level

   std::string _name; ///< name of Logger

   std::unique_ptr<Writer> _writer; ///< Writer Object

   std::ostream& _os; ///< ostream

   std::mutex _mutex; ///< mutex for thread safety (c++11)

   OSColor _color_def_f;   ///< default color foreground
   OSColor _color_def_b;   ///< default color background
   OSColor _color_info_f;  ///< info color foreground
   OSColor _color_info_b;  ///< info color background
   OSColor _color_debug_f; ///< debug color foreground
   OSColor _color_debug_b; ///< debug color background
   OSColor _color_warn_f;  ///< warn color foreground
   OSColor _color_warn_b;  ///< warn color background
   OSColor _color_error_f; ///< error color foreground
   OSColor _color_error_b; ///< error color background

 public: // functions
   /**
    * Getter function for logger name
    *
    * @return logger name as std::string
    */
   inline std::string getName() const { return _name; }

   /**
    * Getter function for Logs
    * @return Logs as LobObj
    */
   inline std::vector<LogObj>& getLogs() { return _logs; }

   /**
    * @brief Basic log function, used from all wrapper functions
    *
    * Basic log function, saves log level and log message and writes log message
    * to given output stream if given log level is activated for terminal output
    *
    * @param[in] level log level of this log
    * @param[in] text  log message of this log
    */
   inline void log(Log::Log level, const std::string& text)
   {
      _mutex.lock();
      // save log
      LogObj obj = {evo::Time::now(), level, text};
      try
      {
         _logs.push_back(std::move(obj));
      } catch(std::bad_alloc& e)
      {
         _os << "Bad alloc in _logs.push_back... will delete all logs to prevent "
                "more damage"
             << std::endl;
         exit(0);
         _logs.clear();
      }
      // prove output
      if(static_cast<LogType>(level) & _current_log_level) // binary and
      {
         _os << LogObj::parse(obj) << _color_def_b << _color_def_f
             << std::endl; // set default color
      }
      else
      {
         _os << _color_def_b << _color_def_f; // to prevent bug where disabled log
                                              // level color terminal
      }
      _mutex.unlock();
   }

   /**
    * Forces logger to write all logs stored in _logs in given file (appends file)
    */
   inline void writeLog()
   {
      _mutex.lock();
      if(!_writer)
      {
         this->initialize("EVO");
      }
      _writer->write(_logs);
      _mutex.unlock();
   }

   /**
    * function for log at info level, std::string only
    *
    * @param[in] text log message
    */
   inline void info(const std::string& text)
   {
      _os << _color_info_f << _color_info_b;
      this->log(Log::INFO, text);
   }

   /**
    * function for log at debug level, std::string only
    *
    * @param[in] text log message
    */
   inline void debug(const std::string& text)
   {
      _os << _color_debug_f << _color_debug_b;
      this->log(Log::DEBUG, text);
   }

   /**
    * function for log at warn level, std::string only
    *
    * @param[in] text log message
    */
   inline void warn(const std::string& text)
   {
      _os << _color_warn_f << _color_warn_b;
      this->log(Log::WARN, text);
   }

   /**
    * function for log at error level, std::string only
    *
    * @param[in] text log message
    */
   inline void error(const std::string& text)
   {
      _os << _color_error_f << _color_error_b;
      this->log(Log::ERROR, text);
   }

   /**
    * function for setting log level, only for terminal output, all logs will be
    * written to file
    *
    * @note log level ERROR can not be disabled (will be force activated)
    *
    * Following code shows usage:
    * @code
    * evo::log::get().setLogLevel(evo::Log::INFO);
    * evo::log::get().setLogLevel(evo::Log::INFO | evo::Log::WARN | evo::Log::ERROR);
    * @endcode
    *
    * @param[in] level as Log-enum (e.G. Log::INFO), more levels can be appendend
    * with |-operator
    */
   inline void setLogLevel(const LogType level)
   {
      _current_log_level = static_cast<LogType>(level);
      this->forceOutput();
   }

   /**
    * function for appending a log level to current configuration
    * @param[in] level as Log-enum (e.G. Log::INFO), more levels can be appendend
    * with |-operator
    */
   inline void appendLogLevel(const LogType level)
   {
      _current_log_level |= static_cast<LogType>(level);
      this->forceOutput();
   }

   /**
    * function for removing a specific log level from current configuration
    *
    * @note log level ERROR can not be disabled (will be force activated)
    *
    * @param[in] level as Log-enum (e.G. Log::INFO), more levels can be appendend
    * with |-operator
    */
   inline void removeLogLevel(const LogType level)
   {
      _current_log_level &= ~(static_cast<LogType>(level));
      this->forceOutput();
   }
};

/**
 * @brief Wrapper class for static functions for logging
 *
 * Use mainly this class for logging see evo::Logger for more documentation
 *
 * @author MSC
 */
class log
{
 public:
   /**
    * Wraps Logger::write()
    */
   static inline void writeLog() { Logger::instance().writeLog(); }

   /**
    * Wraps Logger::initialize(..)
    * @param[in] name logger name
    */
   static inline void init(const std::string& name)
   {
      Logger::instance().initialize(name);
   }

   /**
    * Wraps Logger::instance()
    * @return Logger instance
    */
   static inline Logger& instance() { return Logger::instance(); }

   /**
    * Wraps Logger::instance()
    * @return Logger instance
    */
   static inline Logger& get() { return Logger::instance(); }

   /**
    * Wraps Logger::info(..)
    * @param[in] text log message as std::string
    */
   static inline void info(const std::string& text)
   {
      Logger::instance().info(text);
   }

   /**
    * Wraps Logger::info(..)
    * @param str log message as const char* (C-String)
    */
   static inline void info(const char* str) { Logger::instance().info(str); }

   /**
    * Delegates printf-syntax as std::string to Logger::info(..)
    * @param[in] cstr printf str
    * @param[in] args printf agrs
    */
   template<typename... Args>
   static inline void info(const char* cstr, Args... args)
   {
      Logger::instance().info(log::printfToString(cstr, args...));
   }

   /**
    * Wraps Logger::debug(..)
    * @param[in] text log message as std::string
    */
   static inline void debug(const std::string& text)
   {
      Logger::instance().debug(text);
   }

   /**
    * Wraps Logger::debug(..)
    * @param[in] text log message as const char* (C-String)
    */
   static inline void debug(const char* str) { Logger::instance().debug(str); }

   /**
    * Delegates printf-syntax as std::string to Logger::debug(..)
    * @param[in] cstr printf str
    * @param[in] args printf agrs
    */
   template<typename... Args>
   static inline void debug(const char* cstr, Args... args)
   {
      Logger::instance().debug(log::printfToString(cstr, args...));
   }

   /**
    * Wraps Logger::warn(..)
    * @param[in] text log message as std::string
    */
   static inline void warn(const std::string& text)
   {
      Logger::instance().warn(text);
   }

   /**
    * Wraps Logger::warn(..)
    * @param[in] text log message as const char* (C-String)
    */
   static inline void warn(const char* str) { Logger::instance().warn(str); }

   /**
    * Delegates printf-syntax as std::string to Logger::warn(..)
    * @param[in] cstr printf str
    * @param[in] args printf agrs
    */
   template<typename... Args>
   static inline void warn(const char* cstr, Args... args)
   {
      Logger::instance().warn(log::printfToString(cstr, args...));
   }

   /**
    * Wraps Logger::error(..)
    * @param[in] text log message as std::string
    */
   static inline void error(const std::string& text)
   {
      Logger::instance().error(text);
   }

   /**
    * Wraps Logger::error(..)
    * @param[in] text log message as const char* (C-String)
    */
   static inline void error(const char* str) { Logger::instance().error(str); }

   /**
    * Delegates printf-syntax as std::string to Logger::error(..)
    * @param[in] cstr printf str
    * @param[in] args printf agrs
    */
   template<typename... Args>
   static inline void error(const char* cstr, Args... args)
   {
      Logger::instance().error(log::printfToString(cstr, args...));
   }

 private:
   /**
    * Converts a Printf-Syntax to std::string
    * @param[in] str  printf-style string (%f,%d,...)
    * @param[in] args printf args
    * @return std::string formated with printf
    */
   template<typename... Args>
   static inline std::string printfToString(const char* str, Args... args)
   {
      std::size_t size = snprintf(nullptr, 0, str, args...) + 1; // +1 for '\0'
      std::unique_ptr<char[]> buffer(new char[size]);
      snprintf(buffer.get(), size, str, args...);

      return std::string(buffer.get(), buffer.get() + size - 1); // without '\0'
   }
};

/**
 * Class for ending info log stream
 * @author MSC
 */
class Info
{
 public:
   friend std::ostream& operator<<(std::ostream& os, const evo::Info& rhs)
   {
      // in case it is not used with Logger instance
      std::string str = log::get().str();
      //    if( str.empty() )
      //      return os;

      log::info(str);
      // clear sstream
      log::instance().str("");
      log::instance().clear();
      return os;
   }
};

/**
 * Class for ending debug log stream
 * @author MSC
 */
class Debug
{
   friend std::ostream& operator<<(std::ostream& os, const evo::Debug& rhs)
   {
      // in case it is not used with Logger instance
      std::string str = log::get().str();
      //    if( str.empty() )
      //      return os;

      log::debug(str);
      // clear sstream
      log::instance().str("");
      log::instance().clear();
      return os;
   }
};

/**
 * Class for ending warn log stream
 * @author MSC
 */
class Warn
{
   friend std::ostream& operator<<(std::ostream& os, const evo::Warn& rhs)
   {
      // in case it is not used with Logger instance
      std::string str = log::get().str();
      //    if( str.empty() )
      //      return os;

      log::warn(str);
      // clear sstream
      log::instance().str("");
      log::instance().clear();
      return os;
   }
};

/**
 * Class for ending error log stream
 * @author MSC
 */
class Error
{
   friend std::ostream& operator<<(std::ostream& os, const evo::Error& rhs)
   {
      // in case it is not used with Logger instance
      std::string str = log::get().str();
      //    if( str.empty() )
      //      return os;

      log::error(str);
      // clear sstream
      log::instance().str("");
      log::instance().clear();
      return os;
   }
};

// static instances for usage without ()
__attribute__((unused)) static Info
    info; // GCC flag -> no waring if unsused todo check is ok so
__attribute__((unused)) static Debug debug;
__attribute__((unused)) static Warn warn;
__attribute__((unused)) static Error error;

#define EVO_LOG evo::log::instance()

} // namespace evo

#endif /* LOGGER_H_ */
