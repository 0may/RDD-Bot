//###############################################################
//# Copyright (C) 2016, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE.txt.            #
//###############################################################

#ifndef EVOTIMER_H_
#define EVOTIMER_H_

#include <iostream>

#include "evo_logger/log/Logger.h"
#include "evo_logger/time/Time.h"

namespace evo {

/**
 * Timer class
 *
 * @author MSC
 */
class Timer
{
 public:
   /**
    * Default Constructor, does  not start Timer.
    *
    * @todo start timer in Constructor?
    */
   Timer() { this->start(); }

   /**
    * Destuctor
    */
   virtual ~Timer() {}

   /**
    * Starts Timer
    */
   void start() noexcept { _start = Time::now(); }

   /**
    * Computes Duration between last start and current elapsed-call
    * @return Duration from start until now
    */
   Duration elapsed() noexcept { return (Time::now() - _start); }

 protected:
   Time _start; ///< Timepoint based on evo::Time
};

/**
 * TimerAuto class logs info (given msg + Duration as [ms]) when Destructor is called
 *
 * @author MSC
 */
class TimerAuto_ms : public Timer
{
 public:
   /**
    * Constructor starts Timer and saves info-msg
    * @param[in] msg to log when elapsed, default value = "timer: "
    */
   TimerAuto_ms(std::string msg) noexcept : Timer()
   {
      _msg = msg;
      this->start();
   }

   TimerAuto_ms() : TimerAuto_ms(std::string("timer: ")) {}

   /**
    * Destructor, computes Duration when called and logs msg + Duration as
    * milliseconds.
    */
   virtual ~TimerAuto_ms()
   {
      double t = this->elapsed().toMSec();
      log::info("%s%f ms", _msg.c_str(), t);
   }

 private:
   std::string _msg; ///< Message to log when elapsed
};

/**
 * TimerAuto class logs info (given msg + Duration as [us]) when Destructor is called
 *
 * @author MSC
 */
class TimerAuto_us : public Timer
{
 public:
   /**
    * Constructor starts Timer and saves info-msg
    * @param[in] msg to log when elapsed, default value = "timer: "
    */
   TimerAuto_us(std::string msg) noexcept : Timer()
   {
      _msg = msg;
      this->start();
   }

   TimerAuto_us() : TimerAuto_us(std::string("timer: ")) {}

   /**
    * Destructor, computes Duration when called and logs msg + Duration as
    * microseconds.
    */
   virtual ~TimerAuto_us()
   {
      double t = this->elapsed().toUSec();
      log::info("%s%f us", _msg.c_str(), t);
   }

 private:
   std::string _msg; ///< Message to log when elapsed
};

} // namespace evo

#endif /* EVOTIMER_H_ */
