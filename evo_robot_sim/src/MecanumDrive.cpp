//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MecanumDrive.cpp
 * @author Evocortex GmbH (MMA)
 *
 * @brief Class for the mecanum drive
 *
 * @version 0.1
 * @date 2019-08-09
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#include "MecanumDrive.h"

namespace evo {

MecanumDrive::MecanumDrive() :
    _logger_prefix("MecanumDrive: "), 
    _is_initialized(false),
    _wheel_radius_in_m(0.0), 
    _wheel_separation_length_in_m(0.0),
    _wheel_separation_width_in_m(0.0), 
    _wheel_separation_sum_in_m(0.0),
    _last_rotation_back_left(0.0), 
    _last_rotation_back_right(0.0),
    _last_rotation_front_left(0.0), 
    _last_rotation_front_right(0.0)
{
   evo::log::init("");
}


bool MecanumDrive::checkInitState()
{
   bool parameters_set = true;
   if(_wheel_radius_in_m <= 0.0)
   {
      parameters_set &= false;
      evo::log::get() << _logger_prefix << "Wheel Radius not set!" << evo::warn;
   }
   if(_wheel_separation_length_in_m <= 0.0)
   {
      parameters_set &= false;
      evo::log::get() << _logger_prefix << "Wheel separation length not set!"
                      << evo::warn;
   }
   if(_wheel_separation_width_in_m <= 0.0)
   {
      parameters_set &= false;
      evo::log::get() << _logger_prefix << "Wheel separation width not set!"
                      << evo::warn;
   }

   _is_initialized = parameters_set;
   evo::log::get() << _logger_prefix << "init state: " << _is_initialized
                   << evo::info;
   return _is_initialized;
}

void MecanumDrive::setWheelRadiusInM(const double wheel_radius_in_m)
{
   _wheel_radius_in_m = wheel_radius_in_m;
   _ms2rpm            = 60.0 / (2.0 * _wheel_radius_in_m * M_PI);
   _rpm2ms            = 1.0 / _ms2rpm;
   _rot2m             = (2.0 * _wheel_radius_in_m * M_PI);
   _m2rad          = 1.0 / _wheel_radius_in_m; // 2 * pi / 2 * r * pi = 1/r
   _rad2m          = _wheel_radius_in_m;
}

void MecanumDrive::setWheelSeparationLengthInM(
    const double wheel_separation_length_in_m)
{
   _wheel_separation_length_in_m = wheel_separation_length_in_m;
   _wheel_separation_sum_in_m =
       _wheel_separation_length_in_m + _wheel_separation_width_in_m;
}

void MecanumDrive::setWheelSeparationWidthInM(
    const double wheel_separation_Width_in_m)
{
   _wheel_separation_width_in_m = wheel_separation_Width_in_m;
   _wheel_separation_sum_in_m =
       _wheel_separation_length_in_m + _wheel_separation_width_in_m;
}

void MecanumDrive::setWheelDistanceFrontBackInM(
    const double wheel_distance_front_back_in_m)
{
   setWheelSeparationLengthInM(wheel_distance_front_back_in_m / 2.0);
}

void MecanumDrive::setWheelDistanceLeftRightInM(
    const double wheel_distance_left_right_in_m)
{
   setWheelSeparationWidthInM(wheel_distance_left_right_in_m / 2.0);
}


void MecanumDrive::wheelData2OdomVel(const MecanumWheelData& wd, MecanumVel& mv, const double conversion_factor)
{
   // rpm2ms
   mv._x_ms = conversion_factor * (-wd.front_left + wd.front_right - wd.back_left + wd.back_right) / 4.0;
   mv._y_ms = conversion_factor * ( wd.front_left + wd.front_right - wd.back_left - wd.back_right) / 4.0;
   mv._yaw_rads = conversion_factor * (wd.front_left + wd.front_right + wd.back_left + wd.back_right) / (4.0 * _wheel_separation_sum_in_m);
}

void MecanumDrive::wheelData2OdomPose(const MecanumWheelData& wd, MecanumPose& mp, const double conversion_factor)
{
   // rot2m
   mp._x_m = conversion_factor * (-wd.front_left + wd.front_right - wd.back_left + wd.back_right) / 4.0;
   mp._y_m = conversion_factor * ( wd.front_left + wd.front_right - wd.back_left - wd.back_right) / 4.0;
   mp._yaw_rad = conversion_factor * (wd.front_left + wd.front_right + wd.back_left + wd.back_right) / (4.0 * _wheel_separation_sum_in_m);
}

void MecanumDrive::wheelData2OdomPoseInc(const MecanumWheelData& wd, MecanumWheelData& lwd, MecanumPose& mpi, const double conversion_factor)
{
   MecanumWheelData inc_wd = wd;
   inc_wd.back_left -= lwd.back_left;
   inc_wd.back_right -= lwd.back_right;
   inc_wd.front_left -= lwd.front_left;
   inc_wd.front_right -= lwd.front_right;

   wheelData2OdomPose(inc_wd, mpi, conversion_factor);

   lwd = wd;   
}


void MecanumDrive::cmdVel2wheelData(const MecanumVel& cmd_vel, MecanumWheelData& cmd_wd, const double conversion_factor)
{
   // ms2rpm
   cmd_wd.front_left = conversion_factor * (-cmd_vel._x_ms + cmd_vel._y_ms + (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
   cmd_wd.back_left  = conversion_factor * (-cmd_vel._x_ms - cmd_vel._y_ms + (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));

   cmd_wd.front_right = conversion_factor * ( cmd_vel._x_ms + cmd_vel._y_ms + (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
   cmd_wd.back_right = conversion_factor * ( cmd_vel._x_ms - cmd_vel._y_ms + (_wheel_separation_sum_in_m * cmd_vel._yaw_rads));
}






} // namespace evo
