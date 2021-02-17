//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MecanumDrive.h
 * @author Evocortex GmbH (MMA)
 *
 * @brief Class for the mecanum drive
 *
 * @version 0.1
 * @date 2019-08-09
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 * 
 * WARN: THIS IS A COPIED CLASS OUT OF THE R&D SOFTWARE PACKAGES AND WAS MODIFIED
 * Main differences: removed all references to HW motors (removed evo_mbed dep)
 */

#ifndef MECANUMDRIVE_H
#define MECANUMDRIVE_H

#include "evo_logger/log/Logger.h"

namespace evo {

struct MecanumWheelData
{
   double front_left = 0.0;
   double front_right = 0.0;
   double back_left = 0.0;
   double back_right = 0.0;

   inline MecanumWheelData& operator*=(const double factor)
   {
      this->front_left *= factor;
      this->front_right *= factor;
      this->back_left *= factor;
      this->back_right *= factor;
      return *this;
   }
};

struct MecanumVel
{
   double _x_ms     = 0.0;
   double _y_ms     = 0.0;
   double _yaw_rads = 0.0;
};

struct MecanumPose
{
   double _x_m     = 0.0;
   double _y_m     = 0.0;
   double _yaw_rad = 0.0;

   void reset()
   {
      // todo: test this line
      *this = MecanumPose();
   }

   void updatePoseFromVel(const MecanumVel& vel, const double cycle_time)
   {
      this->_x_m += (std::cos(this->_yaw_rad) * vel._x_ms -
                     std::sin(this->_yaw_rad) * vel._y_ms) *
                    cycle_time;
      this->_y_m += (std::sin(this->_yaw_rad) * vel._x_ms +
                     std::cos(this->_yaw_rad) * vel._y_ms) *
                    cycle_time;
      this->_yaw_rad += vel._yaw_rads * cycle_time;
   }

   void updatePoseFromIncrement(const MecanumPose& inc)
   {
      this->_x_m +=  std::cos(this->_yaw_rad) * inc._x_m - 
                     std::sin(this->_yaw_rad) * inc._y_m;
      this->_y_m +=  std::sin(this->_yaw_rad) * inc._x_m + 
                     std::cos(this->_yaw_rad) * inc._y_m;
      this->_yaw_rad += inc._yaw_rad;
   }
};

struct MecanumCovariance
{
   double cov_pos_x   = 0.0;
   double cov_pos_y   = 0.0;
   double cov_pos_yaw = 0.0;
   double cov_vel_x   = 0.0;
   double cov_vel_y   = 0.0;
   double cov_vel_yaw = 0.0;
};

enum MOTOR_MAPPING_MECANUM
{
   NO_POSITION = 0,
   FRONT_LEFT,
   FRONT_RIGHT,
   BACK_RIGHT,
   BACK_LEFT
};

class MecanumDrive
{
 private:

   // robot dimensions
   double _wheel_radius_in_m;
   double _wheel_separation_length_in_m;
   double _wheel_separation_width_in_m;
   double _wheel_separation_sum_in_m;

   // helper values for calculations
   // if the robot uses rpm
   double _ms2rpm;
   double _rpm2ms;
   double _rot2m;

   // if the robot uses radps
   double _m2rad;
   double _rad2m;

   std::string _logger_prefix;

   bool _is_initialized;
   bool _verbose;

   // TODO: delete this once unused
   // save last ticks to create difference in getPoseIncement()
   double _last_rotation_front_left;
   double _last_rotation_front_right;
   double _last_rotation_back_left;
   double _last_rotation_back_right;

   MecanumWheelData _last_position;
   MecanumWheelData _current_position;
   MecanumWheelData _current_rpm;
   //-------------
 public:
      MecanumDrive();

    /**
     * @brief checks if all motor references and robot dimensions are set
     * 
     * @return true if initialized correct 
     */
    bool checkInitState();


    // Set the robot dimensions for this mecanum drive configuration
    void setWheelRadiusInM(const double wheel_radius_in_m);
    void setWheelSeparationLengthInM(const double wheel_separation_length_in_m);
    void setWheelSeparationWidthInM(const double wheel_separation_Width_in_m);
    void setWheelDistanceFrontBackInM(const double wheel_distance_front_back_in_m);
    void setWheelDistanceLeftRightInM(const double wheel_distance_left_right_in_m);

    void wheelData2OdomVel(const MecanumWheelData& wd, MecanumVel& mv, const double conversion_factor);
    void wheelData2OdomPose(const MecanumWheelData& wd, MecanumPose& mp, const double conversion_factor);
    void wheelData2OdomPoseInc(const MecanumWheelData& wd, MecanumWheelData& lwd, MecanumPose& mpi, const double conversion_factor);


    void cmdVel2wheelData(const MecanumVel& cmd_vel, MecanumWheelData& cmd_wd, const double conversion_factor);

    double getFacRad2m() const {return _rad2m;}
    double getFacM2Rad() const {return _m2rad;}
    double getFacRot2m() const {return _rot2m;}
    double getFacRpm2ms() const {return _rpm2ms;}
    double getFacMs2Rpm() const {return _ms2rpm;}
};
} // namespace evo
#endif // MECANUMDRIVE_H
