//###############################################################
//# Copyright (C) 2016, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE.txt.            #
//###############################################################

#ifndef EVOOSTREAMCOLOR_H_
#define EVOOSTREAMCOLOR_H_

#include <ostream>

namespace evo {

/**
 * Enum for Colors as foreground and background
 */
enum class Color : unsigned int
{
   F_RED        = 31, ///< F_RED
   F_GREEN      = 32, ///< F_GREEN
   F_BLUE       = 34, ///< F_BLUE
   F_DEFAULT    = 39, ///< F_DEFAULT
   F_LIGHT_RED  = 91, ///< F_LIGHT_RED
   F_LIGHT_BLUE = 94, ///< F_LIGHT_BLUE

   B_RED     = 41, ///< B_RED
   B_GREEN   = 42, ///< B_GREEN
   B_BLUE    = 44, ///< B_BLUE
   B_DEFAULT = 49  ///< B_DEFAULT
};

/**
 * Class for manipulating Terminal output color
 *
 * @author MSC
 */
class OSColor
{
 public:
   /**
    * Constructor, defines Color enum
    * @param[in] c enum for defining Color
    */
   OSColor(Color c) : _col(c) {}

   /**
    * Overloaded ostream for Terminal color + start and end chars.
    *
    * @param os ostream for output
    * @param rhs Defined Color for Terminal output
    * @return given ostream
    */
   friend std::ostream& operator<<(std::ostream& os, const OSColor& rhs)
   {
      os << "\33[" << static_cast<unsigned int>(rhs._col) << "m";
      return os;
   }

 private:
   Color _col; ///< defined Colortype
};

} // namespace evo

#endif /* EVOOSTREAMCOLOR_H_ */
