//###############################################################
//# Copyright (C) 2016, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE.txt.            #
//###############################################################

#ifndef UTILITY_H_
#define UTILITY_H_

#include <iostream>
#include <string>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pwd.h>

namespace evo {

/**
 * @brief Contains some useful functions, header only
 * @author MSC
 */
class Utility
{
 public:
   /**
    * Finds home directory without environment-variable, by using pwd
    *
    * @todo find better way, maybe c++17, no windows support
    * @return full path to home-dir as std::string
    */
   static std::string getHomeDir()
   {
      std::string homedir;
      homedir = getpwuid(getuid())->pw_dir;
      return homedir;
   }

   /**
    * Proves if a given directory exists, by using stat
    *
    * @todo find better way, maybe c++17, no windows support
    * @param[in] path to directory
    * @return boolean value if exists
    */
   static bool directoryExists(const std::string& path)
   {
      struct stat st;
      if(stat(path.c_str(), &st) == 0)
         if((st.st_mode & S_IFDIR) != 0)
            return true;
      return false;
   }
};

} // namespace evo

#endif /* UTILITY_H_ */
