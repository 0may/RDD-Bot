//###############################################################
//# Copyright (C) 2016, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE.txt.            #
//###############################################################

#ifndef WRITER_H_
#define WRITER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <cassert>

#include "evo_logger/log/LogType.h"

namespace evo {

/**
 * Class for writing log-messages into a given file, logs will be appended in file.
 *
 * @todo error handling when file is unable to write...
 *
 * @author MSC
 */
class Writer
{
 public:
   /**
    * Constructor
    *
    * @param[in] file for writing logs
    */
   Writer(std::string file) : _file(file) {}

   /**
    * Writes and deletes given logs to file, logs will be appended in file.
    *
    * @param[in, out] obj containing all logs to write
    */
   void write(std::vector<LogObj>& obj)
   {
      std::ofstream out;

      out.open(_file.c_str(), std::ios::app | std::ios::out);
      if(!out)
      {
         /*error*/
         // todo
         return;
      }

      for(auto& e : obj)
      {
         out << LogObj::parse(e) << std::endl;
      }
      out.close();
      // delete vector-content
      obj.clear();
   }

 private:
   std::string _file; ///< File for writing logs
};

} // namespace evo
#endif /* WRITER_H_ */
