//###############################################################
//# Copyright (C) 2016, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE.txt.            #
//###############################################################

#ifndef ___TYPES_H___
#define ___TYPES_H___

namespace evo {

/* set here the used data type for floating point operation. */
#define _EVO_DOUBLE_PRECISION_ 0

#if _EVO_DOUBLE_PRECISION_
using evo_float = float;
#else
using evo_float = double;
#endif

} // namespace evo

#endif
