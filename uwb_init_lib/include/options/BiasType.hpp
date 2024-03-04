/******************************************************************************
 * FILENAME:     BiasType.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     04.03.2024
 *
 *  Copyright (C) 2024
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#ifndef UWB_INIT_BIASTYPE_HPP
#define UWB_INIT_BIASTYPE_HPP

///
/// \brief The BiasType enum describes the type of biase used in the measurement model
///
#include <string>
enum class BiasType
{
  NO_BIAS = 0,     //!< use no bias for initialization, i.e. position only
  CONST_BIAS,  //!< only use constant bias and position in initialization
  ALL_BIAS,    //!< use constant and distance bias in initialization
};

///
/// \brief Return a string with the corresponding bias type
///
const char* BiasTypeString(BiasType e);

std::string BiasType_to_string(BiasType const& e);


#endif // BIASTYPE_HPP
