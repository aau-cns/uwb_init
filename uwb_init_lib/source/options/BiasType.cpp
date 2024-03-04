/******************************************************************************
 * FILENAME:     BiasType.cpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     04.03.2024
 *
 *  Copyright (C) 2024
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#include <options/BiasType.hpp>


std::string BiasType_to_string(const BiasType &e) {
  return std::string(BiasTypeString(e));
}

const char *BiasTypeString(BiasType e)
{
  switch (e)
  {
    case BiasType::NO_BIAS:
      return "BiasType::NO_BIAS";
    case BiasType::CONST_BIAS:
      return "BiasType::CONST_BIAS";
    case BiasType::ALL_BIAS:
      return "BiasType::ALL_BIAS";
    default:
      return "";
  }
}
