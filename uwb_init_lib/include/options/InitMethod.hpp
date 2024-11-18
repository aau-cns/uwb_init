/******************************************************************************
 * FILENAME:     InitMethod.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     05.03.2024
 *
 *  Copyright (C) 2024
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// You can contact the authors at <alessandro.fornasier@aau.at> and
// <giulio.delama@aau.at>

#ifndef UWB_INIT_INITMETHOD_HPP
#define UWB_INIT_INITMETHOD_HPP


namespace uwb_init
{
///
/// \brief The InitMethod enum describes the method used for initialization
///
enum class InitMethod
{
  SINGLE = 0,  //!< use only one measurement to construct LLS matrix
  DOUBLE,  //!< use a pair of measurements to construct LLS matrix
};

///
/// \brief Return a string with the corresponding init method
///
constexpr const char* InitMethodString(InitMethod e)
{
  switch (e)
  {
    case InitMethod::SINGLE:
      return "InitMethod::SINGLE";
    case InitMethod::DOUBLE:
      return "InitMethod::DOUBLE";
    default:
      return "InitMethod::UNKNOWN";
  }
}

} // ns uwb_init

#endif // INITMETHOD_HPP
