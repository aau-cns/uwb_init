// Copyright (C) 2022 Giulio Delamar, Control of Networked Systems,
// University of Klagenfurt, Austria.
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
// You can contact the author at <giulio.delama@aau.at>

#ifndef UWB_INIT_LS_OPTIONS_HPP_
#define UWB_INIT_LS_OPTIONS_HPP_

namespace uwb_init
{
///
/// \brief The LsSolverOptions struct is an object containing all 'static' parameters used
/// for the linear least squares problem solver.
///
struct LsSolverOptions
{
  /// position uncertainty
  double sigma_pos{ 0.03 };

  /// uwb uncertainty
  double sigma_meas{ 0.1 };

  /// const bias flag
  bool const_bias_flag{ false };

};  // struct LsSolverOptions
}  // namespace uwb_init

#endif  // UWB_INIT_LS_OPTIONS_HPP_
