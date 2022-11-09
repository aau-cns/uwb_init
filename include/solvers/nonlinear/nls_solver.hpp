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

#ifndef NLS_SOLVER_HPP_
#define NLS_SOLVER_HPP_

#include "logger/logger.hpp"
#include "options/nls_solver_options.hpp"
#include "utils/data_structs.hpp"
#include "utils/utils.hpp"

namespace uwb_init
{
class NlsSolver
{
public:
  NlsSolver(const std::shared_ptr<Logger> logger);

  // Least Squares solver
  bool solve_nls(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer, Eigen::VectorXd& theta,
                 Eigen::MatrixXd& cov);

  // Output Error Algorithm solver
  bool solve_oea(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer, Eigen::VectorXd& theta,
                 Eigen::MatrixXd& cov);

private:
  /// Shared pointer to logger
  std::shared_ptr<Logger> logger_;

  // LsSolver parameters
  NlsSolverOptions nls_params_;
};

}  // namespace uwb_init

#endif  // NLS_SOLVER_HPP_
