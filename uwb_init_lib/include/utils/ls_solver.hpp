// Copyright (C) 2021 Giulio Delama
// Control of Networked Systems, Universitaet Klagenfurt, Austria
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

#ifndef LS_SOLVER_HPP_
#define LS_SOLVER_HPP_

#include "logger/logger.hpp"
#include "options/uwb_init_options.hpp"
#include "options/ls_solver_options.hpp"
#include "utils/data_structs.hpp"
#include "utils/utils.hpp"

namespace uwb_init
{

class LsSolver
{
public:
    LsSolver(const std::shared_ptr<Logger> logger, const UwbInitOptions& options);

    // Update configuration
    void configure(const UwbInitOptions& options);

    // Least Squares solver
    bool solve_ls(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                  Eigen::VectorXd& lsSolution, Eigen::MatrixXd& cov);

    // Least squares initialization handling
    std::function<bool(const TimedBuffer<UwbData>&, const PositionBuffer&,
                       Eigen::MatrixXd&, Eigen::VectorXd&, Eigen::VectorXd&)> ls_problem;

private:
    /// Shared pointer to logger
    std::shared_ptr<Logger> logger_;

    // LsSolver parameters
    LsSolverOptions ls_params_;

    ///
    /// \brief functions for least squares problem formulation depending on selected method and variables
    /// \param UWB data for the single anchor, coefficient matrix A, measurement vector b (A * x = b), uncertainty s
    /// \return ture if successful, false if not
    ///
    bool ls_single_const_bias(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                              Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s);
    bool ls_single_no_bias(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                           Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s);
    bool ls_double_const_bias(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                              Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s);
    bool ls_double_no_bias(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                           Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s);

};

}  // namespace uwb_init

#endif  // LS_SOLVER_HPP_
