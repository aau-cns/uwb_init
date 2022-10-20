// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
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
// You can contact the author at <martin.scheiber@aau.at>

#ifndef UAV_INIT_LOGGER_HPP_
#define UAV_INIT_LOGGER_HPP_

#include <iostream>
#include <memory>
#include <mutex>

namespace UavInit
{
enum LoggerLevel
{
  INACTIVE,
  INFO,
  WARN,
  ERR,
  FULL
};

class Logger
{
 public:
  /**
   * @brief Logger constructor
   */
  Logger(const LoggerLevel& level) : level_(level) {}

  /**
   * @brief Getter. Get the MSP version in use
   * @return msp version (const reference to MSPVer)
   */
  inline const LoggerLevel& getlevel() const { return level_; }

  /**
   * @brief Setter. Set the logger level version
   * @param level (const reference to LoggerLevel)
   */
  inline void setLevel(const LoggerLevel& level) { level_ = level; }

  /**
   * @brief Format a info message and log it
   * @param msg (std::string)
   */
  inline void info(const std::string& msg)
  {
    if (level_ == LoggerLevel::INFO || level_ == LoggerLevel::FULL)
    {
      log("[INFO] " + msg + '.');
    }
  }

  /**
   * @brief Format a error message (red) and log it
   * @param msg (std::string)
   */
  inline void err(const std::string& msg)
  {
    if (level_ == LoggerLevel::ERR || level_ == LoggerLevel::FULL)
    {
      log("\033[31m[ERROR] " + msg + ".\033[0m");
    }
  }

  /**
   * @brief Format a warning message (yellow) and log it
   * @param msg (std::string)
   */
  inline void warn(const std::string& msg)
  {
    if (level_ == LoggerLevel::WARN || level_ == LoggerLevel::FULL)
    {
      log("\033[33m[WARNING] " + msg + ".\033[0m");
    }
  }

 private:
  /**
   * @brief Log a message if logger is active
   * @param msg (std::string)
   */
  inline void log(const std::string& msg)
  {
    std::scoped_lock lock(logger_mtx_);
    std::cout << msg << '\n' << std::endl;
  }

  /// Logger level
  LoggerLevel level_;

  /// Logger mutex
  std::mutex logger_mtx_;
};

}  // namespace UavInit

#endif  // UAV_INIT_LOGGER_HPP_
