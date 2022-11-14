// Copyright (C) 2022 Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// You can contact the author at <alessandro.fornasier@aau.at>

#ifndef UWB_INIT_UTILITIES_H
#define UWB_INIT_UTILITIES_H

#include <cctype>
#include <sstream>
#include <string>

namespace uwb_init_ros
{
/**
 * @brief containsChar: Check if a string contains character.
 * True is a string contians characters, False if a string is only made of numbers
 *
 * @param[in] string
 * @return bool
 */
inline bool containsChar(const std::string& str)
{
  for (char const& c : str)
  {
    if (std::isdigit(c) == 0)
      return true;
  }
  return false;
}
}  // namespace uwb_init_ros

#endif  // UWB_INIT_UTILITIES_H
