// Copyright (C) 2021 Alessandro Fornasier, Control of Networked Systems, Universitaet Klagenfurt, Austria
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
// You can contact the authors at <alessandro.fornasier@aau.at>

#ifndef UAV_INIT_UTILS_H
#define UAV_INIT_UTILS_H

#include <string>
#include <sstream>
#include <cctype>

namespace uav_init {

    /**
     * @brief containsChar: Check if a string contains character.
     * True is a string contians characters, False if a string is only made of numbers
     *
     * @param[in] string
     * @return bool
     */
    inline bool containsChar(const std::string& str)
    {
        for (char const &c : str) {
            if (std::isdigit(c) == 0) return true;
        }
        return false;
    }

}


#endif /* UAV_INIT_UTILS_H */
