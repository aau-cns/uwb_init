// Copyright (C) 2021 Martin Scheiber, Alessandro Fornasier
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

#include <iostream>

#include "uwb_init.hpp"

using namespace uav_init;

int main()
{
    UwbInitOptions options;
    UwbAnchorBuffer anchorBuf;
    UwbInitializer uwbInit(options);

    double pose_t;
    uav_init::UwbData uwb;
    std::vector<UwbData> uwb_meas;
    Eigen::Vector3d pose;

    for (uint i = 0; i < 100; ++i) {

        pose = Eigen::Vector3d::Zero();

        pose << 1.0, 1.0, 1.0;
        pose_t = 0.1 * i;

        uwb.id = 1;
        uwb.valid = 1;
        uwb.distance = 1.0;
        uwb.timestamp = 0.1 * i;

        uwb_meas = {uwb};

        uwbInit.feed_pose(pose_t, pose);
        uwbInit.feed_uwb(uwb_meas);

    }

    uwbInit.init_anchors(anchorBuf);

    return 0;
}
