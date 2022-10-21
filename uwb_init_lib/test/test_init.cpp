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
    UwbInitializer uwbInit(options);

    UwbAnchorBuffer anchorBuf;

    uwbInit.init_anchors(anchorBuf);

    return 0;
}
