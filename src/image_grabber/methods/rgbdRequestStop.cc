/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            rgbdRequestStop.cc
 *
 * @brief           Requests shutdown of the RGB-D tracking worker.
 */

#include "common/common.h"
#include "image_grabber/objects/RgbdImageGrabberClass.h"
#include "observability/observability.h"

#include <condition_variable>

namespace vs_graphs::image_grabber
{

void RgbdImageGrabber::requestStop()
{
    {
        std::lock_guard<std::mutex> packetLock(rgbdPacketMutex);
        stopRequested = true;
    }

    rgbdPacketCondition.notify_all();
}

} /* namespace vs_graphs::image_grabber */
