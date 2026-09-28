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
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "System.h"

namespace vs_graphs
{
namespace core
{

void System::addSegmentedImage(
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *p_tuple_in)
{
    // Adding the segmented image to the buffer of the SemanticSegmentation
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    if (p_params->general.modeOfOperation ==
        types::SystemParams::General::ModeOfOperation::GEO)
    {
        // just clear the pointcloud of the keyframe and return, as semantic
        // segmentation is not running. Still counts as "returned" -- the
        // keyframe's round trip through the pipeline is over either way, and
        // the lockstep backlog signal must not stall forever in GEO mode.
        vs_graphs::core::KeyFrame *p_keyFrame =
            p_atlas->getKeyFrameById(std::get<0>(*p_tuple_in));
        if (p_keyFrame)
        {
            p_keyFrame->clearPointCloud();
        }
        segmentationReturnedCount.fetch_add(1U, std::memory_order_relaxed);
        lastReturnedKeyFrameId.store(std::get<0>(*p_tuple_in),
                                     std::memory_order_relaxed);
        return;
    }

    p_semanticSegmentation->addSegmentedFrameToBuffer(p_tuple_in);
    segmentationReturnedCount.fetch_add(1U, std::memory_order_relaxed);
    lastReturnedKeyFrameId.store(std::get<0>(*p_tuple_in),
                                 std::memory_order_relaxed);
}

} // namespace core
} // namespace vs_graphs
