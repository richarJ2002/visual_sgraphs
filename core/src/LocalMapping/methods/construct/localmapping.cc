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

#include "LocalMapping.h"

namespace vs_graphs
{
namespace core
{

LocalMapping::LocalMapping(
    System                             *p_system_in,
    Atlas                              *p_atlas_in,
    const float                         monocular_in,
    bool                                inertial_in,
    [[maybe_unused]] const std::string &sequenceName_in) :
    scale(1.0),
    initSection(0),
    initIndex(0),
    iterationIndex(0),
    isFirstImuBaPending(true),
    isSecondImuBaPending(true),
    p_system(p_system_in),
    isMonocular(monocular_in),
    isInertial(inertial_in),
    isResetRequested(false),
    isResetActiveMapRequested(false),
    isFinishRequested(false),
    hasFinished(true),
    p_atlas(p_atlas_in),
    shouldAbortBa(false),
    hasStopped(false),
    isStopRequested(false),
    isStopBlocked(false),
    shouldAcceptKeyFrames(true),
    isInitializationInProgress(false),
    infoInertial(Eigen::MatrixXd::Zero(9, 9))
{
    localMappingCount       = 0;
    initializationStartTime = 0.f;
    isImuBad                = false;
    keyFrameCullingCount    = 0;
    matchesInliers          = 0;

#ifdef REGISTER_TIMES
    localBaExecutionCount = 0;
    localBaAbortCount     = 0;
#endif
}

} // namespace core
} // namespace vs_graphs
