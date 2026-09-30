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

/*!
 * @file            system.cc
 *
 * @brief           Implements the System constructor, declared in System.h.
 */

#include "System.h"
#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "SemanticSegmentation.h"
#include "SemanticsManager.h"
#include "Tracking.h"
#include "Viewer.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <iomanip>
#include <memory>
#include <openssl/evp.h>
#include <pangolin/pangolin.h>
#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

System::System() :
    sensor(MONOCULAR),
    p_vocabulary(nullptr),
    p_keyFrameDatabase(nullptr),
    p_atlas(nullptr),
    p_tracker(nullptr),
    p_localMapper(nullptr),
    p_loopCloser(nullptr),
    p_viewer(static_cast<Viewer *>(nullptr)),
    p_frameDrawer(nullptr),
    p_mapDrawer(nullptr),
    p_semanticSegmentation(nullptr),
    p_semanticsManager(nullptr),
    p_viewerThread(nullptr),
    p_loopClosingThread(nullptr),
    p_localMappingThread(nullptr),
    p_semanticSegmentationThread(nullptr),
    p_semanticsManagerThread(nullptr),
    p_geometricSegmentationThread(static_cast<std::thread *>(nullptr)),
    isResetRequested(false),
    isResetActiveMapRequested(false),
    isLocalizationModeActivationRequested(false),
    isLocalizationModeDeactivationRequested(false),
    isShutdownRequested(false),
    p_settings(nullptr)
{}

} // namespace core
} // namespace vs_graphs
