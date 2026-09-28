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
 * @file            readORB.cc
 *
 * @brief           Implements Settings::readORB(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <opencv2/core/persistence.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

SettingsStatus Settings::readORB(cv::FileStorage &storage_inout)
{
    bool found;

    int parameter{};
    if (readParameter<int>(storage_inout,
                           "ORBextractor.nFeatures",
                           found,
                           parameter) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readParameter cannot fail; continue as before.
    }
    featureCount = parameter;
    float parameter2{};
    if (readParameter<float>(storage_inout,
                             "ORBextractor.scaleFactor",
                             found,
                             parameter2) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readParameter cannot fail; continue as before.
    }
    orbScaleFactor = parameter2;
    int parameter3{};
    if (readParameter<int>(storage_inout,
                           "ORBextractor.nLevels",
                           found,
                           parameter3) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readParameter cannot fail; continue as before.
    }
    pyramidLevels = parameter3;
    int parameter4{};
    if (readParameter<int>(storage_inout,
                           "ORBextractor.iniThFAST",
                           found,
                           parameter4) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readParameter cannot fail; continue as before.
    }
    initialFastThreshold = parameter4;
    int parameter5{};
    if (readParameter<int>(storage_inout,
                           "ORBextractor.minThFAST",
                           found,
                           parameter5) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readParameter cannot fail; continue as before.
    }
    minimumFastThreshold = parameter5;

    return SettingsStatus::SETTINGS_STATUS_SUCCESS;
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
