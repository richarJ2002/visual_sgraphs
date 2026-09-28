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
 * @file            readLoadAndSave.cc
 *
 * @brief           Implements Settings::readLoadAndSave(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <string>

#include <opencv2/core/persistence.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

using namespace std;

void Settings::readLoadAndSave(cv::FileStorage &storage_inout)
{
    bool found;

    atlasLoadPath = readParameter<string>(storage_inout,
                                          "System.LoadAtlasFromFile",
                                          found,
                                          false);
    atlasSavePath = readParameter<string>(storage_inout,
                                          "System.SaveAtlasToFile",
                                          found,
                                          false);
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
