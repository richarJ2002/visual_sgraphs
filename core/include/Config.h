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

#ifndef CONFIG_H
#define CONFIG_H

/*!
 * @file         Config.h
 *
 * @brief        Declares the configuration containers and file parser.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Viewer configuration container.
 */
class ViewerConfig
{};

/*!
 * @brief        Camera configuration container.
 */
class CameraConfig
{};

/*!
 * @brief        ORB extractor configuration container.
 */
class ORBExtractorConfig
{};

/*!
 * @brief        IMU configuration container.
 */
class IMUConfig
{};

/*!
 * @brief        Parses the estimator configuration file.
 */
class ConfigParser
{
  public:
    /*!
     * @brief        Parses the configuration file at the given path.
     *
     * @param[in]    configFilePath_in
     *               Path of the configuration file to parse.
     *
     * @return       True when parsing succeeded. The stub
     *               implementation always reports success.
     */
    bool parseConfigFile(const std::string &configFilePath_in);

  private:
    /*!
     * @brief        Stored viewer configuration.
     */
    ViewerConfig       viewerConfig;
    /*!
     * @brief        Stored camera configuration.
     */
    CameraConfig       cameraConfig;
    /*!
     * @brief        Stored ORB extractor configuration.
     */
    ORBExtractorConfig orbConfig;
    /*!
     * @brief        Stored IMU configuration.
     */
    IMUConfig          imuConfig;
};

} // namespace core
} // namespace vs_graphs
#endif // CONFIG_H
