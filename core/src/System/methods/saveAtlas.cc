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

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::saveAtlas(int type_in, bool &isSaved_out)
{
    try
    {
        if (!saveAtlasFile.empty())
        {
            // Save the current session
            if (p_atlas->preSave() != AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: preSave returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            std::string pathSaveFileName = "./";
            pathSaveFileName = pathSaveFileName.append(saveAtlasFile);
            pathSaveFileName = pathSaveFileName.append(".osa");

            std::string vocabularyChecksum{};
            if (calculateCheckSum(vocabularyFilePath,
                                  TEXT_FILE,
                                  vocabularyChecksum) !=
                SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: calculateCheckSum returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            std::size_t found          = vocabularyFilePath.find_last_of("/\\");
            std::string vocabularyName = vocabularyFilePath.substr(found + 1);

            if (type_in == TEXT_FILE) // File text
            {
                std::cout << "Starting to write the save text file to "
                          << pathSaveFileName.c_str() << std::endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::text_oarchive oa(ofs);

                oa << vocabularyName;
                oa << vocabularyChecksum;
                oa << p_atlas;
                std::cout << "End to write the save text file" << std::endl;
            }
            else if (type_in == BINARY_FILE) // File binary
            {
                std::cout << "Starting to write the save binary file to "
                          << pathSaveFileName.c_str() << std::endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::binary_oarchive oa(ofs);
                oa << vocabularyName;
                oa << vocabularyChecksum;
                oa << p_atlas;
                std::cout << "End to write save binary file" << std::endl;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        isSaved_out = false;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }
    catch (...)
    {
        std::cerr << "Unknows exeption" << std::endl;
        isSaved_out = false;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }

    isSaved_out = true;
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
