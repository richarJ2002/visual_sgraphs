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

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::loadAtlas(int type_in, bool &isLoaded_out)
{
    std::string fileVocabulary, vocabularyChecksum;
    bool        isRead = false;

    std::string pathLoadFileName = "./";
    pathLoadFileName             = pathLoadFileName.append(loadAtlasFile);
    pathLoadFileName             = pathLoadFileName.append(".osa");

    if (type_in == TEXT_FILE) // File text
    {
        std::cout << "Starting to read the save text file "
                  << pathLoadFileName.c_str() << std::endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if (!ifs.good())
        {
            std::cout << "Load file not found" << std::endl;
            isLoaded_out = false;
            return SystemStatus::SYSTEM_STATUS_SUCCESS;
        }
        boost::archive::text_iarchive ia(ifs);
        ia >> fileVocabulary;
        ia >> vocabularyChecksum;
        ia >> p_atlas;
        std::cout << "End to load the save text file " << std::endl;
        isRead = true;
    }
    else if (type_in == BINARY_FILE) // File binary
    {
        std::cout << "Starting to read the save binary file "
                  << pathLoadFileName.c_str() << std::endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if (!ifs.good())
        {
            std::cout << "Load file not found" << std::endl;
            isLoaded_out = false;
            return SystemStatus::SYSTEM_STATUS_SUCCESS;
        }
        boost::archive::binary_iarchive ia(ifs);
        ia >> fileVocabulary;
        ia >> vocabularyChecksum;
        ia >> p_atlas;
        std::cout << "End to load the save binary file" << std::endl;
        isRead = true;
    }

    if (isRead)
    {
        // Check if the vocabulary is the same
        std::string inputVocabularyChecksum{};
        if (calculateCheckSum(vocabularyFilePath,
                              TEXT_FILE,
                              inputVocabularyChecksum) !=
            SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: calculateCheckSum returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (inputVocabularyChecksum.compare(vocabularyChecksum) != 0)
        {
            std::cout
                << "The vocabulary load isn't the same which the load session "
                   "was created "
                << std::endl;
            std::cout << "-Vocabulary name: " << fileVocabulary << std::endl;
            isLoaded_out = false;
            return SystemStatus::SYSTEM_STATUS_SUCCESS; // Both are differents
        }

        if (p_atlas->setKeyFrameDatabase(p_keyFrameDatabase) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setKeyFrameDatabase returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_atlas->setORBVocabulary(p_vocabulary) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setORBVocabulary returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_atlas->postLoad() != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: postLoad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        isLoaded_out = true;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }
    isLoaded_out = false;
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
