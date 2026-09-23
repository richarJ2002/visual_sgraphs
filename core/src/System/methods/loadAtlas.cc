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

namespace vs_graphs
{
namespace core
{

bool System::loadAtlas(int type)
{
    string strFileVoc, strVocChecksum;
    bool   isRead = false;

    string pathLoadFileName = "./";
    pathLoadFileName        = pathLoadFileName.append(loadAtlasFile);
    pathLoadFileName        = pathLoadFileName.append(".osa");

    if (type == TEXT_FILE) // File text
    {
        cout << "Starting to read the save text file "
             << pathLoadFileName.c_str() << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if (!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::text_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> p_atlas;
        cout << "End to load the save text file " << endl;
        isRead = true;
    }
    else if (type == BINARY_FILE) // File binary
    {
        cout << "Starting to read the save binary file "
             << pathLoadFileName.c_str() << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if (!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::binary_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> p_atlas;
        cout << "End to load the save binary file" << endl;
        isRead = true;
    }

    if (isRead)
    {
        // Check if the vocabulary is the same
        string strInputVocabularyChecksum =
            calculateCheckSum(vocabularyFilePath, TEXT_FILE);

        if (strInputVocabularyChecksum.compare(strVocChecksum) != 0)
        {
            cout << "The vocabulary load isn't the same which the load session "
                    "was created "
                 << endl;
            cout << "-Vocabulary name: " << strFileVoc << endl;
            return false; // Both are differents
        }

        p_atlas->setKeyFrameDatabase(p_keyFrameDatabase);
        p_atlas->setORBVocabulary(p_vocabulary);
        p_atlas->PostLoad();

        return true;
    }
    return false;
}

} // namespace core
} // namespace vs_graphs
