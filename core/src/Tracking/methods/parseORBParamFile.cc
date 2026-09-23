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

#include "Tracking.h"

#include <iostream>

namespace vs_graphs
{
namespace core
{

bool Tracking::parseORBParamFile(cv::FileStorage &fSettings)
{
    bool  b_miss_params = false;
    int   nFeatures     = 0;
    int   nLevels       = 0;
    int   fIniThFAST    = 0;
    int   fMinThFAST    = 0;
    float fScaleFactor  = 0.0F;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if (!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is "
                     "not an integer*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if (!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is "
                     "not a real number*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if (!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not "
                     "an integer*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if (!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is "
                     "not an integer*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if (!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is "
                     "not an integer*"
                  << std::endl;
        b_miss_params = true;
    }

    if (b_miss_params)
    {
        return false;
    }

    p_orbExtractorLeft = new ORBextractor(nFeatures,
                                          fScaleFactor,
                                          nLevels,
                                          fIniThFAST,
                                          fMinThFAST);

    if (sensor == System::STEREO || sensor == System::IMU_STEREO)
        p_orbExtractorRight = new ORBextractor(nFeatures,
                                               fScaleFactor,
                                               nLevels,
                                               fIniThFAST,
                                               fMinThFAST);

    if (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR)
        p_iniOrbExtractor = new ORBextractor(5 * nFeatures,
                                             fScaleFactor,
                                             nLevels,
                                             fIniThFAST,
                                             fMinThFAST);

    // Adaptive FAST threshold initialization
    lastFrameFeatures        = 0;
    consecutiveLowFeatures   = 0;
    baseInitialFastThreshold = fIniThFAST;
    baseMinimumFastThreshold = fMinThFAST;

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}

} // namespace core
} // namespace vs_graphs
