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

bool Tracking::parseORBParamFile(cv::FileStorage &settings_in)
{
    bool  isParameterMissing   = false;
    int   featureCount         = 0;
    int   levelCount           = 0;
    int   initialThresholdFast = 0;
    int   minimumThresholdFast = 0;
    float scaleFactor          = 0.0F;

    cv::FileNode node = settings_in["ORBextractor.nFeatures"];
    if (!node.empty() && node.isInt())
    {
        featureCount = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is "
                     "not an integer*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settings_in["ORBextractor.scaleFactor"];
    if (!node.empty() && node.isReal())
    {
        scaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is "
                     "not a real number*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settings_in["ORBextractor.nLevels"];
    if (!node.empty() && node.isInt())
    {
        levelCount = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not "
                     "an integer*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settings_in["ORBextractor.iniThFAST"];
    if (!node.empty() && node.isInt())
    {
        initialThresholdFast = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is "
                     "not an integer*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settings_in["ORBextractor.minThFAST"];
    if (!node.empty() && node.isInt())
    {
        minimumThresholdFast = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is "
                     "not an integer*"
                  << std::endl;
        isParameterMissing = true;
    }

    if (isParameterMissing)
    {
        return false;
    }

    p_orbExtractorLeft = new ORBextractor(featureCount,
                                          scaleFactor,
                                          levelCount,
                                          initialThresholdFast,
                                          minimumThresholdFast);

    if (sensor == System::STEREO || sensor == System::IMU_STEREO)
        p_orbExtractorRight = new ORBextractor(featureCount,
                                               scaleFactor,
                                               levelCount,
                                               initialThresholdFast,
                                               minimumThresholdFast);

    if (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR)
        p_iniOrbExtractor = new ORBextractor(5 * featureCount,
                                             scaleFactor,
                                             levelCount,
                                             initialThresholdFast,
                                             minimumThresholdFast);

    // Adaptive FAST threshold initialization
    lastFrameFeatures        = 0;
    consecutiveLowFeatures   = 0;
    baseInitialFastThreshold = initialThresholdFast;
    baseMinimumFastThreshold = minimumThresholdFast;

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << featureCount << endl;
    cout << "- Scale Levels: " << levelCount << endl;
    cout << "- Scale Factor: " << scaleFactor << endl;
    cout << "- Initial Fast Threshold: " << initialThresholdFast << endl;
    cout << "- Minimum Fast Threshold: " << minimumThresholdFast << endl;

    return true;
}

} // namespace core
} // namespace vs_graphs
