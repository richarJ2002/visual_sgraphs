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

void Tracking::loadTrackingParameters(const string &settingPath_in)
{
    cv::FileStorage settings(settingPath_in, cv::FileStorage::READ);

    auto readInt = [&settings](const char *p_name,
                               int         minimum,
                               int         maximum,
                               int         defaultValue)
    {
        const cv::FileNode node = settings[p_name];
        if (node.empty())
        {
            return defaultValue;
        }

        if (!node.isInt())
        {
            std::cerr << "[Tracking] Ignoring '" << p_name
                      << "': expected an integer in [" << minimum << ", "
                      << maximum << "]. Using " << defaultValue << "."
                      << std::endl;
            return defaultValue;
        }

        const int value = node.operator int();
        if (value < minimum || value > maximum)
        {
            std::cerr << "[Tracking] Ignoring '" << p_name << "': " << value
                      << " is outside [" << minimum << ", " << maximum
                      << "]. Using " << defaultValue << "." << std::endl;
            return defaultValue;
        }
        return value;
    };

    auto readReal = [&settings](const char *p_name,
                                double      minimum,
                                double      maximum,
                                double      defaultValue)
    {
        const cv::FileNode node = settings[p_name];
        if (node.empty())
        {
            return defaultValue;
        }

        if (!node.isReal())
        {
            std::cerr << "[Tracking] Ignoring '" << p_name
                      << "': expected a real number in [" << minimum << ", "
                      << maximum << "]. Using " << defaultValue << "."
                      << std::endl;
            return defaultValue;
        }

        const double value = node.real();
        if (!std::isfinite(value) || value < minimum || value > maximum)
        {
            std::cerr << "[Tracking] Ignoring '" << p_name << "': " << value
                      << " is outside [" << minimum << ", " << maximum
                      << "]. Using " << defaultValue << "." << std::endl;
            return defaultValue;
        }
        return value;
    };

    minInliersForKF =
        readInt("Tracking.MinInliersForKF", 1, 1000, minInliersForKF);
    minCloseInliersForKF =
        readInt("Tracking.MinCloseInliersForKF", 0, 1000, minCloseInliersForKF);
    if (minCloseInliersForKF > minInliersForKF)
    {
        std::cerr << "[Tracking] Tracking.MinCloseInliersForKF exceeds "
                     "Tracking.MinInliersForKF. Using "
                  << minInliersForKF << "." << std::endl;
        minCloseInliersForKF = minInliersForKF;
    }

    minKeyFrameTemporalSpacing = readReal("Tracking.MinTemporalSpacingKF",
                                          0.0,
                                          60.0,
                                          minKeyFrameTemporalSpacing);
    maxKFsInLocalMap =
        readInt("Tracking.MaxKFsInLocalMap", 1, 10000, maxKFsInLocalMap);
    motionModelSearchRadiusMultiplier = static_cast<float>(
        readReal("Tracking.MotionModelSearchRadiusMultiplier",
                 1.0,
                 4.0,
                 motionModelSearchRadiusMultiplier));
    motionModelMaxSearchRadius = readInt("Tracking.MotionModelMaxSearchRadius",
                                         15,
                                         200,
                                         motionModelMaxSearchRadius);
    initializationMinPoints    = readInt("Tracking.InitializationMinPoints",
                                      1,
                                      10000,
                                      initializationMinPoints);
    relocalizationMinInliers   = readInt("Tracking.RelocalizationMinInliers",
                                       6,
                                       1000,
                                       relocalizationMinInliers);

    cout << endl << "Effective Tracking Parameters:" << endl;
    cout << "- Min Inliers for KF: " << minInliersForKF << endl;
    cout << "- Min Close Inliers for KF: " << minCloseInliersForKF << endl;
    cout << "- Min Temporal Spacing KF: " << minKeyFrameTemporalSpacing << " s"
         << endl;
    cout << "- Max KFs in Local Map: " << maxKFsInLocalMap << endl;
    cout << "- Motion Model Search Radius Multiplier: "
         << motionModelSearchRadiusMultiplier << endl;
    cout << "- Motion Model Max Search Radius: " << motionModelMaxSearchRadius
         << endl;
    cout << "- Initialization Min Points: " << initializationMinPoints << endl;
    cout << "- Relocalization Min Inliers: " << relocalizationMinInliers
         << endl;
}

} // namespace core
} // namespace vs_graphs
