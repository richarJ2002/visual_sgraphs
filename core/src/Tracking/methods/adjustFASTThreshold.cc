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

#include "System.h"
#include "Tracking.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

// Adaptive FAST threshold: lower thresholds when tracking degrades
// In low-texture corridors, fewer features are extracted, so we lower the
// threshold
TrackingStatus Tracking::adjustFASTThreshold()
{
    // Count features in current frame
    int currentFeatureCount = currentFrame.keyPointCount;

    // If this is the first frame after initialization, just record
    if (lastFrameFeatures == 0)
    {
        lastFrameFeatures = currentFeatureCount;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    // Check if feature count dropped significantly
    float featureRatio = static_cast<float>(currentFeatureCount) /
                         static_cast<float>(std::max(1, lastFrameFeatures));

    // If features dropped below 50% of previous (more sensitive), or absolute
    // count is very low
    bool lowFeatures = (featureRatio < 0.5f) || (currentFeatureCount < 400);

    if (lowFeatures)
    {
        consecutiveLowFeatures++;
    }
    else
    {
        consecutiveLowFeatures = 0;
    }

    // Adjust thresholds based on consecutive low-feature frames
    // Lower thresholds to extract more features in textureless areas
    int newInitialThresholdFast = baseInitialFastThreshold;
    int newMinimumThresholdFast = baseMinimumFastThreshold;

    if (consecutiveLowFeatures >= 1) // React faster - after just 1 frame
    {
        // Progressively lower thresholds (but not below minimum)
        // Each step reduces by 3, minimum of 1 for both (more aggressive)
        int reduction = std::min(consecutiveLowFeatures, 6) * 3;
        newInitialThresholdFast =
            std::max(baseInitialFastThreshold - reduction, 1);
        newMinimumThresholdFast =
            std::max(baseMinimumFastThreshold - reduction, 1);
    }
    else if (consecutiveLowFeatures == 0 && currentFeatureCount > 2500)
    {
        // Plenty of features - can restore base thresholds
        newInitialThresholdFast = baseInitialFastThreshold;
        newMinimumThresholdFast = baseMinimumFastThreshold;
    }

    // Apply new thresholds if changed
    int orbExtractorLeftGetInitialFastThreshold{};
    if (p_orbExtractorLeft->getInitialFastThreshold(
            orbExtractorLeftGetInitialFastThreshold) !=
        ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getInitialFastThreshold returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    int orbExtractorLeftGetMinimumFastThreshold{};
    if (!(newInitialThresholdFast != orbExtractorLeftGetInitialFastThreshold) &&
        p_orbExtractorLeft->getMinimumFastThreshold(
            orbExtractorLeftGetMinimumFastThreshold) !=
            ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMinimumFastThreshold returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (newInitialThresholdFast != orbExtractorLeftGetInitialFastThreshold ||
        newMinimumThresholdFast != orbExtractorLeftGetMinimumFastThreshold)
    {
        if (p_orbExtractorLeft->setInitialFastThreshold(
                newInitialThresholdFast) !=
            ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: setInitialFastThreshold returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_orbExtractorLeft->setMinimumFastThreshold(
                newMinimumThresholdFast) !=
            ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: setMinimumFastThreshold returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_orbExtractorRight)
        {
            if (p_orbExtractorRight->setInitialFastThreshold(
                    newInitialThresholdFast) !=
                ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setInitialFastThreshold returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_orbExtractorRight->setMinimumFastThreshold(
                    newMinimumThresholdFast) !=
                ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setMinimumFastThreshold returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        if (p_iniOrbExtractor)
        {
            if (p_iniOrbExtractor->setInitialFastThreshold(
                    newInitialThresholdFast) !=
                ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setInitialFastThreshold returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_iniOrbExtractor->setMinimumFastThreshold(
                    newMinimumThresholdFast) !=
                ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setMinimumFastThreshold returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        if (Verbose::printMess(
                "[Tracking] Adaptive FAST: iniTh=" +
                    std::to_string(newInitialThresholdFast) +
                    " minTh=" + std::to_string(newMinimumThresholdFast) +
                    " (features=" + std::to_string(currentFeatureCount) +
                    " consecutive_low=" +
                    std::to_string(consecutiveLowFeatures) + ")",
                Verbose::VERBOSITY_NORMAL) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    lastFrameFeatures = currentFeatureCount;

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
