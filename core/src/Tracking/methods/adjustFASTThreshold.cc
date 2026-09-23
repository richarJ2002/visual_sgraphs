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

namespace vs_graphs
{
namespace core
{

// Adaptive FAST threshold: lower thresholds when tracking degrades
// In low-texture corridors, fewer features are extracted, so we lower the
// threshold
void Tracking::adjustFASTThreshold()
{
    // Count features in current frame
    int nCurrentFeatures = currentFrame.N;

    // If this is the first frame after initialization, just record
    if (lastFrameFeatures == 0)
    {
        lastFrameFeatures = nCurrentFeatures;
        return;
    }

    // Check if feature count dropped significantly
    float featureRatio =
        (float)nCurrentFeatures / (float)std::max(1, lastFrameFeatures);

    // If features dropped below 50% of previous (more sensitive), or absolute
    // count is very low
    bool lowFeatures = (featureRatio < 0.5f) || (nCurrentFeatures < 400);

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
    int newIniThFAST = baseInitialFastThreshold;
    int newMinThFAST = baseMinimumFastThreshold;

    if (consecutiveLowFeatures >= 1) // React faster - after just 1 frame
    {
        // Progressively lower thresholds (but not below minimum)
        // Each step reduces by 3, minimum of 1 for both (more aggressive)
        int reduction = std::min(consecutiveLowFeatures, 6) * 3;
        newIniThFAST  = std::max(baseInitialFastThreshold - reduction, 1);
        newMinThFAST  = std::max(baseMinimumFastThreshold - reduction, 1);
    }
    else if (consecutiveLowFeatures == 0 && nCurrentFeatures > 2500)
    {
        // Plenty of features - can restore base thresholds
        newIniThFAST = baseInitialFastThreshold;
        newMinThFAST = baseMinimumFastThreshold;
    }

    // Apply new thresholds if changed
    if (newIniThFAST != p_orbExtractorLeft->getInitialFastThreshold() ||
        newMinThFAST != p_orbExtractorLeft->getMinimumFastThreshold())
    {
        p_orbExtractorLeft->setInitialFastThreshold(newIniThFAST);
        p_orbExtractorLeft->setMinimumFastThreshold(newMinThFAST);
        if (p_orbExtractorRight)
        {
            p_orbExtractorRight->setInitialFastThreshold(newIniThFAST);
            p_orbExtractorRight->setMinimumFastThreshold(newMinThFAST);
        }
        if (p_iniOrbExtractor)
        {
            p_iniOrbExtractor->setInitialFastThreshold(newIniThFAST);
            p_iniOrbExtractor->setMinimumFastThreshold(newMinThFAST);
        }
        Verbose::printMess(
            "[Tracking] Adaptive FAST: iniTh=" + std::to_string(newIniThFAST) +
                " minTh=" + std::to_string(newMinThFAST) +
                " (features=" + std::to_string(nCurrentFeatures) +
                " consecutive_low=" + std::to_string(consecutiveLowFeatures) +
                ")",
            Verbose::VERBOSITY_NORMAL);
    }

    lastFrameFeatures = nCurrentFeatures;
}

} // namespace core
} // namespace vs_graphs
