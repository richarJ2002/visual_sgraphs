/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

#include "TwoViewReconstruction.h"

#include "GeometricTools.h"
#include "Utils/Converter/objects/Converter.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include <thread>

using namespace std;
namespace vs_graphs
{
namespace core
{

bool TwoViewReconstruction::Reconstruct(const std::vector<cv::KeyPoint> &vKeys1,
                                        const std::vector<cv::KeyPoint> &vKeys2,
                                        const vector<int>   &vMatches12,
                                        Sophus::SE3f        &T21,
                                        vector<cv::Point3f> &vP3D,
                                        vector<bool>        &vbTriangulated)
{
    keys1.clear();
    keys2.clear();

    keys1 = vKeys1;
    keys2 = vKeys2;

    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    matches12.clear();
    matches12.reserve(keys2.size());
    matchedFlags1.resize(keys1.size());
    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (vMatches12[i] >= 0)
        {
            matches12.push_back(make_pair(i, vMatches12[i]));
            matchedFlags1[i] = true;
        }
        else
            matchedFlags1[i] = false;
    }

    const int N = matches12.size();

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for (int i = 0; i < N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    sets = vector<vector<size_t>>(maxIterations, vector<size_t>(8, 0));

    DUtils::Random::SeedRandOnce(0);

    for (int it = 0; it < maxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for (size_t j = 0; j < 8; j++)
        {
            int randi =
                DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);
            int idx = vAvailableIndices[randi];

            sets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a
    // homography
    vector<bool>    vbMatchesInliersH, vbMatchesInliersF;
    float           SH, SF;
    Eigen::Matrix3f H, F;

    thread threadH(&TwoViewReconstruction::findHomography,
                   this,
                   ref(vbMatchesInliersH),
                   ref(SH),
                   ref(H));
    thread threadF(&TwoViewReconstruction::findFundamental,
                   this,
                   ref(vbMatchesInliersF),
                   ref(SF),
                   ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    if (SH + SF == 0.f)
        return false;
    float RH = SH / (SH + SF);

    float minParallax = 1.0;

    // Try to reconstruct from homography or fundamental depending on the ratio
    // (0.40-0.45)
    if (RH > 0.50) // if(RH>0.40)
    {
        // cout << "Initialization from Homography" << endl;
        return reconstructH(vbMatchesInliersH,
                            H,
                            calibrationMatrix,
                            T21,
                            vP3D,
                            vbTriangulated,
                            minParallax,
                            50);
    }
    else // if(pF_HF>0.6)
    {
        // cout << "Initialization from Fundamental" << endl;
        return reconstructF(vbMatchesInliersF,
                            F,
                            calibrationMatrix,
                            T21,
                            vP3D,
                            vbTriangulated,
                            minParallax,
                            50);
    }
}

} // namespace core
} // namespace vs_graphs
