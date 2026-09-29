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
#include "Tracking.h"

#include <iomanip>

namespace vs_graphs
{
namespace core
{

void System::saveTrajectoryTUM(const string &filename_in)
{
    cout << endl
         << "Saving camera trajectory to " << filename_in << " ..." << endl;
    if (sensor == MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular."
             << endl;
        return;
    }

    vector<KeyFrame *> keyFrames = p_atlas->getAllKeyFrames();
    sort(keyFrames.begin(), keyFrames.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Two = keyFrames[0]->getPoseInverse();

    ofstream f;
    f.open(filename_in.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    list<vs_graphs::core::KeyFrame *>::iterator rits =
        p_tracker->referenceKeyFrames.begin();
    list<double>::iterator lT  = p_tracker->frameTimes.begin();
    list<bool>::iterator   lbL = p_tracker->lostFlags.begin();
    for (list<Sophus::SE3f>::iterator
             lit  = p_tracker->relativeFramePoses.begin(),
             lend = p_tracker->relativeFramePoses.end();
         lit != lend;
         lit++, rits++, lT++, lbL++)
    {
        if (*lbL)
            continue;

        KeyFrame *p_keyFrame = *rits;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        while (p_keyFrame->isBad())
        {
            Trw        = Trw * p_keyFrame->tcp;
            p_keyFrame = p_keyFrame->getParent();
        }

        Trw = Trw * p_keyFrame->getPose() * Two;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f    twc = Twc.translation();
        Eigen::Quaternionf q   = Twc.unit_quaternion();

        f << setprecision(6) << *lT << " " << setprecision(9) << twc(0) << " "
          << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
          << q.z() << " " << q.w() << endl;
    }
    f.close();
}

} // namespace core
} // namespace vs_graphs
