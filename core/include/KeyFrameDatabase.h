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

/*!
 * @file            KeyFrameDatabase.h
 *
 * @brief           Declares KeyFrameDatabase, the bag-of-words index of key
 *                  frames used for relocalisation, loop detection and map
 *                  merging.
 */

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <boost/serialization/access.hpp>
#include <list>
#include <set>
#include <vector>

#include "KeyFrameDatabaseStatus.h"
#include "ORBVocabulary.h"

#include <Eigen/Core>
#include <mutex>

namespace vs_graphs
{
namespace core
{

class KeyFrame;
class Frame;
class Map;

class KeyFrameDatabase
{
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrameDatabase() {}
    KeyFrameDatabase(const ORBVocabulary &vocabulary_in) :
        p_vocabulary(&vocabulary_in)
    {
        invertedFile.resize(vocabulary_in.size());
    }

    [[nodiscard]] KeyFrameDatabaseStatus add(KeyFrame *p_keyFrame_in);

    [[nodiscard]] KeyFrameDatabaseStatus erase(KeyFrame *p_keyFrame_in);

    [[nodiscard]] KeyFrameDatabaseStatus clear();
    [[nodiscard]] KeyFrameDatabaseStatus clearMap(Map *p_map_in);

    // Loop Detection(DEPRECATED)
    [[nodiscard]] KeyFrameDatabaseStatus
        detectLoopCandidates(KeyFrame                *p_currentKeyFrame_in,
                             float                    minScore_in,
                             std::vector<KeyFrame *> &loopCandidates_out);

    // Loop and Merge Detection
    [[nodiscard]] KeyFrameDatabaseStatus
        detectCandidates(KeyFrame                *p_currentKeyFrame_in,
                         float                    minScore_in,
                         std::vector<KeyFrame *> &loopCandidateKeyFrames_out,
                         std::vector<KeyFrame *> &mergeCandidateKeyFrames_out);
    [[nodiscard]] KeyFrameDatabaseStatus detectBestCandidates(
        KeyFrame                *p_currentKeyFrame_in,
        std::vector<KeyFrame *> &loopCandidateKeyFrames_out,
        std::vector<KeyFrame *> &mergeCandidateKeyFrames_out,
        int                      minWordCount_in);
    [[nodiscard]] KeyFrameDatabaseStatus detectNBestCandidates(
        KeyFrame                *p_currentKeyFrame_in,
        std::vector<KeyFrame *> &loopCandidateKeyFrames_out,
        std::vector<KeyFrame *> &mergeCandidateKeyFrames_out,
        int                      candidateCount_in);

    // Relocalization
    [[nodiscard]] KeyFrameDatabaseStatus detectRelocalizationCandidates(
        Frame                   *p_frame_in,
        Map                     *p_map_in,
        std::vector<KeyFrame *> &relocalizationCandidates_out);

    [[nodiscard]] KeyFrameDatabaseStatus
        setORBVocabulary(ORBVocabulary *p_orbVocabulary_in);

  protected:
    // Associated vocabulary
    const ORBVocabulary *p_vocabulary;

    // Inverted file
    std::vector<std::list<KeyFrame *>> invertedFile;

    // For save relation without pointer, this is necessary for save/load
    // function
    std::vector<std::list<long unsigned int>> backupInvertedFileIds;

    // Mutex
    std::mutex databaseMutex;
};

} // namespace core
} // namespace vs_graphs

#endif
