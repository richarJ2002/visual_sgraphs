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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <list>
#include <set>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "ORBVocabulary.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>

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
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version)
    {
        ar & backupInvertedFileIds;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrameDatabase() {}
    KeyFrameDatabase(const ORBVocabulary &vocabulary_in) :
        p_vocabulary(&vocabulary_in)
    {
        invertedFile.resize(vocabulary_in.size());
    }

    void add(KeyFrame *p_keyFrame_in);

    void erase(KeyFrame *p_keyFrame_in);

    void clear();
    void clearMap(Map *p_map_in);

    // Loop Detection(DEPRECATED)
    std::vector<KeyFrame *> detectLoopCandidates(KeyFrame *p_currentKeyFrame_in,
                                                 float     minScore_in);

    // Loop and Merge Detection
    void detectCandidates(KeyFrame           *p_currentKeyFrame_in,
                          float               minScore_in,
                          vector<KeyFrame *> &loopCandidateKeyFrames_out,
                          vector<KeyFrame *> &mergeCandidateKeyFrames_out);
    void detectBestCandidates(KeyFrame           *p_currentKeyFrame_in,
                              vector<KeyFrame *> &loopCandidateKeyFrames_out,
                              vector<KeyFrame *> &mergeCandidateKeyFrames_out,
                              int                 minWordCount_in);
    void detectNBestCandidates(KeyFrame           *p_currentKeyFrame_in,
                               vector<KeyFrame *> &loopCandidateKeyFrames_out,
                               vector<KeyFrame *> &mergeCandidateKeyFrames_out,
                               int                 candidateCount_in);

    // Relocalization
    std::vector<KeyFrame *> detectRelocalizationCandidates(Frame *p_frame_in,
                                                           Map   *p_map_in);

    void preSave();
    void postLoad(map<long unsigned int, KeyFrame *> keyFrameId_in);
    void setORBVocabulary(ORBVocabulary *p_orbVocabulary_in);

  protected:
    // Associated vocabulary
    const ORBVocabulary *p_vocabulary;

    // Inverted file
    std::vector<list<KeyFrame *>> invertedFile;

    // For save relation without pointer, this is necessary for save/load
    // function
    std::vector<list<long unsigned int>> backupInvertedFileIds;

    // Mutex
    std::mutex databaseMutex;
};

} // namespace core
} // namespace vs_graphs

#endif
