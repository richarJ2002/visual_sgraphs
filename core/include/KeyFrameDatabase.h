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

/*!
 * @brief           Inverted-file index from visual words of the ORB vocabulary
 *                  to the key frames that contain them. Loop closing, map
 *                  merging and relocalisation use it to find key frames that
 *                  look like a query by bag-of-words similarity.
 */
class KeyFrameDatabase
{
    friend class boost::serialization::access;

    /*!
     * @brief           Saves or loads backupInvertedFileIds.
     *
     * @param[in,out]   ar
     *                  Boost archive that is written to or read from.
     *
     * @param[in]       version
     *                  Archive class version; unused.
     */
    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates an empty database with no vocabulary and no word
     *                  list, as Boost does before loading one. clear() needs a
     *                  vocabulary, so call setORBVocabulary() first.
     */
    KeyFrameDatabase() {}
    /*!
     * @brief           Creates an empty database with one empty key frame list
     *                  per vocabulary word.
     *
     * @param[in]       vocabulary_in
     *                  Vocabulary that defines the words; only its address is
     *                  kept, so it must outlive the database.
     */
    KeyFrameDatabase(const ORBVocabulary &vocabulary_in) :
        p_vocabulary(&vocabulary_in)
    {
        invertedFile.resize(vocabulary_in.size());
    }

    /*!
     * @brief           Adds a key frame to the list of every visual word in its
     *                  bag-of-words vector. Takes databaseMutex.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame to add; borrowed, the map owns it.
     *
     * @return          KEY_FRAME_DATABASE_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameDatabaseStatus add(KeyFrame *p_keyFrame_in);

    /*!
     * @brief           Removes a key frame from the list of every visual word
     *                  in its bag-of-words vector. Takes databaseMutex.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame to remove; borrowed, it is not deleted.
     *
     * @return          KEY_FRAME_DATABASE_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameDatabaseStatus erase(KeyFrame *p_keyFrame_in);

    /*!
     * @brief           Empties every word list and keeps the vocabulary size.
     *                  Does not take databaseMutex.
     *
     * @return          KEY_FRAME_DATABASE_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameDatabaseStatus clear();
    /*!
     * @brief           Removes every key frame that belongs to one map from all
     *                  word lists. The key frames are not deleted. Takes
     *                  databaseMutex.
     *
     * @param[in]       p_map_in
     *                  Map whose key frames are removed; compared by address.
     *
     * @return          KEY_FRAME_DATABASE_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameDatabaseStatus clearMap(Map *p_map_in);

    // Loop Detection(DEPRECATED)
    /*!
     * @brief           Deprecated, see detectCandidates. Finds loop closure
     *                  candidates: key frames of the same map that share visual
     *                  words with the current key frame and score at least
     *                  minScore_in. Keeps candidates whose accumulated
     *                  similarity (own score plus covisible neighbours) is
     *                  above 0.75 times the best accumulated score.
     *
     * @param[in]       p_currentKeyFrame_in
     *                  Key frame to search for; borrowed. Candidates exclude
     *                  the key frames covisible with it.
     *
     * @param[in]       minScore_in
     *                  Minimum bag-of-words similarity score for a key frame to
     *                  be considered.
     *
     * @param[out]      loopCandidates_out
     *                  Replaced by the loop candidates; empty when none
     *                  qualify.
     *
     * @return          KEY_FRAME_DATABASE_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameDatabaseStatus
        detectLoopCandidates(KeyFrame                *p_currentKeyFrame_in,
                             float                    minScore_in,
                             std::vector<KeyFrame *> &loopCandidates_out);

    // Loop and Merge Detection
    /*!
     * @brief           Finds loop closure candidates (same map as the current
     *                  key frame) and merge candidates (another map that is not
     *                  bad) in one pass over the shared visual words. Each list
     *                  keeps candidates scoring at least minScore_in whose
     *                  accumulated similarity is above 0.75 times the best
     *                  accumulated score of that list.
     *
     * @param[in]       p_currentKeyFrame_in
     *                  Key frame to search for; borrowed. Candidates exclude
     *                  the key frames covisible with it.
     *
     * @param[in]       minScore_in
     *                  Minimum bag-of-words similarity score for a key frame to
     *                  be considered.
     *
     * @param[out]      loopCandidateKeyFrames_out
     *                  Loop candidates are appended; the vector is not cleared
     *                  first.
     *
     * @param[out]      mergeCandidateKeyFrames_out
     *                  Merge candidates are appended; the vector is not cleared
     *                  first.
     *
     * @return          KEY_FRAME_DATABASE_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameDatabaseStatus
        detectCandidates(KeyFrame                *p_currentKeyFrame_in,
                         float                    minScore_in,
                         std::vector<KeyFrame *> &loopCandidateKeyFrames_out,
                         std::vector<KeyFrame *> &mergeCandidateKeyFrames_out);
    /*!
     * @brief           Finds loop candidates (same map as the current key
     *                  frame) and merge candidates (any other map) among the
     *                  key frames that share enough visual words with it. Keeps
     *                  candidates whose accumulated similarity (own score plus
     *                  covisible neighbours) is above 0.75 times the best
     *                  accumulated score. Nothing is written when no key frame
     *                  shares a word.
     *
     * @param[in]       p_currentKeyFrame_in
     *                  Key frame to search for; borrowed. Candidates exclude
     *                  the key frames covisible with it.
     *
     * @param[out]      loopCandidateKeyFrames_out
     *                  Loop candidates are appended; the vector is not cleared
     *                  first.
     *
     * @param[out]      mergeCandidateKeyFrames_out
     *                  Merge candidates are appended; the vector is not cleared
     *                  first.
     *
     * @param[in]       minWordCount_in
     *                  Minimum number of shared words; raised to 0.8 times the
     *                  highest shared count when that is larger.
     *
     * @return          KEY_FRAME_DATABASE_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameDatabaseStatus detectBestCandidates(
        KeyFrame                *p_currentKeyFrame_in,
        std::vector<KeyFrame *> &loopCandidateKeyFrames_out,
        std::vector<KeyFrame *> &mergeCandidateKeyFrames_out,
        int                      minWordCount_in);
    /*!
     * @brief           Like detectBestCandidates, but ranks all candidates by
     *                  accumulated score, skips bad key frames, and returns up
     *                  to candidateCount_in loop candidates (same map as the
     *                  current key frame) and as many merge candidates (another
     *                  map that is not bad).
     *
     * @param[in]       p_currentKeyFrame_in
     *                  Key frame to search for; borrowed. Candidates exclude
     *                  the key frames covisible with it.
     *
     * @param[out]      loopCandidateKeyFrames_out
     *                  Loop candidates, best first, are appended; the vector is
     *                  not cleared first.
     *
     * @param[out]      mergeCandidateKeyFrames_out
     *                  Merge candidates, best first, are appended; the vector
     *                  is not cleared first.
     *
     * @param[in]       candidateCount_in
     *                  Maximum number of candidates per list; must not be
     *                  negative.
     *
     * @return          KEY_FRAME_DATABASE_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameDatabaseStatus detectNBestCandidates(
        KeyFrame                *p_currentKeyFrame_in,
        std::vector<KeyFrame *> &loopCandidateKeyFrames_out,
        std::vector<KeyFrame *> &mergeCandidateKeyFrames_out,
        int                      candidateCount_in);

    // Relocalization
    /*!
     * @brief           Finds key frames of one map that could re-localise a
     *                  lost frame: the key frames sharing enough visual words
     *                  with the frame. Keeps candidates whose accumulated
     *                  similarity (own score plus covisible neighbours) is
     *                  above 0.75 times the best accumulated score.
     *
     * @param[in]       p_frame_in
     *                  Frame to relocalise; borrowed.
     *
     * @param[in]       p_map_in
     *                  Only key frames of this map are returned; compared by
     *                  address.
     *
     * @param[out]      relocalizationCandidates_out
     *                  Replaced by the candidates; empty when none qualify.
     *
     * @return          KEY_FRAME_DATABASE_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameDatabaseStatus detectRelocalizationCandidates(
        Frame                   *p_frame_in,
        Map                     *p_map_in,
        std::vector<KeyFrame *> &relocalizationCandidates_out);

    /*!
     * @brief           Replaces the vocabulary and resets the inverted file to
     *                  one empty list per word, so every key frame must be
     *                  added again. Does not take databaseMutex.
     *
     * @param[in]       p_orbVocabulary_in
     *                  New vocabulary; only its address is kept, so it must
     *                  outlive the database.
     *
     * @return          KEY_FRAME_DATABASE_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameDatabaseStatus
        setORBVocabulary(ORBVocabulary *p_orbVocabulary_in);

  protected:
    /*!
     * @brief           Vocabulary that defines the visual words; borrowed,
     *                  nullptr until a vocabulary is given.
     */
    const ORBVocabulary *p_vocabulary{nullptr};

    /*!
     * @brief           For each visual word, the key frames whose bag-of-words
     *                  vector contains it; guarded by databaseMutex. The key
     *                  frames are owned by their maps.
     */
    std::vector<std::list<KeyFrame *>> invertedFile;

    /*!
     * @brief           Key frame ids per word that serialize saves in place of
     *                  the pointers in invertedFile.
     */
    std::vector<std::list<long unsigned int>> backupInvertedFileIds;

    /*!
     * @brief           Guards invertedFile in add, erase, clearMap and the
     *                  detect functions.
     */
    std::mutex databaseMutex;
};

} // namespace core
} // namespace vs_graphs

#endif
