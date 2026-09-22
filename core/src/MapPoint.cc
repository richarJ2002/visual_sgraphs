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
 * @file         MapPoint.cc
 *
 * @brief        Implements MapPoint declared in MapPoint.h.
 */

#include "MapPoint.h"
#include "ORBmatcher.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

long unsigned int MapPoint::nNextId = 0;
mutex             MapPoint::mGlobalMutex;

MapPoint::MapPoint() :
    firstKeyFrameId(0),
    firstFrameId(0),
    observationCount(0),
    trackReferenceFrameId(0),
    lastSeenFrameId(0),
    baLocalKeyFrameId(0),
    fuseCandidateKeyFrameId(0),
    loopPointKeyFrameId(0),
    correctedByKeyFrameId(0),
    correctedReferenceKeyFrameId(0),
    baGlobalKeyFrameId(0),
    visibleCount(1),
    foundCount(1),
    mbBad(false),
    p_replaced(static_cast<MapPoint *>(nullptr))
{
    p_replaced = static_cast<MapPoint *>(nullptr);
}

MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map *pMap) :
    firstKeyFrameId(pRefKF->mnId),
    firstFrameId(pRefKF->frameId),
    observationCount(0),
    trackReferenceFrameId(0),
    lastSeenFrameId(0),
    baLocalKeyFrameId(0),
    fuseCandidateKeyFrameId(0),
    loopPointKeyFrameId(0),
    correctedByKeyFrameId(0),
    correctedReferenceKeyFrameId(0),
    baGlobalKeyFrameId(0),
    originMapId(pMap->getId()),
    p_referenceKeyFrame(pRefKF),
    visibleCount(1),
    foundCount(1),
    mbBad(false),
    p_replaced(static_cast<MapPoint *>(nullptr)),
    minDistance(0),
    maxDistance(0),
    p_map(pMap)
{
    setWorldPos(Pos);

    normalVector.setZero();

    trackInViewR = false;
    trackInView  = false;

    // MapPoints can be created from Tracking and Local Mapping. This mutex
    // avoid conflicts with id.
    unique_lock<mutex> lock(p_map->mMutexPointCreation);
    mnId = nNextId++;
}

MapPoint::MapPoint(const double invDepth,
                   cv::Point2f  uv_init,
                   KeyFrame    *pRefKF,
                   KeyFrame    *pHostKF,
                   Map         *pMap) :
    firstKeyFrameId(pRefKF->mnId),
    firstFrameId(pRefKF->frameId),
    observationCount(0),
    trackReferenceFrameId(0),
    lastSeenFrameId(0),
    baLocalKeyFrameId(0),
    fuseCandidateKeyFrameId(0),
    loopPointKeyFrameId(0),
    correctedByKeyFrameId(0),
    correctedReferenceKeyFrameId(0),
    baGlobalKeyFrameId(0),
    originMapId(pMap->getId()),
    p_referenceKeyFrame(pRefKF),
    visibleCount(1),
    foundCount(1),
    mbBad(false),
    p_replaced(static_cast<MapPoint *>(nullptr)),
    minDistance(0),
    maxDistance(0),
    p_map(pMap)
{
    inverseDepth = invDepth;
    initU        = (double)uv_init.x;
    initV        = (double)uv_init.y;
    p_hostKF     = pHostKF;

    normalVector.setZero();

    // Worldpos is not set
    // MapPoints can be created from Tracking and Local Mapping. This mutex
    // avoid conflicts with id.
    unique_lock<mutex> lock(p_map->mMutexPointCreation);
    mnId = nNextId++;
}

MapPoint::MapPoint(const Eigen::Vector3f &Pos,
                   Map                   *pMap,
                   Frame                 *pFrame,
                   const int             &idxF) :
    firstKeyFrameId(-1),
    firstFrameId(pFrame->mnId),
    observationCount(0),
    trackReferenceFrameId(0),
    lastSeenFrameId(0),
    baLocalKeyFrameId(0),
    fuseCandidateKeyFrameId(0),
    loopPointKeyFrameId(0),
    correctedByKeyFrameId(0),
    correctedReferenceKeyFrameId(0),
    baGlobalKeyFrameId(0),
    originMapId(pMap->getId()),
    p_referenceKeyFrame(static_cast<KeyFrame *>(nullptr)),
    visibleCount(1),
    foundCount(1),
    mbBad(false),
    p_replaced(nullptr),
    p_map(pMap)
{
    setWorldPos(Pos);

    Eigen::Vector3f Ow;
    if (pFrame->Nleft == -1 || idxF < pFrame->Nleft)
    {
        Ow = pFrame->getCameraCenter();
    }
    else
    {
        Eigen::Matrix3f Rwl = pFrame->getRotationRwc();
        Eigen::Vector3f tlr = pFrame->getRelativePoseTlr().translation();
        Eigen::Vector3f twl = pFrame->getCenterOw();

        Ow = Rwl * tlr + twl;
    }
    normalVector = worldPos - Ow;
    normalVector = normalVector / normalVector.norm();

    Eigen::Vector3f PC   = worldPos - Ow;
    const float     dist = PC.norm();
    const int       level =
        (pFrame->Nleft == -1) ? pFrame->keyPointsUndistorted[idxF].octave
              : (idxF < pFrame->Nleft) ? pFrame->keyPoints[idxF].octave
                                       : pFrame->keyPointsRight[idxF].octave;
    const float levelScaleFactor = pFrame->scaleFactors[level];
    const int   nLevels          = pFrame->scaleLevelCount;

    maxDistance = dist * levelScaleFactor;
    minDistance = maxDistance / pFrame->scaleFactors[nLevels - 1];

    pFrame->descriptors.row(idxF).copyTo(descriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex
    // avoid conflicts with id.
    unique_lock<mutex> lock(p_map->mMutexPointCreation);
    mnId = nNextId++;
}

void MapPoint::setWorldPos(const Eigen::Vector3f &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    worldPos = Pos;
}

Eigen::Vector3f MapPoint::getWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return worldPos;
}

Eigen::Vector3f MapPoint::getNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return normalVector;
}

KeyFrame *MapPoint::getReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return p_referenceKeyFrame;
}

void MapPoint::addObservation(KeyFrame *pKF, int idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    tuple<int, int>    indexes;

    if (observations.count(pKF))
    {
        indexes = observations[pKF];
    }
    else
    {
        indexes = tuple<int, int>(-1, -1);
    }

    if (pKF->Nleft != -1 && idx >= pKF->Nleft)
    {
        get<1>(indexes) = idx;
    }
    else
    {
        get<0>(indexes) = idx;
    }

    observations[pKF] = indexes;

    if (!pKF->p_camera2 && pKF->uRight[idx] >= 0)
        observationCount += 2;
    else
        observationCount++;
}

void MapPoint::eraseObservation(KeyFrame *pKF)
{
    bool bBad = false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (observations.count(pKF))
        {
            tuple<int, int> indexes = observations[pKF];
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if (leftIndex != -1)
            {
                if (!pKF->p_camera2 && pKF->uRight[leftIndex] >= 0)
                    observationCount -= 2;
                else
                    observationCount--;
            }
            if (rightIndex != -1)
            {
                observationCount--;
            }

            observations.erase(pKF);

            if (p_referenceKeyFrame == pKF)
            {
                p_referenceKeyFrame = nullptr;

                for (const auto &[p_candidateKeyFrame, featureIndexes] :
                     observations)
                {
                    (void)featureIndexes;

                    if (p_candidateKeyFrame == nullptr ||
                        p_candidateKeyFrame->isBad())
                    {
                        continue;
                    }

                    if (p_referenceKeyFrame == nullptr ||
                        p_candidateKeyFrame->mnId < p_referenceKeyFrame->mnId)
                    {
                        p_referenceKeyFrame = p_candidateKeyFrame;
                    }
                }
            }

            // If only 2 observations or less, discard point
            if (observationCount <= 2)
                bBad = true;
        }
    }

    if (bBad)
        setBadFlag();
}

std::map<KeyFrame *, std::tuple<int, int>> MapPoint::getObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return observations;
}

int MapPoint::getObservationCount()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return observationCount;
}

void MapPoint::setBadFlag()
{
    map<KeyFrame *, tuple<int, int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad = true;
        obs   = observations;
        observations.clear();
    }
    for (map<KeyFrame *, tuple<int, int>>::iterator mit  = obs.begin(),
                                                    mend = obs.end();
         mit != mend;
         mit++)
    {
        KeyFrame *pKF = mit->first;
        int leftIndex = get<0>(mit->second), rightIndex = get<1>(mit->second);
        if (leftIndex != -1)
        {
            pKF->eraseMapPointMatch(leftIndex);
        }
        if (rightIndex != -1)
        {
            pKF->eraseMapPointMatch(rightIndex);
        }
    }

    p_map->eraseMapPoint(this);
}

MapPoint *MapPoint::getReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return p_replaced;
}

void MapPoint::replace(MapPoint *pMP)
{
    if (pMP->mnId == this->mnId)
        return;

    int                              nvisible, nfound;
    map<KeyFrame *, tuple<int, int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs = observations;
        observations.clear();
        mbBad      = true;
        nvisible   = visibleCount;
        nfound     = foundCount;
        p_replaced = pMP;
    }

    for (map<KeyFrame *, tuple<int, int>>::iterator mit  = obs.begin(),
                                                    mend = obs.end();
         mit != mend;
         mit++)
    {
        // Replace measurement in keyframe
        KeyFrame *pKF = mit->first;

        tuple<int, int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if (!pMP->isInKeyFrame(pKF))
        {
            if (leftIndex != -1)
            {
                pKF->replaceMapPointMatch(leftIndex, pMP);
                pMP->addObservation(pKF, leftIndex);
            }
            if (rightIndex != -1)
            {
                pKF->replaceMapPointMatch(rightIndex, pMP);
                pMP->addObservation(pKF, rightIndex);
            }
        }
        else
        {
            if (leftIndex != -1)
            {
                pKF->eraseMapPointMatch(leftIndex);
            }
            if (rightIndex != -1)
            {
                pKF->eraseMapPointMatch(rightIndex);
            }
        }
    }
    pMP->increaseFound(nfound);
    pMP->increaseVisible(nvisible);
    pMP->computeDistinctiveDescriptors();

    p_map->eraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock1(mMutexFeatures, std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos, std::defer_lock);
    lock(lock1, lock2);

    return mbBad;
}

void MapPoint::increaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    visibleCount += n;
}

void MapPoint::increaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    foundCount += n;
}

float MapPoint::getFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(foundCount) / visibleCount;
}

void MapPoint::computeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame *, tuple<int, int>> observedKeyFrames;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if (mbBad)
            return;
        observedKeyFrames = observations;
    }

    if (observedKeyFrames.empty())
        return;

    vDescriptors.reserve(observedKeyFrames.size());

    for (map<KeyFrame *, tuple<int, int>>::iterator
             mit  = observedKeyFrames.begin(),
             mend = observedKeyFrames.end();
         mit != mend;
         mit++)
    {
        KeyFrame *pKF = mit->first;

        if (!pKF->isBad())
        {
            tuple<int, int> indexes = mit->second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if (leftIndex != -1)
            {
                vDescriptors.push_back(pKF->descriptors.row(leftIndex));
            }
            if (rightIndex != -1)
            {
                vDescriptors.push_back(pKF->descriptors.row(rightIndex));
            }
        }
    }

    if (vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    // Symmetric N x N distance matrix held row-major, so row i occupies the
    // contiguous range [i * N, i * N + N).
    std::vector<float> Distances(N * N);
    for (size_t i = 0; i < N; i++)
    {
        Distances[i * N + i] = 0;
        for (size_t j = i + 1; j < N; j++)
        {
            int distij = ORBmatcher::computeDescriptorDistance(vDescriptors[i],
                                                               vDescriptors[j]);
            Distances[i * N + j] = distij;
            Distances[j * N + i] = distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx    = 0;
    for (size_t i = 0; i < N; i++)
    {
        const float *p_row = &Distances[i * N];
        vector<int>  vDists(p_row, p_row + N);
        sort(vDists.begin(), vDists.end());
        int median = vDists[0.5 * (N - 1)];

        if (median < BestMedian)
        {
            BestMedian = median;
            BestIdx    = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        descriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::getDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return descriptor.clone();
}

tuple<int, int> MapPoint::getIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if (observations.count(pKF))
        return observations[pKF];
    else
        return tuple<int, int>(-1, -1);
}

bool MapPoint::isInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (observations.count(pKF));
}

void MapPoint::updateNormalAndDepth()
{
    map<KeyFrame *, tuple<int, int>> observedKeyFrames;
    KeyFrame                        *pRefKF;
    Eigen::Vector3f                  Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if (mbBad)
            return;
        observedKeyFrames = observations;
        pRefKF            = p_referenceKeyFrame;
        Pos               = worldPos;
    }

    if (observedKeyFrames.empty())
        return;

    Eigen::Vector3f normal;
    normal.setZero();
    int n = 0;
    for (map<KeyFrame *, tuple<int, int>>::iterator
             mit  = observedKeyFrames.begin(),
             mend = observedKeyFrames.end();
         mit != mend;
         mit++)
    {
        KeyFrame *pKF = mit->first;

        tuple<int, int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if (leftIndex != -1)
        {
            Eigen::Vector3f Owi     = pKF->getCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal                  = normal + normali / normali.norm();
            n++;
        }
        if (rightIndex != -1)
        {
            Eigen::Vector3f Owi     = pKF->getRightCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal                  = normal + normali / normali.norm();
            n++;
        }
    }

    Eigen::Vector3f PC   = Pos - pRefKF->getCameraCenter();
    const float     dist = PC.norm();

    tuple<int, int> indexes   = observedKeyFrames[pRefKF];
    int             leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    int             level;
    if (pRefKF->Nleft == -1)
    {
        level = pRefKF->keyPointsUndistorted[leftIndex].octave;
    }
    else if (leftIndex != -1)
    {
        level = pRefKF->keyPoints[leftIndex].octave;
    }
    else
    {
        level = pRefKF->keyPointsRight[rightIndex - pRefKF->Nleft].octave;
    }

    // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor = pRefKF->scaleFactors[level];
    const int   nLevels          = pRefKF->scaleLevelCount;

    {
        unique_lock<mutex> lock3(mMutexPos);
        maxDistance  = dist * levelScaleFactor;
        minDistance  = maxDistance / pRefKF->scaleFactors[nLevels - 1];
        normalVector = normal / n;
    }
}

void MapPoint::setNormalVector(const Eigen::Vector3f &normal)
{
    unique_lock<mutex> lock3(mMutexPos);
    normalVector = normal;
}

float MapPoint::getMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * minDistance;
}

float MapPoint::getMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * maxDistance;
}

int MapPoint::predictScale(const float &currentDist, KeyFrame *pKF)
{
    if (currentDist == 0.0f)
        return 0;

    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = maxDistance / currentDist;
    }

    int nScale = ceil(log(ratio) / pKF->logScaleFactor);
    if (nScale < 0)
        nScale = 0;
    else if (nScale >= pKF->scaleLevelCount)
        nScale = pKF->scaleLevelCount - 1;

    return nScale;
}

int MapPoint::predictScale(const float &currentDist, Frame *pF)
{
    if (currentDist == 0.0f)
        return 0;

    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = maxDistance / currentDist;
    }

    int nScale = ceil(log(ratio) / pF->logScaleFactor);
    if (nScale < 0)
        nScale = 0;
    else if (nScale >= pF->scaleLevelCount)
        nScale = pF->scaleLevelCount - 1;

    return nScale;
}

void MapPoint::printObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    cout << "MP_OBS: MP " << mnId << endl;
    for (map<KeyFrame *, tuple<int, int>>::iterator mit  = observations.begin(),
                                                    mend = observations.end();
         mit != mend;
         mit++)
    {
        KeyFrame       *pKFi    = mit->first;
        tuple<int, int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        cout << "--OBS in KF " << pKFi->mnId << " in map "
             << pKFi->getMap()->getId() << endl;
    }
}

Map *MapPoint::getMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return p_map;
}

void MapPoint::updateMap(Map *pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    p_map = pMap;
}

void MapPoint::PreSave(set<KeyFrame *> &spKF, set<MapPoint *> &spMP)
{
    backupReplacedId = -1;

    backupObservationIds1.clear();
    backupObservationIds2.clear();

    // Snapshot the observation map and replaced pointer under the feature lock.
    // Dropped keyframes are erased below, after the lock is released, because
    // EraseObservation() takes mMutexFeatures again.
    std::map<KeyFrame *, std::tuple<int, int>> tmp_mObservations;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (p_replaced && spMP.find(p_replaced) != spMP.end())
            backupReplacedId = p_replaced->mnId;

        tmp_mObservations.insert(observations.begin(), observations.end());
    }

    for (std::map<KeyFrame *, std::tuple<int, int>>::const_iterator
             it  = tmp_mObservations.begin(),
             end = tmp_mObservations.end();
         it != end;
         ++it)
    {
        KeyFrame *pKFi = it->first;
        if (spKF.find(pKFi) != spKF.end())
        {
            backupObservationIds1[it->first->mnId] = get<0>(it->second);
            backupObservationIds2[it->first->mnId] = get<1>(it->second);
        }
        else
        {
            eraseObservation(pKFi);
        }
    }

    // Save the id of the reference KF
    unique_lock<mutex> lock(mMutexFeatures);
    if (spKF.find(p_referenceKeyFrame) != spKF.end())
    {
        backupRefKeyFrameId = p_referenceKeyFrame->mnId;
    }
}

void MapPoint::PostLoad(map<long unsigned int, KeyFrame *> &mpKFid,
                        map<long unsigned int, MapPoint *> &mpMPid)
{
    p_referenceKeyFrame = mpKFid[backupRefKeyFrameId];
    if (!p_referenceKeyFrame)
    {
        cout << "ERROR: MP without KF reference " << backupRefKeyFrameId
             << "; Num obs: " << observationCount << endl;
    }
    p_replaced = static_cast<MapPoint *>(nullptr);
    if (backupReplacedId >= 0)
    {
        map<long unsigned int, MapPoint *>::iterator it =
            mpMPid.find(backupReplacedId);
        if (it != mpMPid.end())
            p_replaced = it->second;
    }

    observations.clear();

    for (map<long unsigned int, int>::const_iterator
             it  = backupObservationIds1.begin(),
             end = backupObservationIds1.end();
         it != end;
         ++it)
    {
        KeyFrame                                   *pKFi = mpKFid[it->first];
        map<long unsigned int, int>::const_iterator it2 =
            backupObservationIds2.find(it->first);
        std::tuple<int, int> indexes = tuple<int, int>(it->second, it2->second);
        if (pKFi)
        {
            observations[pKFi] = indexes;
        }
    }

    backupObservationIds1.clear();
    backupObservationIds2.clear();
}

} // namespace core
} // namespace vs_graphs
