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

#include "KeyFrame.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"
#include <mutex>

namespace vs_graphs
{
namespace core
{

long unsigned int KeyFrame::nNextId = 0;

KeyFrame::KeyFrame() :
    frameId(0),
    timeStamp(0),
    gridCols(FRAME_GRID_COLS),
    gridRows(FRAME_GRID_ROWS),
    gridElementWidthInverse(0),
    gridElementHeightInverse(0),
    trackReferenceFrameId(0),
    fuseTargetKeyFrameId(0),
    baLocalKeyFrameId(0),
    baFixedKeyFrameId(0),
    optimizationCount(0),
    loopQuery(0),
    loopWords(0),
    relocQuery(0),
    relocWords(0),
    mergeQuery(0),
    mergeWords(0),
    placeRecognitionQuery(0),
    placeRecognitionWords(0),
    placeRecognitionScore(0),
    currentPlaceRecognition(false),
    baGlobalKeyFrameId(0),
    mergeCorrectedKeyFrameId(0),
    baLocalMergeId(0),
    fx(0),
    fy(0),
    cx(0),
    cy(0),
    invfx(0),
    invfy(0),
    mbf(0),
    mb(0),
    depthThreshold(0),
    N(0),
    keyPoints(),
    keyPointsUndistorted(),
    uRight(),
    depths(),
    scaleLevelCount(0),
    scaleFactor(0),
    logScaleFactor(0),
    scaleFactors(0),
    levelSigmaSquared(0),
    invLevelSigmaSquared(0),
    gridMinX(0),
    gridMinY(0),
    gridMaxX(0),
    gridMaxY(0),
    p_prevKF(static_cast<KeyFrame *>(nullptr)),
    p_nextKF(static_cast<KeyFrame *>(nullptr)),
    velocityAvailable(false),
    firstConnection(true),
    p_parent(nullptr),
    notErase(false),
    toBeErased(false),
    mbBad(false),
    halfBaseline(0),
    Nleft(0),
    Nright(0)
{}

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB) :
    isImu(pMap->isImuInitialized()),
    frameId(F.mnId),
    timeStamp(F.timeStamp),
    gridCols(FRAME_GRID_COLS),
    gridRows(FRAME_GRID_ROWS),
    gridElementWidthInverse(F.gridElementWidthInverse),
    gridElementHeightInverse(F.gridElementHeightInverse),
    trackReferenceFrameId(0),
    fuseTargetKeyFrameId(0),
    baLocalKeyFrameId(0),
    baFixedKeyFrameId(0),
    optimizationCount(0),
    loopQuery(0),
    loopWords(0),
    relocQuery(0),
    relocWords(0),
    placeRecognitionQuery(0),
    placeRecognitionWords(0),
    placeRecognitionScore(0),
    currentPlaceRecognition(false),
    baGlobalKeyFrameId(0),
    mergeCorrectedKeyFrameId(0),
    baLocalMergeId(0),
    fx(F.fx),
    fy(F.fy),
    cx(F.cx),
    cy(F.cy),
    invfx(F.invfx),
    invfy(F.invfy),
    mbf(F.mbf),
    mb(F.mb),
    depthThreshold(F.depthThreshold),
    distortionCoefficients(F.distortionCoefficients),
    N(F.N),
    keyPoints(F.keyPoints),
    keyPointsUndistorted(F.keyPointsUndistorted),
    uRight(F.uRight),
    depths(F.depths),
    descriptors(F.descriptors.clone()),
    bowVector(F.bowVector),
    featureVector(F.featureVector),
    scaleLevelCount(F.scaleLevelCount),
    scaleFactor(F.scaleFactor),
    logScaleFactor(F.logScaleFactor),
    scaleFactors(F.scaleFactors),
    levelSigmaSquared(F.levelSigmaSquared),
    invLevelSigmaSquared(F.invLevelSigmaSquared),
    gridMinX(F.gridMinX),
    gridMinY(F.gridMinY),
    gridMaxX(F.gridMaxX),
    gridMaxY(F.gridMaxY),
    p_prevKF(nullptr),
    p_nextKF(nullptr),
    p_imuPreintegrated(F.p_imuPreintegrated),
    imuCalibration(F.imuCalibration),
    fileName(F.fileName),
    datasetId(F.datasetId),
    colorImg(F.colorImg),
    isPublished(false),
    velocityAvailable(false),
    poseTlr(F.getRelativePoseTlr()),
    poseTrl(F.getRelativePoseTrl()),
    mapPoints(F.mapPoints),
    p_keyFrameDatabase(pKFDB),
    p_orbVocabulary(F.p_orbVocabulary),
    firstConnection(true),
    p_parent(nullptr),
    notErase(false),
    toBeErased(false),
    mbBad(false),
    halfBaseline(F.mb / 2),
    currentFrameMarkers(F.mapMarkers),
    currentFrameMapPoints(F.mapPoints),
    currentFramePointClouds(F.pointClouds),
    p_map(pMap),
    calibrationMatrixEigen(F.calibrationMatrixEigen),
    p_camera(F.p_camera),
    p_camera2(F.p_camera2),
    leftToRightMatches(F.leftToRightMatches),
    rightToLeftMatches(F.rightToLeftMatches),
    keyPointsRight(F.keyPointsRight),
    Nleft(F.Nleft),
    Nright(F.Nright)
{
    mnId = nNextId++;

    grid.resize(gridCols);
    if (F.Nleft != -1)
        gridRight.resize(gridCols);
    for (int i = 0; i < gridCols; i++)
    {
        grid[i].resize(gridRows);
        if (F.Nleft != -1)
            gridRight[i].resize(gridRows);
        for (int j = 0; j < gridRows; j++)
        {
            grid[i][j] = F.grid[i][j];
            if (F.Nleft != -1)
            {
                gridRight[i][j] = F.gridRight[i][j];
            }
        }
    }

    if (!F.hasVelocity())
    {
        velocityVw.setZero();
        velocityAvailable = false;
    }
    else
    {
        velocityVw        = F.getVelocity();
        velocityAvailable = true;
    }

    imuBias = F.imuBias;
    setPose(F.getPose());

    originMapId = pMap->getId();
}

void KeyFrame::computeBagOfWords()
{
    if (bowVector.empty() || featureVector.empty())
    {
        vector<cv::Mat> vCurrentDesc =
            utils::converter::Converter::toDescriptorVector(descriptors);
        // Feature vector associate features with nodes in the 4th level (from
        // leaves up) We assume the vocabulary tree has 6 levels, change the 4
        // otherwise
        p_orbVocabulary->transform(vCurrentDesc, bowVector, featureVector, 4);
    }
}

void KeyFrame::setPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexPose);

    poseTcw     = Tcw;
    rotationRcw = poseTcw.rotationMatrix();
    twc         = poseTcw.inverse();
    rotationRwc = twc.rotationMatrix();

    if (imuCalibration.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
    {
        owb =
            rotationRwc * imuCalibration.mTcb.translation() + twc.translation();
    }
}

void KeyFrame::setVelocity(const Eigen::Vector3f &Vw)
{
    unique_lock<mutex> lock(mMutexPose);
    velocityVw        = Vw;
    velocityAvailable = true;
}

Sophus::SE3f KeyFrame::getPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return poseTcw;
}

Sophus::SE3f KeyFrame::getPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return twc;
}

Eigen::Vector3f KeyFrame::getCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return twc.translation();
}

Eigen::Vector3f KeyFrame::getImuPosition()
{
    unique_lock<mutex> lock(mMutexPose);
    return owb;
}

Eigen::Matrix3f KeyFrame::getImuRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return (twc * imuCalibration.mTcb).rotationMatrix();
}

Sophus::SE3f KeyFrame::getImuPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return twc * imuCalibration.mTcb;
}

Eigen::Matrix3f KeyFrame::getRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return rotationRcw;
}

Eigen::Vector3f KeyFrame::getTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return poseTcw.translation();
}

Eigen::Vector3f KeyFrame::getVelocity()
{
    unique_lock<mutex> lock(mMutexPose);
    return velocityVw;
}

bool KeyFrame::isVelocitySet()
{
    unique_lock<mutex> lock(mMutexPose);
    return velocityAvailable;
}

std::vector<semantic::Marker *> KeyFrame::getCurrentFrameMarkers() const
{
    return currentFrameMarkers;
}

std::vector<MapPoint *> KeyFrame::getCurrentFrameMapPoints() const
{
    return currentFrameMapPoints;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    KeyFrame::getCurrentFramePointCloud() const
{
    return currentFramePointClouds;
}

void KeyFrame::clearPointCloud()
{
    currentFramePointClouds->clear();
    currentFramePointClouds = nullptr;

    // clear images
    colorImg.release();
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
    KeyFrame::getClsCloudPtrs() const
{
    return currentClsCloudPtrs;
}

void KeyFrame::clearClsClouds()
{
    for (auto &clsCloud : currentClsCloudPtrs)
    {
        clsCloud->clear();
        clsCloud = nullptr;
    }
    currentClsCloudPtrs.clear();
}

void KeyFrame::setCurrentClsCloudPtrs(
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs)
{
    currentClsCloudPtrs = clsCloudPtrs;
}

void KeyFrame::addConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (!connectedKeyFrameWeights.count(pKF))
            connectedKeyFrameWeights[pKF] = weight;
        else if (connectedKeyFrameWeights[pKF] != weight)
            connectedKeyFrameWeights[pKF] = weight;
        else
            return;
    }

    updateBestCovisibles();
}

void KeyFrame::updateBestCovisibles()
{
    unique_lock<mutex>            lock(mMutexConnections);
    vector<pair<int, KeyFrame *>> vPairs;
    vPairs.reserve(connectedKeyFrameWeights.size());
    for (map<KeyFrame *, int>::iterator mit  = connectedKeyFrameWeights.begin(),
                                        mend = connectedKeyFrameWeights.end();
         mit != mend;
         mit++)
        vPairs.push_back(make_pair(mit->second, mit->first));

    sort(vPairs.begin(), vPairs.end());
    list<KeyFrame *> lKFs;
    list<int>        lWs;
    for (size_t i = 0, iend = vPairs.size(); i < iend; i++)
    {
        if (vPairs[i].second != nullptr)
        {
            if (!vPairs[i].second->isBad())
            {
                lKFs.push_front(vPairs[i].second);
                lWs.push_front(vPairs[i].first);
            }
        }
    }

    orderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
    orderedWeights            = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame *> KeyFrame::getConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame *>    s;
    for (map<KeyFrame *, int>::iterator mit = connectedKeyFrameWeights.begin();
         mit != connectedKeyFrameWeights.end();
         mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame *> KeyFrame::getVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return orderedConnectedKeyFrames;
}

vector<KeyFrame *> KeyFrame::getBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if ((int)orderedConnectedKeyFrames.size() < N)
        return orderedConnectedKeyFrames;
    else
        return vector<KeyFrame *>(orderedConnectedKeyFrames.begin(),
                                  orderedConnectedKeyFrames.begin() + N);
}

vector<KeyFrame *> KeyFrame::getCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if (orderedConnectedKeyFrames.empty())
    {
        return vector<KeyFrame *>();
    }

    vector<int>::iterator it = upper_bound(orderedWeights.begin(),
                                           orderedWeights.end(),
                                           w,
                                           KeyFrame::weightComp);

    if (it == orderedWeights.end() && orderedWeights.back() < w)
    {
        return vector<KeyFrame *>();
    }
    else
    {
        int n = it - orderedWeights.begin();
        return vector<KeyFrame *>(orderedConnectedKeyFrames.begin(),
                                  orderedConnectedKeyFrames.begin() + n);
    }
}

int KeyFrame::getWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if (connectedKeyFrameWeights.count(pKF))
        return connectedKeyFrameWeights[pKF];
    else
        return 0;
}

int KeyFrame::getMapPointCount()
{
    unique_lock<mutex> lock(mMutexFeatures);
    int                numberMPs = 0;
    for (size_t i = 0, iend = mapPoints.size(); i < iend; i++)
    {
        if (!mapPoints[i])
            continue;
        numberMPs++;
    }
    return numberMPs;
}

void KeyFrame::addMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mapPoints[idx] = pMP;
}

void KeyFrame::addMapMarker(semantic::Marker *marker)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mapMarkers.push_back(marker);
}

void KeyFrame::addMapPlane(geometric::Plane *plane)
{
    if (plane == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mMutexFeatures);

    if (std::find(mapPlanes.begin(), mapPlanes.end(), plane) == mapPlanes.end())
    {
        mapPlanes.push_back(plane);
    }
}

void KeyFrame::removeMapPlane(geometric::Plane *plane)
{
    unique_lock<mutex> lock(mMutexFeatures);

    if (!plane)
    {
        std::cerr << "ERROR: KeyFrame::RemoveMapPlane: plane is NULL"
                  << std::endl;
        return;
    }

    mapPlanes.erase(std::remove(mapPlanes.begin(), mapPlanes.end(), plane),
                    mapPlanes.end());
}

bool KeyFrame::replaceMapPlane(geometric::Plane *p_retiredPlane_in,
                               geometric::Plane *p_retainedPlane_in)
{
    if (p_retiredPlane_in == nullptr || p_retainedPlane_in == nullptr ||
        p_retiredPlane_in == p_retainedPlane_in)
    {
        return false;
    }

    unique_lock<mutex> lock(mMutexFeatures);

    bool                            replacedRetiredPlane = false;
    std::vector<geometric::Plane *> rebuiltPlanes;
    rebuiltPlanes.reserve(mapPlanes.size());

    for (geometric::Plane *p_existingPlane : mapPlanes)
    {
        geometric::Plane *p_candidatePlane = p_existingPlane;

        if (p_existingPlane == p_retiredPlane_in)
        {
            p_candidatePlane     = p_retainedPlane_in;
            replacedRetiredPlane = true;
        }

        if (p_candidatePlane == nullptr ||
            std::find(rebuiltPlanes.begin(),
                      rebuiltPlanes.end(),
                      p_candidatePlane) != rebuiltPlanes.end())
        {
            continue;
        }

        rebuiltPlanes.push_back(p_candidatePlane);
    }

    if (replacedRetiredPlane)
    {
        mapPlanes.swap(rebuiltPlanes);
    }

    return replacedRetiredPlane;
}

void vs_graphs::core::KeyFrame::addMapPassage(
    vs_graphs::core::semantic::Passage *p_passage_in)
{
    if (p_passage_in == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mMutexFeatures);

    if (std::find(mapPassages.begin(), mapPassages.end(), p_passage_in) ==
        mapPassages.end())
    {
        mapPassages.push_back(p_passage_in);
    }
}

bool vs_graphs::core::KeyFrame::replaceMapPassage(
    vs_graphs::core::semantic::Passage *p_retiredPassage_in,
    vs_graphs::core::semantic::Passage *p_retainedPassage_in)
{
    if (p_retiredPassage_in == nullptr || p_retainedPassage_in == nullptr ||
        p_retiredPassage_in == p_retainedPassage_in)
    {
        return false;
    }

    unique_lock<mutex> lock(mMutexFeatures);

    bool replacedAssociation = false;
    std::vector<vs_graphs::core::semantic::Passage *> rebuiltPassages;
    rebuiltPassages.reserve(mapPassages.size());

    for (vs_graphs::core::semantic::Passage *p_existingPassage : mapPassages)
    {
        vs_graphs::core::semantic::Passage *p_candidatePassage =
            p_existingPassage;

        if (p_existingPassage == p_retiredPassage_in)
        {
            p_candidatePassage  = p_retainedPassage_in;
            replacedAssociation = true;
        }

        if (p_candidatePassage == nullptr ||
            std::find(rebuiltPassages.begin(),
                      rebuiltPassages.end(),
                      p_candidatePassage) != rebuiltPassages.end())
        {
            continue;
        }

        rebuiltPassages.push_back(p_candidatePassage);
    }

    if (replacedAssociation)
    {
        mapPassages.swap(rebuiltPassages);
    }

    return replacedAssociation;
}

void KeyFrame::eraseMapPointMatch(const int &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mapPoints[idx] = static_cast<MapPoint *>(nullptr);
}

void KeyFrame::eraseMapPointMatch(MapPoint *pMP)
{
    tuple<int, int> indexes   = pMP->getIndexInKeyFrame(this);
    int             leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    if (leftIndex != -1)
        mapPoints[leftIndex] = static_cast<MapPoint *>(nullptr);
    if (rightIndex != -1)
        mapPoints[rightIndex] = static_cast<MapPoint *>(nullptr);
}

void KeyFrame::replaceMapPointMatch(const int &idx, MapPoint *pMP)
{
    mapPoints[idx] = pMP;
}

set<MapPoint *> KeyFrame::getMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);

    /* Init vector of map points */
    set<MapPoint *> s;

    /* Iterate through map points and move them over to list if valid */
    for (size_t i = 0, iend = mapPoints.size(); i < iend; i++)
    {
        /* If map point is invalid, skip */
        if (!mapPoints[i])
        {
            continue;
        }

        /* Extract map point from list */
        MapPoint *pMP = mapPoints[i];

        /* If point is determined to be bad, skip */
        if (!pMP->isBad())
        {
            s.insert(pMP);
        }
    }

    return s;
}

int KeyFrame::getTrackedMapPointCount(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int        nPoints   = 0;
    const bool bCheckObs = minObs > 0;
    for (int i = 0; i < N; i++)
    {
        MapPoint *pMP = mapPoints[i];
        if (pMP)
        {
            if (!pMP->isBad())
            {
                if (bCheckObs)
                {
                    if (mapPoints[i]->getObservationCount() >= minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint *> KeyFrame::getMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mapPoints;
}

MapPoint *KeyFrame::getMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mapPoints[idx];
}

vector<semantic::Marker *> KeyFrame::getMapMarkers()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mapMarkers;
}

vector<geometric::Plane *> KeyFrame::getMapPlanes()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mapPlanes;
}

std::vector<vs_graphs::core::semantic::Passage *>
    vs_graphs::core::KeyFrame::getMapPassages()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mapPassages;
}

void KeyFrame::updateConnections(bool upParent)
{
    map<KeyFrame *, int> KFcounter;

    vector<MapPoint *> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mapPoints;
    }

    // for all plane observations in the keyframe check in which other keyframes
    // are they seen increase counter for those keyframes
    if (types::SystemParams::getParams()->planeBasedCovisibility.enabled)
    {
        unsigned int scorePerPlane = types::SystemParams::getParams()
                                         ->planeBasedCovisibility.scorePerPlane;
        for (vector<geometric::Plane *>::iterator vit  = mapPlanes.begin(),
                                                  vend = mapPlanes.end();
             vit != vend;
             vit++)
        {
            geometric::Plane *pPlane = *vit;

            if (!pPlane)
                continue;

            map<KeyFrame *, vs_graphs::core::geometric::Plane::Observation>
                observations = pPlane->getObservations();

            for (map<KeyFrame *,
                     vs_graphs::core::geometric::Plane::Observation>::iterator
                     mit  = observations.begin(),
                     mend = observations.end();
                 mit != mend;
                 mit++)
            {
                if (mit->first->mnId == mnId || mit->first->isBad() ||
                    mit->first->getMap() != p_map)
                    continue;

                if (pPlane->getPlaneType() ==
                    geometric::Plane::PlaneVariant::UNDEFINED)
                    KFcounter[mit->first] += static_cast<int>(
                        scorePerPlane *
                        0.2); // undefined planes have less weight
                else
                    KFcounter[mit->first] += scorePerPlane;
            }
        }
    }

    // For all map points in keyframe check in which other keyframes are they
    // seen Increase counter for those keyframes
    for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end();
         vit != vend;
         vit++)
    {
        MapPoint *pMP = *vit;

        if (!pMP)
            continue;

        if (pMP->isBad())
            continue;

        map<KeyFrame *, tuple<int, int>> observations = pMP->getObservations();

        for (map<KeyFrame *, tuple<int, int>>::iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            if (mit->first->mnId == mnId || mit->first->isBad() ||
                mit->first->getMap() != p_map)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if (KFcounter.empty())
        return;

    // If the counter is greater than threshold add connection
    // In case no keyframe counter is over threshold add the one with maximum
    // counter
    int       nmax   = 0;
    KeyFrame *pKFmax = nullptr;
    int       th     = 15;

    vector<pair<int, KeyFrame *>> vPairs;
    vPairs.reserve(KFcounter.size());
    if (!upParent)
        cout << "UPDATE_CONN: current KF " << mnId << endl;
    for (map<KeyFrame *, int>::iterator mit  = KFcounter.begin(),
                                        mend = KFcounter.end();
         mit != mend;
         mit++)
    {
        if (!upParent)
            cout << "  UPDATE_CONN: KF " << mit->first->mnId
                 << " ; num matches: " << mit->second << endl;
        if (mit->second > nmax)
        {
            nmax   = mit->second;
            pKFmax = mit->first;
        }
        if (mit->second >= th)
        {
            vPairs.push_back(make_pair(mit->second, mit->first));
            (mit->first)->addConnection(this, mit->second);
        }
    }

    if (vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax, pKFmax));
        pKFmax->addConnection(this, nmax);
    }

    sort(vPairs.begin(), vPairs.end());
    list<KeyFrame *> lKFs;
    list<int>        lWs;
    for (size_t i = 0; i < vPairs.size(); i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        connectedKeyFrameWeights = KFcounter;
        orderedConnectedKeyFrames =
            vector<KeyFrame *>(lKFs.begin(), lKFs.end());
        orderedWeights = vector<int>(lWs.begin(), lWs.end());

        if (firstConnection && mnId != p_map->getInitKeyFrameId())
        {
            p_parent = orderedConnectedKeyFrames.front();
            p_parent->addChild(this);
            firstConnection = false;
        }
    }
}

void KeyFrame::addChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    childrens.insert(pKF);
}

void KeyFrame::eraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    childrens.erase(pKF);
}

void KeyFrame::changeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    if (pKF == this)
    {
        cout << "ERROR: Change parent KF, the parent and child are the same KF"
             << endl;
        throw std::invalid_argument("The parent and child can not be the same");
    }

    p_parent = pKF;
    pKF->addChild(this);
}

set<KeyFrame *> KeyFrame::getChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return childrens;
}

KeyFrame *KeyFrame::getParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return p_parent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return childrens.count(pKF);
}

void KeyFrame::setFirstConnection(bool bFirst)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    firstConnection = bFirst;
}

void KeyFrame::addLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    notErase = true;
    loopEdges.insert(pKF);
}

set<KeyFrame *> KeyFrame::getLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return loopEdges;
}

void KeyFrame::addMergeEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    notErase = true;
    mergeEdges.insert(pKF);
}

set<KeyFrame *> KeyFrame::getMergeEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mergeEdges;
}

void KeyFrame::setNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    notErase = true;
}

void KeyFrame::setErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (loopEdges.empty())
        {
            notErase = false;
        }
    }

    if (toBeErased)
    {
        setBadFlag();
    }
}

void KeyFrame::setBadFlag()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (mnId == p_map->getInitKeyFrameId())
        {
            return;
        }
        else if (notErase)
        {
            toBeErased = true;
            return;
        }
    }

    for (map<KeyFrame *, int>::iterator mit  = connectedKeyFrameWeights.begin(),
                                        mend = connectedKeyFrameWeights.end();
         mit != mend;
         mit++)
    {
        mit->first->eraseConnection(this);
    }

    /*
     * Semantic observations store non-owning KeyFrame pointers. Detach this
     * keyframe before LocalMapping is allowed to delete it; otherwise a later
     * merge, GBA, or observation insertion can dereference freed memory.
     */
    const std::vector<geometric::Plane *> observedPlanes = getMapPlanes();
    for (geometric::Plane *p_plane : observedPlanes)
    {
        if (p_plane != nullptr)
        {
            p_plane->eraseObservation(this);
        }
    }

    const std::vector<semantic::Marker *> observedMarkers = getMapMarkers();
    for (semantic::Marker *p_marker : observedMarkers)
    {
        if (p_marker != nullptr)
        {
            p_marker->eraseObservation(this);
        }
    }

    for (size_t i = 0; i < mapPoints.size(); i++)
    {
        if (mapPoints[i])
        {
            mapPoints[i]->eraseObservation(this);
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        connectedKeyFrameWeights.clear();
        orderedConnectedKeyFrames.clear();
        mapPlanes.clear();
        mapMarkers.clear();

        // Update Spanning Tree
        set<KeyFrame *> sParentCandidates;
        if (p_parent)
            sParentCandidates.insert(p_parent);

        // Assign at each iteration one children with a parent (the pair with
        // highest covisibility weight) Include that children as new parent
        // candidate for the rest
        while (!childrens.empty())
        {
            bool bContinue = false;

            int       max = -1;
            KeyFrame *pC;
            KeyFrame *pP;

            for (set<KeyFrame *>::iterator sit  = childrens.begin(),
                                           send = childrens.end();
                 sit != send;
                 sit++)
            {
                KeyFrame *pKF = *sit;
                if (pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame *> vpConnected =
                    pKF->getVectorCovisibleKeyFrames();
                for (size_t i = 0, iend = vpConnected.size(); i < iend; i++)
                {
                    for (set<KeyFrame *>::iterator
                             spcit  = sParentCandidates.begin(),
                             spcend = sParentCandidates.end();
                         spcit != spcend;
                         spcit++)
                    {
                        if (vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->getWeight(vpConnected[i]);
                            if (w > max)
                            {
                                pC        = pKF;
                                pP        = vpConnected[i];
                                max       = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if (bContinue)
            {
                pC->changeParent(pP);
                sParentCandidates.insert(pC);
                childrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate,
        // assign to the original parent of this KF
        if (!childrens.empty())
        {
            for (set<KeyFrame *>::iterator sit = childrens.begin();
                 sit != childrens.end();
                 sit++)
            {
                (*sit)->changeParent(p_parent);
            }
        }

        if (p_parent)
        {
            p_parent->eraseChild(this);
            tcp = poseTcw * p_parent->getPoseInverse();
        }
        mbBad = true;
    }

    p_map->eraseKeyFrame(this);
    p_keyFrameDatabase->erase(this);
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::eraseConnection(KeyFrame *pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if (connectedKeyFrameWeights.count(pKF))
        {
            connectedKeyFrameWeights.erase(pKF);
            bUpdate = true;
        }
    }

    if (bUpdate)
        updateBestCovisibles();
}

vector<size_t> KeyFrame::getFeaturesInArea(const float &x,
                                           const float &y,
                                           const float &r,
                                           const bool   bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX =
        max(0, (int)floor((x - gridMinX - factorX) * gridElementWidthInverse));
    if (nMinCellX >= gridCols)
        return vIndices;

    const int nMaxCellX =
        min((int)gridCols - 1,
            (int)ceil((x - gridMinX + factorX) * gridElementWidthInverse));
    if (nMaxCellX < 0)
        return vIndices;

    const int nMinCellY =
        max(0, (int)floor((y - gridMinY - factorY) * gridElementHeightInverse));
    if (nMinCellY >= gridRows)
        return vIndices;

    const int nMaxCellY =
        min((int)gridRows - 1,
            (int)ceil((y - gridMinY + factorY) * gridElementHeightInverse));
    if (nMaxCellY < 0)
        return vIndices;

    for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
    {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
        {
            const vector<size_t> vCell =
                (!bRight) ? grid[ix][iy] : gridRight[ix][iy];
            for (size_t j = 0, jend = vCell.size(); j < jend; j++)
            {
                const cv::KeyPoint &kpUn =
                    (Nleft == -1) ? keyPointsUndistorted[vCell[j]]
                    : (!bRight)   ? keyPoints[vCell[j]]
                                  : keyPointsRight[vCell[j]];
                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if (fabs(distx) < r && fabs(disty) < r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::isInImage(const float &x, const float &y) const
{
    return (x >= gridMinX && x < gridMaxX && y >= gridMinY && y < gridMaxY);
}

bool KeyFrame::unprojectStereo(int i, Eigen::Vector3f &x3D)
{
    const float z = depths[i];
    if (z > 0)
    {
        const float     u = keyPoints[i].pt.x;
        const float     v = keyPoints[i].pt.y;
        const float     x = (u - cx) * z * invfx;
        const float     y = (v - cy) * z * invfy;
        Eigen::Vector3f x3Dc(x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        x3D = rotationRwc * x3Dc + twc.translation();
        return true;
    }
    else
        return false;
}

float KeyFrame::computeSceneMedianDepth(const int q)
{
    if (N == 0)
        return -1.0;

    vector<MapPoint *> vpMapPoints;
    Eigen::Matrix3f    Rcw;
    Eigen::Vector3f    tcw;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mapPoints;
        tcw         = poseTcw.translation();
        Rcw         = rotationRcw;
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    Eigen::Matrix<float, 1, 3> Rcw2 = Rcw.row(2);
    float                      zcw  = tcw(2);
    for (int i = 0; i < N; i++)
    {
        if (mapPoints[i])
        {
            MapPoint       *pMP  = mapPoints[i];
            Eigen::Vector3f x3Dw = pMP->getWorldPos();
            float           z    = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(), vDepths.end());

    return vDepths[(vDepths.size() - 1) / q];
}

void KeyFrame::setNewBias(const IMU::Bias &b)
{
    unique_lock<mutex> lock(mMutexPose);
    imuBias = b;
    if (p_imuPreintegrated)
        p_imuPreintegrated->setNewBias(b);
}

Eigen::Vector3f KeyFrame::getGyroBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(imuBias.bwx, imuBias.bwy, imuBias.bwz);
}

Eigen::Vector3f KeyFrame::getAccBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(imuBias.bax, imuBias.bay, imuBias.baz);
}

IMU::Bias KeyFrame::getImuBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return imuBias;
}

Map *KeyFrame::getMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return p_map;
}

void KeyFrame::updateMap(Map *pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    p_map = pMap;
}

void KeyFrame::PreSave(
    set<KeyFrame *>                                        &spKF,
    set<MapPoint *>                                        &spMP,
    set<camera_models::geometriccamera::GeometricCamera *> &spCam)
{
    // Save the id of each MapPoint in this KF, there can be null pointer in the
    // vector
    backupMapPointsId.clear();
    backupMapPointsId.reserve(N);
    for (int i = 0; i < N; ++i)
    {

        if (mapPoints[i] && spMP.find(mapPoints[i]) !=
                                spMP.end()) // Checks if the element is not null
            backupMapPointsId.push_back(mapPoints[i]->mnId);
        else // If the element is null his value is -1 because all the id are
             // positives
            backupMapPointsId.push_back(-1);
    }
    // Save the id of each connected KF with it weight
    backupConnectedKeyFrameIdWeights.clear();
    for (std::map<KeyFrame *, int>::const_iterator
             it  = connectedKeyFrameWeights.begin(),
             end = connectedKeyFrameWeights.end();
         it != end;
         ++it)
    {
        if (spKF.find(it->first) != spKF.end())
            backupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
    }

    // Save the parent id
    backupParentId = -1;
    if (p_parent && spKF.find(p_parent) != spKF.end())
        backupParentId = p_parent->mnId;

    // Save the id of the childrens KF
    backupChildrensId.clear();
    backupChildrensId.reserve(childrens.size());
    for (KeyFrame *pKFi : childrens)
    {
        if (spKF.find(pKFi) != spKF.end())
            backupChildrensId.push_back(pKFi->mnId);
    }

    // Save the id of the loop edge KF
    backupLoopEdgesId.clear();
    backupLoopEdgesId.reserve(loopEdges.size());
    for (KeyFrame *pKFi : loopEdges)
    {
        if (spKF.find(pKFi) != spKF.end())
            backupLoopEdgesId.push_back(pKFi->mnId);
    }

    // Save the id of the merge edge KF
    backupMergeEdgesId.clear();
    backupMergeEdgesId.reserve(mergeEdges.size());
    for (KeyFrame *pKFi : mergeEdges)
    {
        if (spKF.find(pKFi) != spKF.end())
            backupMergeEdgesId.push_back(pKFi->mnId);
    }

    // Camera data
    backupCameraId = -1;
    if (p_camera && spCam.find(p_camera) != spCam.end())
        backupCameraId = p_camera->getId();

    backupCamera2Id = -1;
    if (p_camera2 && spCam.find(p_camera2) != spCam.end())
        backupCamera2Id = p_camera2->getId();

    // Inertial data
    backupPrevKFId = -1;
    if (p_prevKF && spKF.find(p_prevKF) != spKF.end())
        backupPrevKFId = p_prevKF->mnId;

    backupNextKFId = -1;
    if (p_nextKF && spKF.find(p_nextKF) != spKF.end())
        backupNextKFId = p_nextKF->mnId;

    if (p_imuPreintegrated)
        backupImuPreintegrated.copyFrom(p_imuPreintegrated);
}

void KeyFrame::PostLoad(
    map<long unsigned int, KeyFrame *> &mpKFid,
    map<long unsigned int, MapPoint *> &mpMPid,
    map<unsigned int, camera_models::geometriccamera::GeometricCamera *>
        &mpCamId)
{
    // Rebuild the empty variables

    // Pose
    setPose(poseTcw);

    poseTrl = poseTlr.inverse();

    // Reference reconstruction
    // Each MapPoint sight from this KeyFrame
    mapPoints.clear();
    mapPoints.resize(N);
    for (int i = 0; i < N; ++i)
    {
        if (backupMapPointsId[i] != -1)
            mapPoints[i] = mpMPid[backupMapPointsId[i]];
        else
            mapPoints[i] = static_cast<MapPoint *>(nullptr);
    }

    // Conected KeyFrames with him weight
    connectedKeyFrameWeights.clear();
    for (map<long unsigned int, int>::const_iterator
             it  = backupConnectedKeyFrameIdWeights.begin(),
             end = backupConnectedKeyFrameIdWeights.end();
         it != end;
         ++it)
    {
        KeyFrame *pKFi                 = mpKFid[it->first];
        connectedKeyFrameWeights[pKFi] = it->second;
    }

    // Restore parent KeyFrame
    if (backupParentId >= 0)
        p_parent = mpKFid[backupParentId];

    // KeyFrame childrens
    childrens.clear();
    for (vector<long unsigned int>::const_iterator
             it  = backupChildrensId.begin(),
             end = backupChildrensId.end();
         it != end;
         ++it)
    {
        childrens.insert(mpKFid[*it]);
    }

    // Loop edge KeyFrame
    loopEdges.clear();
    for (vector<long unsigned int>::const_iterator
             it  = backupLoopEdgesId.begin(),
             end = backupLoopEdgesId.end();
         it != end;
         ++it)
    {
        loopEdges.insert(mpKFid[*it]);
    }

    // Merge edge KeyFrame
    mergeEdges.clear();
    for (vector<long unsigned int>::const_iterator
             it  = backupMergeEdgesId.begin(),
             end = backupMergeEdgesId.end();
         it != end;
         ++it)
    {
        mergeEdges.insert(mpKFid[*it]);
    }

    // Camera data
    if (backupCameraId >= 0)
    {
        p_camera = mpCamId[backupCameraId];
    }
    else
    {
        cout << "ERROR: There is not a main camera in KF " << mnId << endl;
    }
    if (backupCamera2Id >= 0)
    {
        p_camera2 = mpCamId[backupCamera2Id];
    }

    // Inertial data
    if (backupPrevKFId != -1)
    {
        p_prevKF = mpKFid[backupPrevKFId];
    }
    if (backupNextKFId != -1)
    {
        p_nextKF = mpKFid[backupNextKFId];
    }
    p_imuPreintegrated = &backupImuPreintegrated;

    // Remove all backup container
    backupMapPointsId.clear();
    backupConnectedKeyFrameIdWeights.clear();
    backupChildrensId.clear();
    backupLoopEdgesId.clear();

    updateBestCovisibles();
}

bool KeyFrame::projectPointDistort(MapPoint    *pMP,
                                   cv::Point2f &kp,
                                   float       &u,
                                   float       &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->getWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc  = rotationRcw * P + poseTcw.translation();
    float          &PcX = Pc(0);
    float          &PcY = Pc(1);
    float          &PcZ = Pc(2);

    // Check positive depth
    if (PcZ < 0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    float invz = 1.0f / PcZ;
    u          = fx * PcX * invz + cx;
    v          = fy * PcY * invz + cy;

    // cout << "c";

    if (u < gridMinX || u > gridMaxX)
        return false;
    if (v < gridMinY || v > gridMaxY)
        return false;

    float x  = (u - cx) * invfx;
    float y  = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = distortionCoefficients.at<float>(0);
    float k2 = distortionCoefficients.at<float>(1);
    float p1 = distortionCoefficients.at<float>(2);
    float p2 = distortionCoefficients.at<float>(3);
    float k3 = 0;
    if (distortionCoefficients.total() == 5)
    {
        k3 = distortionCoefficients.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    float u_distort = x_distort * fx + cx;
    float v_distort = y_distort * fy + cy;

    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

bool KeyFrame::projectPointUnDistort(MapPoint    *pMP,
                                     cv::Point2f &kp,
                                     float       &u,
                                     float       &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->getWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc  = rotationRcw * P + poseTcw.translation();
    float          &PcX = Pc(0);
    float          &PcY = Pc(1);
    float          &PcZ = Pc(2);

    // Check positive depth
    if (PcZ < 0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f / PcZ;
    u                = fx * PcX * invz + cx;
    v                = fy * PcY * invz + cy;

    if (u < gridMinX || u > gridMaxX)
        return false;
    if (v < gridMinY || v > gridMaxY)
        return false;

    kp = cv::Point2f(u, v);

    return true;
}

Sophus::SE3f KeyFrame::getRelativePoseTrl()
{
    unique_lock<mutex> lock(mMutexPose);
    return poseTrl;
}

Sophus::SE3f KeyFrame::getRelativePoseTlr()
{
    unique_lock<mutex> lock(mMutexPose);
    return poseTlr;
}

Sophus::SE3<float> KeyFrame::getRightPose()
{
    unique_lock<mutex> lock(mMutexPose);

    return poseTrl * poseTcw;
}

Sophus::SE3<float> KeyFrame::getRightPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);

    return twc * poseTlr;
}

Eigen::Vector3f KeyFrame::getRightCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);

    return (twc * poseTlr).translation();
}

Eigen::Matrix<float, 3, 3> KeyFrame::getRightRotation()
{
    unique_lock<mutex> lock(mMutexPose);

    return (poseTrl.so3() * poseTcw.so3()).matrix();
}

Eigen::Vector3f KeyFrame::getRightTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return (poseTrl * poseTcw).translation();
}

void KeyFrame::setORBVocabulary(ORBVocabulary *pORBVoc)
{
    p_orbVocabulary = pORBVoc;
}

void KeyFrame::setKeyFrameDatabase(KeyFrameDatabase *pKFDB)
{
    p_keyFrameDatabase = pKFDB;
}

} // namespace core
} // namespace vs_graphs
