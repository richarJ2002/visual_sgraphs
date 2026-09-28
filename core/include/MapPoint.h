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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Utils/Converter/objects/Converter.h"

#include "SerializationUtils.h"

#include <mutex>
#include <opencv2/core/core.hpp>

#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>

namespace vs_graphs
{
namespace core
{

class KeyFrame;
class Map;
class Frame;

class MapPoint
{

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & id;
        ar & firstKeyFrameId;
        ar & firstFrameId;
        ar & observationCount;
        // Variables used by the tracking
        // ar & mTrackProjX;
        // ar & mTrackProjY;
        // ar & mTrackDepth;
        // ar & mTrackDepthR;
        // ar & mTrackProjXR;
        // ar & mTrackProjYR;
        // ar & mbTrackInView;
        // ar & mbTrackInViewR;
        // ar & mnTrackScaleLevel;
        // ar & mnTrackScaleLevelR;
        // ar & mTrackViewCos;
        // ar & mTrackViewCosR;
        // ar & trackReferenceFrameId;
        // ar & lastSeenFrameId;

        // Variables used by local mapping
        // ar & baLocalKeyFrameId;
        // ar & fuseCandidateKeyFrameId;

        // Variables used by loop closing and merging
        // ar & loopPointKeyFrameId;
        // ar & correctedByKeyFrameId;
        // ar & correctedReferenceKeyFrameId;
        // serializeMatrix(ar,mPosGBA,version);
        // ar & baGlobalKeyFrameId;
        // ar & baLocalMergeId;
        // serializeMatrix(ar,mPosMerge,version);
        // serializeMatrix(ar,mNormalVectorMerge,version);

        // Protected variables
        ar &boost::serialization::make_array(worldPos.data(), worldPos.size());
        ar &boost::serialization::make_array(normalVector.data(),
                                             normalVector.size());
        // ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
        // ar & mObservations;
        ar & backupObservationIds1;
        ar & backupObservationIds2;
        serializeMatrix(ar, descriptor, version);
        ar & backupRefKeyFrameId;
        // ar & mnVisible;
        // ar & mnFound;

        ar & isFlaggedBad;
        ar & backupReplacedId;

        ar & minDistance;
        ar & maxDistance;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapPoint();

    MapPoint(const Eigen::Vector3f &Pos_in,
             KeyFrame              *p_referenceKeyFrame_in,
             Map                   *p_map_in);
    MapPoint(const double invDepth_in,
             cv::Point2f  initialPixel_in,
             KeyFrame    *p_referenceKeyFrame_in,
             KeyFrame    *p_hostKeyFrame_in,
             Map         *p_map_in);
    MapPoint(const Eigen::Vector3f &Pos_in,
             Map                   *p_map_in,
             Frame                 *p_frame_inout,
             const int             &indexF_in);

    void            setWorldPos(const Eigen::Vector3f &Pos_in);
    Eigen::Vector3f getWorldPos();

    Eigen::Vector3f getNormal();
    void            setNormalVector(const Eigen::Vector3f &normal_in);

    KeyFrame *getReferenceKeyFrame();

    std::map<KeyFrame *, std::tuple<int, int>> getObservations();
    int                                        getObservationCount();

    void addObservation(KeyFrame *p_keyFrame_inout, int index_in);
    void eraseObservation(KeyFrame *p_keyFrame_in);

    std::tuple<int, int> getIndexInKeyFrame(KeyFrame *p_keyFrame_in);
    bool                 isInKeyFrame(KeyFrame *p_keyFrame_in);

    void setBadFlag();
    bool isBad();

    void      replace(MapPoint *p_mapPoint_inout);
    MapPoint *getReplaced();

    void       increaseVisible(int n_in = 1);
    void       increaseFound(int n_in = 1);
    float      getFoundRatio();
    inline int getFound()
    {
        return foundCount;
    }

    void computeDistinctiveDescriptors();

    cv::Mat getDescriptor();

    void updateNormalAndDepth();

    float getMinDistanceInvariance();
    float getMaxDistanceInvariance();
    int predictScale(const float &currentDistance_in, KeyFrame *p_keyFrame_in);
    int predictScale(const float &currentDistance_in, Frame *p_pF_in);

    Map *getMap();
    void updateMap(Map *p_map_in);

    void printObservations();

    void preSave(set<KeyFrame *> &keyFrames_in, set<MapPoint *> &mapPoints_in);
    void postLoad(map<long unsigned int, KeyFrame *> &keyFrameId_in,
                  map<long unsigned int, MapPoint *> &mapPointId_in);

  public:
    long unsigned int        id;
    static long unsigned int nextId;
    long int                 firstKeyFrameId;
    long int                 firstFrameId;
    int                      observationCount;

    // Variables used by the tracking
    float             trackProjX;
    float             trackProjY;
    float             trackDepth;
    float             trackDepthR;
    float             trackProjXR;
    float             trackProjYR;
    bool              isTrackedInView, isTrackedInRightView;
    int               trackScaleLevel, trackScaleLevelR;
    float             trackViewCos, trackViewCosR;
    long unsigned int trackReferenceFrameId;
    long unsigned int lastSeenFrameId;

    // Variables used by local mapping
    long unsigned int baLocalKeyFrameId;
    long unsigned int fuseCandidateKeyFrameId;

    // Variables used by loop closing
    long unsigned int loopPointKeyFrameId;
    long unsigned int correctedByKeyFrameId;
    long unsigned int correctedReferenceKeyFrameId;
    Eigen::Vector3f   posGBA;
    long unsigned int baGlobalKeyFrameId;
    long unsigned int baLocalMergeId;

    // Variable used by merging
    Eigen::Vector3f posMerge;
    Eigen::Vector3f normalVectorMerge;

    // Fopr inverse depth optimization
    double    inverseDepth;
    double    initU;
    double    initV;
    KeyFrame *p_hostKF;

    static std::mutex globalMutex;

    unsigned int originMapId;

  protected:
    // Position in absolute coordinates
    Eigen::Vector3f worldPos;

    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame *, std::tuple<int, int>> observations;
    // For save relation without pointer, this is necessary for save/load
    // function
    std::map<long unsigned int, int>           backupObservationIds1;
    std::map<long unsigned int, int>           backupObservationIds2;

    // Mean viewing direction
    Eigen::Vector3f normalVector;

    // Best descriptor to fast matching
    cv::Mat descriptor;

    // Reference KeyFrame
    KeyFrame         *p_referenceKeyFrame;
    long unsigned int backupRefKeyFrameId;

    // Tracking counters
    int visibleCount;
    int foundCount;

    // Bad flag (we do not currently erase MapPoint from memory)
    bool          isFlaggedBad;
    MapPoint     *p_replaced;
    // For save relation without pointer, this is necessary for save/load
    // function
    long long int backupReplacedId;

    // Scale invariance distances
    float minDistance;
    float maxDistance;

    Map *p_map;

    // Mutex
    std::mutex positionMutex;
    std::mutex featuresMutex;
    std::mutex mapMutex;
};

} // namespace core
} // namespace vs_graphs

#endif // MAPPOINT_H
