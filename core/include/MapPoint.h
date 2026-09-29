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

#include "MapPointStatus.h"
#include "Utils/Converter/objects/Converter.h"

#include <boost/serialization/access.hpp>
#include <map>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <set>
#include <tuple>

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
    void serialize(Archive &ar, const unsigned int version);

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

    [[nodiscard]] MapPointStatus setWorldPos(const Eigen::Vector3f &Pos_in);
    [[nodiscard]] MapPointStatus getWorldPos(Eigen::Vector3f &worldPos_out);

    [[nodiscard]] MapPointStatus getNormal(Eigen::Vector3f &normal_out);
    [[nodiscard]] MapPointStatus
        setNormalVector(const Eigen::Vector3f &normal_in);

    [[nodiscard]] MapPointStatus
        getReferenceKeyFrame(KeyFrame *&p_referenceKeyFrame_out);

    [[nodiscard]] MapPointStatus getObservations(
        std::map<KeyFrame *, std::tuple<int, int>> &observations_out);
    [[nodiscard]] MapPointStatus getObservationCount(int &observationCount_out);

    [[nodiscard]] MapPointStatus addObservation(KeyFrame *p_keyFrame_inout,
                                                int       index_in);
    [[nodiscard]] MapPointStatus eraseObservation(KeyFrame *p_keyFrame_in);

    [[nodiscard]] MapPointStatus
                                 getIndexInKeyFrame(KeyFrame             *p_keyFrame_in,
                                                    std::tuple<int, int> &indexInKeyFrame_out);
    [[nodiscard]] MapPointStatus isInKeyFrame(KeyFrame *p_keyFrame_in,
                                              bool     &isInKeyFrame_out);

    [[nodiscard]] MapPointStatus setBadFlag();
    [[nodiscard]] MapPointStatus isBad(bool &isBad_out);

    [[nodiscard]] MapPointStatus replace(MapPoint *p_mapPoint_inout);
    [[nodiscard]] MapPointStatus getReplaced(MapPoint *&p_replaced_out);

    [[nodiscard]] MapPointStatus increaseVisible(int n_in = 1);
    [[nodiscard]] MapPointStatus increaseFound(int n_in = 1);
    [[nodiscard]] MapPointStatus getFoundRatio(float &foundRatio_out);

    [[nodiscard]] MapPointStatus computeDistinctiveDescriptors();

    [[nodiscard]] MapPointStatus getDescriptor(cv::Mat &descriptor_out);

    [[nodiscard]] MapPointStatus updateNormalAndDepth();

    [[nodiscard]] MapPointStatus
        getMinDistanceInvariance(float &minDistanceInvariance_out);
    [[nodiscard]] MapPointStatus
        getMaxDistanceInvariance(float &maxDistanceInvariance_out);
    [[nodiscard]] MapPointStatus predictScale(const float &currentDistance_in,
                                              KeyFrame    *p_keyFrame_in,
                                              int         &scaleLevel_out);
    [[nodiscard]] MapPointStatus predictScale(const float &currentDistance_in,
                                              Frame       *p_pF_in,
                                              int         &scaleLevel_out);

    [[nodiscard]] MapPointStatus getMap(Map *&p_map_out);
    [[nodiscard]] MapPointStatus updateMap(Map *p_map_in);

    [[nodiscard]] MapPointStatus printObservations();

    [[nodiscard]] MapPointStatus preSave(std::set<KeyFrame *> &keyFrames_in,
                                         std::set<MapPoint *> &mapPoints_in);
    [[nodiscard]] MapPointStatus
        postLoad(std::map<long unsigned int, KeyFrame *> &keyFrameId_in,
                 std::map<long unsigned int, MapPoint *> &mapPointId_in);

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
