/**
 * This file is a modified version of a file from ORB-SLAM3.
 * 
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 * 
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 * 
 * This file is part of vS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Converter.h"

#include "SerializationUtils.h"

#include <opencv2/core/core.hpp>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

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
            ar & mnId;
            ar & mnFirstKFid;
            ar & firstFrameId;
            ar & nObs;
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
            ar &boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
            ar &boost::serialization::make_array(mNormalVector.data(), mNormalVector.size());
            // ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
            // ar & mObservations;
            ar & mBackupObservationsId1;
            ar & mBackupObservationsId2;
            serializeMatrix(ar, mDescriptor, version);
            ar & mBackupRefKFId;
            // ar & mnVisible;
            // ar & mnFound;

            ar & mbBad;
            ar & mBackupReplacedId;

            ar & mfMinDistance;
            ar & mfMaxDistance;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        MapPoint();

        MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map *pMap);
        MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame *pRefKF, KeyFrame *pHostKF, Map *pMap);
        MapPoint(const Eigen::Vector3f &Pos, Map *pMap, Frame *pFrame, const int &idxF);

        void setWorldPos(const Eigen::Vector3f &Pos);
        Eigen::Vector3f getWorldPos();

        Eigen::Vector3f getNormal();
        void setNormalVector(const Eigen::Vector3f &normal);

        KeyFrame *getReferenceKeyFrame();

        std::map<KeyFrame *, std::tuple<int, int>> getObservations();
        int getObservationCount();

        void addObservation(KeyFrame *pKF, int idx);
        void eraseObservation(KeyFrame *pKF);

        std::tuple<int, int> getIndexInKeyFrame(KeyFrame *pKF);
        bool isInKeyFrame(KeyFrame *pKF);

        void setBadFlag();
        bool isBad();

        void replace(MapPoint *pMP);
        MapPoint *getReplaced();

        void increaseVisible(int n = 1);
        void increaseFound(int n = 1);
        float getFoundRatio();
        inline int getFound()
        {
            return mnFound;
        }

        void computeDistinctiveDescriptors();

        cv::Mat getDescriptor();

        void updateNormalAndDepth();

        float getMinDistanceInvariance();
        float getMaxDistanceInvariance();
        int predictScale(const float &currentDist, KeyFrame *pKF);
        int predictScale(const float &currentDist, Frame *pF);

        Map *getMap();
        void updateMap(Map *pMap);

        void printObservations();

        void PreSave(set<KeyFrame *> &spKF, set<MapPoint *> &spMP);
        void PostLoad(map<long unsigned int, KeyFrame *> &mpKFid, map<long unsigned int, MapPoint *> &mpMPid);

    public:
        long unsigned int mnId;
        static long unsigned int nNextId;
        long int mnFirstKFid;
        long int firstFrameId;
        int nObs;

        // Variables used by the tracking
        float mTrackProjX;
        float mTrackProjY;
        float mTrackDepth;
        float mTrackDepthR;
        float mTrackProjXR;
        float mTrackProjYR;
        bool mbTrackInView, mbTrackInViewR;
        int mnTrackScaleLevel, mnTrackScaleLevelR;
        float mTrackViewCos, mTrackViewCosR;
        long unsigned int trackReferenceFrameId;
        long unsigned int lastSeenFrameId;

        // Variables used by local mapping
        long unsigned int baLocalKeyFrameId;
        long unsigned int fuseCandidateKeyFrameId;

        // Variables used by loop closing
        long unsigned int loopPointKeyFrameId;
        long unsigned int correctedByKeyFrameId;
        long unsigned int correctedReferenceKeyFrameId;
        Eigen::Vector3f mPosGBA;
        long unsigned int baGlobalKeyFrameId;
        long unsigned int baLocalMergeId;

        // Variable used by merging
        Eigen::Vector3f mPosMerge;
        Eigen::Vector3f mNormalVectorMerge;

        // Fopr inverse depth optimization
        double mInvDepth;
        double mInitU;
        double mInitV;
        KeyFrame *mpHostKF;

        static std::mutex mGlobalMutex;

        unsigned int originMapId;

    protected:
        // Position in absolute coordinates
        Eigen::Vector3f mWorldPos;

        // Keyframes observing the point and associated index in keyframe
        std::map<KeyFrame *, std::tuple<int, int>> mObservations;
        // For save relation without pointer, this is necessary for save/load function
        std::map<long unsigned int, int> mBackupObservationsId1;
        std::map<long unsigned int, int> mBackupObservationsId2;

        // Mean viewing direction
        Eigen::Vector3f mNormalVector;

        // Best descriptor to fast matching
        cv::Mat mDescriptor;

        // Reference KeyFrame
        KeyFrame *mpRefKF;
        long unsigned int mBackupRefKFId;

        // Tracking counters
        int mnVisible;
        int mnFound;

        // Bad flag (we do not currently erase MapPoint from memory)
        bool mbBad;
        MapPoint *mpReplaced;
        // For save relation without pointer, this is necessary for save/load function
        long long int mBackupReplacedId;

        // Scale invariance distances
        float mfMinDistance;
        float mfMaxDistance;

        Map *mpMap;

        // Mutex
        std::mutex mMutexPos;
        std::mutex mMutexFeatures;
        std::mutex mMutexMap;
    };

} // namespace core
} // namespace vs_graphs

#endif // MAPPOINT_H
