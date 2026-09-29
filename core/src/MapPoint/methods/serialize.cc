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

#include "MapPoint.h"

#include "SerializationUtils.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>

namespace vs_graphs
{
namespace core
{

// Explicitly instantiated below for every archive the map save/load
// code uses, so that no other translation unit compiles this body.
template <class Archive>
void MapPoint::serialize(Archive &ar, const unsigned int version)
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

template void MapPoint::serialize<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &,
    const unsigned int);
template void MapPoint::serialize<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &,
    const unsigned int);
template void MapPoint::serialize<boost::archive::text_iarchive>(
    boost::archive::text_iarchive &,
    const unsigned int);
template void MapPoint::serialize<boost::archive::text_oarchive>(
    boost::archive::text_oarchive &,
    const unsigned int);

} // namespace core
} // namespace vs_graphs
