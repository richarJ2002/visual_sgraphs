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

#include "Atlas.h"
#include "Frame.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>

namespace vs_graphs
{
namespace core
{

// Explicitly instantiated below for every archive the map save/load
// code uses, so that no other translation unit compiles this body.
template <class Archive>
void Atlas::serialize(Archive &ar, [[maybe_unused]] const unsigned int version)
{
    ar.template register_type<camera_models::pinhole::Pinhole>();
    ar.template register_type<camera_models::kannalabrandt8::KannalaBrandt8>();

    // Save/load a set structure, the set structure is broken in
    // libboost 1.58 for ubuntu 16.04, a vector is serializated ar &
    // mspMaps;
    ar & backupMaps;
    ar & cameras;
    // Need to save/load the static Id from Frame, KeyFrame, MapPoint and
    // Map
    ar &Map::nextId;
    ar &Frame::nextId;
    ar &KeyFrame::nextId;
    ar &MapPoint::nextId;
    ar &camera_models::geometriccamera::GeometricCamera::nextId;
    ar & lastInitKeyFrameId;
}

template void Atlas::serialize<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &,
    const unsigned int);
template void Atlas::serialize<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &,
    const unsigned int);
template void Atlas::serialize<boost::archive::text_iarchive>(
    boost::archive::text_iarchive &,
    const unsigned int);
template void Atlas::serialize<boost::archive::text_oarchive>(
    boost::archive::text_oarchive &,
    const unsigned int);

} // namespace core
} // namespace vs_graphs
