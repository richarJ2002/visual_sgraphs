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
 * @file            serialize.cc
 *
 * @brief           Implements Pinhole::serialize(), declared in
 *                  CameraModels/Pinhole/objects/Pinhole.h.
 */

#include "CameraModels/Pinhole/objects/Pinhole.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace pinhole
{

// Explicitly instantiated below for every archive the map save/load
// code uses, so that no other translation unit compiles this body.
template <class Archive>
void Pinhole::serialize(Archive                            &ar,
                        [[maybe_unused]] const unsigned int version)
{
    ar &boost::serialization::base_object<geometriccamera::GeometricCamera>(
        *this);
}

/*!
 * @brief           Reads the GeometricCamera base data from a binary input
 *                  archive (explicit instantiation of Pinhole::serialize).
 */
template void Pinhole::serialize<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &,
    const unsigned int);
/*!
 * @brief           Writes the GeometricCamera base data to a binary output
 *                  archive (explicit instantiation of Pinhole::serialize).
 */
template void Pinhole::serialize<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &,
    const unsigned int);
/*!
 * @brief           Reads the GeometricCamera base data from a text input
 *                  archive (explicit instantiation of Pinhole::serialize).
 */
template void Pinhole::serialize<boost::archive::text_iarchive>(
    boost::archive::text_iarchive &,
    const unsigned int);
/*!
 * @brief           Writes the GeometricCamera base data to a text output
 *                  archive (explicit instantiation of Pinhole::serialize).
 */
template void Pinhole::serialize<boost::archive::text_oarchive>(
    boost::archive::text_oarchive &,
    const unsigned int);

} // namespace pinhole
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
