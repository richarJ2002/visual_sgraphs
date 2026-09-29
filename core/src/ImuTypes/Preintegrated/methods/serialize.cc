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

#include "ImuTypes.h"

#include "SerializationUtils.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace vs_graphs
{
namespace core
{
namespace IMU
{

// Explicitly instantiated below for every archive the map save/load
// code uses, so that no other translation unit compiles this body.
template <class Archive>
void Preintegrated::serialize(Archive                            &ar,
                              [[maybe_unused]] const unsigned int version)
{
    ar & dT;
    ar &boost::serialization::make_array(C.data(), C.size());
    ar &boost::serialization::make_array(Info.data(), Info.size());
    ar &boost::serialization::make_array(Nga.diagonal().data(),
                                         Nga.diagonal().size());
    ar &boost::serialization::make_array(NgaWalk.diagonal().data(),
                                         NgaWalk.diagonal().size());
    ar & b;
    ar &boost::serialization::make_array(dR.data(), dR.size());
    ar &boost::serialization::make_array(dV.data(), dV.size());
    ar &boost::serialization::make_array(dP.data(), dP.size());
    ar &boost::serialization::make_array(JRg.data(), JRg.size());
    ar &boost::serialization::make_array(JVg.data(), JVg.size());
    ar &boost::serialization::make_array(JVa.data(), JVa.size());
    ar &boost::serialization::make_array(JPg.data(), JPg.size());
    ar &boost::serialization::make_array(JPa.data(), JPa.size());
    ar &boost::serialization::make_array(avgA.data(), avgA.size());
    ar &boost::serialization::make_array(avgW.data(), avgW.size());

    ar & bu;
    ar &boost::serialization::make_array(db.data(), db.size());
    ar & measurements;
}

template void Preintegrated::serialize<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &,
    const unsigned int);
template void Preintegrated::serialize<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &,
    const unsigned int);
template void Preintegrated::serialize<boost::archive::text_iarchive>(
    boost::archive::text_iarchive &,
    const unsigned int);
template void Preintegrated::serialize<boost::archive::text_oarchive>(
    boost::archive::text_oarchive &,
    const unsigned int);

} // namespace IMU
} // namespace core
} // namespace vs_graphs
