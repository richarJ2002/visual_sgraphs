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
 * @file            MapMergeRecordStruct.h
 *
 * @brief           Declares MapMergeRecord, the record of one committed map
 *                  merge.
 */

#ifndef VS_GRAPHS_CORE_SRC_MAP_OBJECTS_MAP_MERGE_RECORD_STRUCT_H
#define VS_GRAPHS_CORE_SRC_MAP_OBJECTS_MAP_MERGE_RECORD_STRUCT_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vs_graphs
{
namespace core
{
namespace map
{
/*!
 * @brief           What a committed merge did: which maps it joined and the
 *                  similarity that moved the absorbed map into the surviving
 *                  map's world frame.
 */
struct MapMergeRecord
{
    /*!
     * @brief           Id of the map that was absorbed (marked bad).
     */
    unsigned long      absorbedMapId{0UL};
    /*!
     * @brief           Id of the map that survived and holds this record.
     */
    unsigned long      survivingMapId{0UL};
    /*!
     * @brief           Time of the merge on the ROS node clock (simulated time
     *                  when use_sim_time is set), seconds; the capture time of
     *                  the key frame that triggered the merge.
     */
    double             mergeTime_s{0.0};
    /*!
     * @brief           Rotation part of the merge similarity.
     *
     * @frame           Absorbed map's world to the surviving map's world.
     */
    Eigen::Quaterniond mergeRotation_absorbedToSurviving{
        Eigen::Quaterniond::Identity()};
    /*!
     * @brief           Translation part of the merge similarity, metres,
     *                  applied after rotation and scale:
     *                  p_surviving = scale * R * p_absorbed + t.
     *
     * @frame           Absorbed map's world to the surviving map's world.
     */
    Eigen::Vector3d mergeTranslation_absorbedToSurviving_m{
        Eigen::Vector3d::Zero()};
    /*!
     * @brief           Scale part of the merge similarity, unitless.
     */
    double mergeScale_absorbedToSurviving{1.0};
};

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_MAP_OBJECTS_MAP_MERGE_RECORD_STRUCT_H */
