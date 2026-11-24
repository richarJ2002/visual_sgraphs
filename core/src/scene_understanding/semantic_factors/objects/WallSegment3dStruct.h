/*!
 * @file            WallSegment3dStruct.h
 *
 * @brief           Declares WallSegment3d, the finite horizontal extent of
 *                  one wall that has a plane vertex in a bundle adjustment.
 */

#ifndef VS_GRAPHS_SEMANTIC_FACTORS_OBJECTS_WALL_SEGMENT_3D_STRUCT_H
#define VS_GRAPHS_SEMANTIC_FACTORS_OBJECTS_WALL_SEGMENT_3D_STRUCT_H

#include <Eigen/Core>
#include <vector>

namespace vs_graphs::core::geometric
{
class Plane;
} /* namespace vs_graphs::core::geometric */
namespace g2o
{
class VertexPlane;
} /* namespace g2o */

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           One wall of a bundle adjustment as a horizontal segment at
 *                  its centroid height, built before the optimisation from
 *                  the wall's support points; frozen while it runs.
 */
struct WallSegment3d
{
  public:
    /*!
     * @brief           The wall; borrowed, owned by the map.
     */
    geometric::Plane *p_wall{nullptr};

    /*!
     * @brief           Id of the wall plane.
     */
    int wallId{-1};

    /*!
     * @brief           The wall's plane vertex; borrowed, owned by the
     *                  optimiser.
     */
    g2o::VertexPlane *p_wallVertex{nullptr};

    /*!
     * @brief           First end of the wall.
     *
     * @frame           World
     *
     * @units           metres
     */
    Eigen::Vector3d wallStart_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Second end of the wall.
     *
     * @frame           World
     *
     * @units           metres
     */
    Eigen::Vector3d wallEnd_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Wall normal without its up component, unit length;
     *                  either sign.
     *
     * @frame           World
     */
    Eigen::Vector3d horizontalNormal_world{Eigen::Vector3d::UnitX()};

    /*!
     * @brief           Ids of the rooms that list this wall, ascending; empty
     *                  for a wall of no room.
     */
    std::vector<int> roomIds;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTIC_FACTORS_OBJECTS_WALL_SEGMENT_3D_STRUCT_H */
