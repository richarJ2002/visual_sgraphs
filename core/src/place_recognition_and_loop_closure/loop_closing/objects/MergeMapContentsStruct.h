/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            MergeMapContentsStruct.h
 *
 * @brief           Declares MergeMapContents, the object lists of the map that
 *                  a merge absorbs.
 */

#ifndef VS_GRAPHS_CORE_LOOPCLOSING_MERGE_MAP_CONTENTS_STRUCT_H
#define VS_GRAPHS_CORE_LOOPCLOSING_MERGE_MAP_CONTENTS_STRUCT_H

#include <vector>

namespace vs_graphs
{
namespace core
{

namespace geometric
{
class Plane;
} /* namespace geometric */

namespace keyframe
{
class KeyFrame;
} /* namespace keyframe */

namespace map_point
{
class MapPoint;
} /* namespace map_point */

namespace loop_closing
{

/*!
 * @brief           Copies of the plane, keyframe and map point lists of
 *                  the map that a merge absorbs (its scene graph is handled
 *                  by the SemanticsManager). The objects are borrowed from
 *                  that map; the lists stay valid while the objects are moved
 *                  into the surviving map.
 */
struct MergeMapContents
{
    /*!
     * @brief           Planes of the absorbed map.
     */
    std::vector<geometric::Plane *> planes;

    /*!
     * @brief           Keyframes of the absorbed map.
     */
    std::vector<keyframe::KeyFrame *> keyFrames;

    /*!
     * @brief           Map points of the absorbed map.
     */
    std::vector<map_point::MapPoint *> mapPoints;
};

} /* namespace loop_closing */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_LOOPCLOSING_MERGE_MAP_CONTENTS_STRUCT_H */
