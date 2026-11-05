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
 * @file            PlaneMatchVerdictStruct.h
 *
 * @brief           Declares the per-candidate outcome of an OPPOSITE_FACE
 *                  Utils::findMatchingPlane() call.
 */

#ifndef VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEMATCHVERDICTSTRUCT_H
#define VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEMATCHVERDICTSTRUCT_H

#include "mapping/geometry_helpers/objects/TwinFaceCheckEnum.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

/*!
 * @brief           Why one candidate plane did or did not match.
 */
struct PlaneMatchVerdict
{
  public:
    /*!
     * @brief           PLAUSIBLE, or the first check that failed.
     */
    TwinFaceCheck decidingCheck{TwinFaceCheck::INVALID_PLANE};

    /*!
     * @brief           Measurement behind decidingCheck: the face separation
     *                  in metres for THICKNESS_OUT_OF_RANGE, the smallest
     *                  observation-origin distance from a face plane in metres
     *                  for OBSERVATION_SIDE_UNRESOLVED, the footprint overlap
     *                  ratio for PLAUSIBLE and LOW_OVERLAP; 0 otherwise.
     */
    double decidingValue{0.0};

    /*!
     * @brief           Face separation in metres (the mean distance of each
     *                  centroid from the other plane), set once the thickness
     *                  check is reached; 0 otherwise.
     */
    double faceSeparation_m{0.0};
};

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEMATCHVERDICTSTRUCT_H */
