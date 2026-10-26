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
 * @file            TwinFaceCheckEnum.h
 *
 * @brief           Declares the outcome of the twin-wall-face plausibility
 *                  test.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_TWIN_FACE_CHECK_ENUM_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_TWIN_FACE_CHECK_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Which check of arePlausibleTwinWallFaces() decided the
 *                  result, in the order the checks run. Values are fixed and
 *                  never reordered.
 */
enum class TwinFaceCheck : std::uint8_t
{
    /*!
     * @brief           Every check passed; the planes are plausible twins.
     */
    PLAUSIBLE = 0U,

    /*!
     * @brief           A plane is null or bad, both are the same plane, or an
     *                  equation is degenerate.
     */
    INVALID_PLANE = 1U,

    /*!
     * @brief           The normals differ by more than the parallelism
     *                  threshold.
     */
    NOT_PARALLEL = 2U,

    /*!
     * @brief           The face separation is outside the plausible wall
     *                  thickness range.
     */
    THICKNESS_OUT_OF_RANGE = 3U,

    /*!
     * @brief           A plane carries no observation origin.
     */
    NO_OBSERVATION_ORIGIN = 4U,

    /*!
     * @brief           An observation origin lies too close to a face plane to
     *                  tell its side.
     */
    OBSERVATION_SIDE_UNRESOLVED = 5U,

    /*!
     * @brief           Both faces were observed from the same side.
     */
    OBSERVED_FROM_SAME_SIDE = 6U,

    /*!
     * @brief           A footprint could not be projected or is degenerate, or
     *                  the footprints do not overlap.
     */
    NO_FOOTPRINT_OVERLAP = 7U,

    /*!
     * @brief           The footprints overlap less than the minimum ratio.
     */
    LOW_OVERLAP = 8U
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_TWIN_FACE_CHECK_ENUM_H  \
        */
