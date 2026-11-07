/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
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
 * @file            WallAdmissionEvidenceStruct.h
 *
 * @brief           Declares the facts that decide whether a wall may join a
 *                  room.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_WALL_ADMISSION_EVIDENCE_STRUCT_H
#define VS_GRAPHS_SEMANTICS_MANAGER_WALL_ADMISSION_EVIDENCE_STRUCT_H

#include <cstddef>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Facts that decide whether a wall plane may join a room;
 *                  filled by evaluateWallAdmissionEvidence().
 */
struct WallAdmissionEvidence
{
    /*!
     * @brief           True when the wall may join a room: it is a wall by type
     *                  and by expected type, its finite fit is adequate, and it
     *                  was seen often enough or strongly enough at first sight.
     *
     * @frame           N/A
     *
     * @units           N/A
     */
    bool isAdmissible = false;

    /*!
     * @brief           True when at least 20 points fit the plane, enough of
     *                  the points fit, and the fitted extent is large enough
     *                  (length, height and area).
     *
     * @frame           N/A
     *
     * @units           N/A
     */
    bool hasAdequateFiniteFit = false;

    /*!
     * @brief           Points of the support cloud with finite coordinates.
     *
     * @frame           N/A
     *
     * @units           number of points
     */
    std::size_t finitePointCount = 0U;

    /*!
     * @brief           Finite points within the RANSAC distance of the plane.
     *
     * @frame           N/A
     *
     * @units           number of points
     */
    std::size_t fittedPointCount = 0U;
    /*!
     * @brief           Number of times the wall has been observed.
     *
     * @frame           N/A
     *
     * @units           number of observations
     */
    std::size_t observationCount = 0U;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_WALL_ADMISSION_EVIDENCE_STRUCT_H */
