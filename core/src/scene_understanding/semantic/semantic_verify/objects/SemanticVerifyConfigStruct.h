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
 * @file            SemanticVerifyConfigStruct.h
 *
 * @brief           Declares SemanticVerifyConfig, the thresholds deciding
 * same-place.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_VERIFY_CONFIG_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_VERIFY_CONFIG_STRUCT_H

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Thresholds and weights that decide when two rooms' wall sets
 *                  are the same place. Defaults are fallbacks;
 *                  configFromSystemParams() reads the operator's values.
 */
struct SemanticVerifyConfig
{
    /*!
     * @brief           Largest angle between a transformed wall normal and its
     *                  partner's normal for the pair to count as an inlier,
     *                  degrees.
     */
    double       maxNormalAngle_deg{10.0};
    /*!
     * @brief           Largest difference between a transformed wall plane
     *                  offset and its partner's offset for an inlier, metres.
     */
    double       maxOffset_m{0.35};
    /*!
     * @brief           Largest symmetric distance between the support samples
     *                  of two paired walls for an inlier, metres.
     */
    double       maxSupportDist_m{0.25};
    /*!
     * @brief           Smallest fraction of walls that must be inliers for a
     *                  hypothesis to pass, dimensionless.
     */
    double       minInlierRatio{0.6};
    /*!
     * @brief           Largest condition number of the translation system a
     *                  hypothesis may have; also scales the normalised
     *                  condition number.
     */
    double       maxConditionNumber{100.0};
    /*!
     * @brief           Number of inliers the best hypothesis must exceed the
     *                  runner-up by, otherwise the result is rejected as
     *                  ambiguous.
     */
    unsigned int ambiguityMarginInliers{1U};
    /*!
     * @brief           Largest number of walls collected from one room; further
     *                  walls are ignored.
     */
    std::size_t  maxWallsPerRoom{16U};
    /*!
     * @brief           Largest number of hypotheses verify() evaluates before
     *                  it stops enumerating.
     */
    std::size_t  maxHypotheses{2000U};
    /*!
     * @brief           Largest number of support points kept per wall.
     */
    std::size_t  maxSupportSamplePerWall{64U};
    /*!
     * @brief           Explicit |cos(theta)| gate, distinct from
     *                  maxNormalAngle_deg.
     */
    double       minAbsCosNormalAngle{0.85};
    /*!
     * @brief           Standard deviation of the normal angle residual that
     *                  weights the refinement, radians.
     */
    double       sigmaTheta_rad{0.05};
    /*!
     * @brief           Standard deviation of the plane offset residual that
     *                  weights the refinement, metres.
     */
    double       sigmaOffset_m{0.05};
    /*!
     * @brief           Huber kernel threshold of the refinement, in weighted
     *                  residual units.
     */
    double       huberDelta{1.345};
    /*!
     * @brief           Number of refinement iterations the optimizer runs.
     */
    unsigned int optimizerIterations{20U};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_VERIFY_CONFIG_STRUCT_H \
        */
