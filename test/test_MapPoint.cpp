/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
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
 * @file            test_MapPoint.cpp
 *
 * @brief           MapPoint regression test: postLoad restores an observation
 *                  whose saved right key point index is missing as not seen by
 *                  the right camera.
 */

#include "MapPoint.h"

#include "KeyFrame.h"

#include <gtest/gtest.h>

#include <map>
#include <tuple>

namespace vs_graphs
{
namespace core
{
namespace
{

/*!
 * @brief           Map point whose saved ids a test can set, to give postLoad
 *                  an archive state that preSave never writes.
 */
class MapPointWithSavedIds : public MapPoint
{
  public:
    /*!
     * @brief           Replaces the ids that preSave would have stored.
     *
     * @param[in]       referenceKeyFrameId_in
     *                  Id of the reference key frame.
     *
     * @param[in]       leftIndices_in
     *                  Observing key frame id to left key point index.
     *
     * @param[in]       rightIndices_in
     *                  Observing key frame id to right key point index.
     */
    void setSavedIds(long unsigned int referenceKeyFrameId_in,
                     const std::map<long unsigned int, int> &leftIndices_in,
                     const std::map<long unsigned int, int> &rightIndices_in)
    {
        backupRefKeyFrameId   = referenceKeyFrameId_in;
        backupReplacedId      = -1;
        backupObservationIds1 = leftIndices_in;
        backupObservationIds2 = rightIndices_in;
    }
};

/*!
 * @brief           Checks that an observation saved without a right key point
 *                  index is restored with -1 for the right camera.
 */
TEST(MapPoint, PostLoadReadsAMissingRightIndexAsUnseen)
{
    KeyFrame observingKeyFrame;
    observingKeyFrame.id = 7U;

    MapPointWithSavedIds loadedPoint;
    loadedPoint.setSavedIds(7U, {{7U, 3}}, {});

    std::map<long unsigned int, KeyFrame *> keyFramesById{
        {7U, &observingKeyFrame}};
    std::map<long unsigned int, MapPoint *> mapPointsById{};
    ASSERT_EQ(loadedPoint.postLoad(keyFramesById, mapPointsById),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);

    std::map<KeyFrame *, std::tuple<int, int>> observations;
    ASSERT_EQ(loadedPoint.getObservations(observations),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    ASSERT_EQ(observations.count(&observingKeyFrame), 1U);
    EXPECT_EQ(std::get<0>(observations[&observingKeyFrame]), 3);
    EXPECT_EQ(std::get<1>(observations[&observingKeyFrame]), -1);
}

} // namespace
} // namespace core
} // namespace vs_graphs
