/*!
 * @file GlobalWallMetrics.cc
 * @brief Implementation of the global wall precision/recall/F1 adapter
 *        declared in GlobalWallMetrics.h (semantic-axiom-reliability-plan.md,
 *        P0.5).
 */

#include "GlobalWallMetrics.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace test
{
namespace
{

/* Mirrors compare_sgraph_to_ground_truth.py's module-level constants. */
constexpr double kMaxNormalAngleDeg = 10.0;
constexpr double kMaxOffsetM        = 0.35;
constexpr double kMaxRoomMatchDistM = 3.0;

struct RoomRecord
{
    int             id         = 0;
    Eigen::Vector2d centroidXy = Eigen::Vector2d::Zero();
};

struct WallRecord
{
    int             roomId  = 0;
    Eigen::Vector3d normal  = Eigen::Vector3d::Zero();
    double          offsetD = 0.0;
};

std::vector<RoomRecord> parseRooms(const nlohmann::json &sgraph_in)
{
    std::vector<RoomRecord> rooms;
    for (const nlohmann::json &room : sgraph_in.at("rooms"))
    {
        RoomRecord record;
        record.id                      = room.at("id").get<int>();
        const nlohmann::json &centroid = room.at("centroid_xy");
        record.centroidXy = Eigen::Vector2d(centroid.at(0).get<double>(),
                                            centroid.at(1).get<double>());
        rooms.push_back(record);
    }
    return rooms;
}

std::vector<WallRecord> parseWalls(const nlohmann::json &sgraph_in)
{
    std::vector<WallRecord> walls;
    for (const nlohmann::json &wall : sgraph_in.at("walls"))
    {
        WallRecord record;
        record.roomId                = wall.at("room_id").get<int>();
        const nlohmann::json &normal = wall.at("normal");
        record.normal  = Eigen::Vector3d(normal.at(0).get<double>(),
                                        normal.at(1).get<double>(),
                                        normal.at(2).get<double>());
        record.offsetD = wall.at("offset_d").get<double>();
        walls.push_back(record);
    }
    return walls;
}

/*!
 * @brief   Solves the rectangular minimum-total-cost one-to-one assignment
 *          problem for `rowCount <= colCount`: assigns every row to a
 *          distinct column so the sum of assigned costs is minimal.
 *
 * This is the classic O(rowCount^2 * colCount) shortest-augmenting-path
 * Hungarian algorithm with row/column potentials (see e.g.
 * https://cp-algorithms.com/graph/hungarian-algorithm.html), the same
 * minimum-total-cost one-to-one assignment problem
 * `scipy.optimize.linear_sum_assignment` solves for a rectangular cost
 * matrix. It is deterministic: identical input always produces the same
 * assignment, including when multiple assignments tie at the same total
 * cost.
 *
 * @param   cost_in Cost matrix with `cost_in.size()` rows and
 *                   `cost_in.front().size()` columns; every row must have
 *                   the same column count, and `cost_in.size()` must not
 *                   exceed that column count. Finite costs only.
 *
 * @return  One entry per row: the assigned column index. Empty when
 *          `cost_in` is empty.
 */
std::vector<std::size_t>
    solveAssignmentRowsLeqCols(const std::vector<std::vector<double>> &cost_in)
{
    const std::size_t rowCount = cost_in.size();
    if (rowCount == 0)
    {
        return {};
    }
    const std::size_t colCount = cost_in.front().size();

    constexpr double         kInf = std::numeric_limits<double>::infinity();
    std::vector<double>      rowPotential(rowCount + 1, 0.0);
    std::vector<double>      colPotential(colCount + 1, 0.0);
    std::vector<std::size_t> rowAssignedToCol(colCount + 1, 0);
    std::vector<std::size_t> parentCol(colCount + 1, 0);

    for (std::size_t sourceRow = 1; sourceRow <= rowCount; ++sourceRow)
    {
        rowAssignedToCol[0]            = sourceRow;
        std::size_t         currentCol = 0;
        std::vector<double> minReducedCost(colCount + 1, kInf);
        std::vector<bool>   colVisited(colCount + 1, false);
        do
        {
            colVisited[currentCol]       = true;
            const std::size_t currentRow = rowAssignedToCol[currentCol];
            std::size_t       nextCol    = 0;
            double            delta      = kInf;
            for (std::size_t col = 1; col <= colCount; ++col)
            {
                if (colVisited[col])
                {
                    continue;
                }
                const double reducedCost = cost_in[currentRow - 1][col - 1] -
                                           rowPotential[currentRow] -
                                           colPotential[col];
                if (reducedCost < minReducedCost[col])
                {
                    minReducedCost[col] = reducedCost;
                    parentCol[col]      = currentCol;
                }
                if (minReducedCost[col] < delta)
                {
                    delta   = minReducedCost[col];
                    nextCol = col;
                }
            }
            for (std::size_t col = 0; col <= colCount; ++col)
            {
                if (colVisited[col])
                {
                    rowPotential[rowAssignedToCol[col]] += delta;
                    colPotential[col] -= delta;
                }
                else
                {
                    minReducedCost[col] -= delta;
                }
            }
            currentCol = nextCol;
        }
        while (rowAssignedToCol[currentCol] != 0);

        while (currentCol != 0)
        {
            const std::size_t previousCol = parentCol[currentCol];
            rowAssignedToCol[currentCol]  = rowAssignedToCol[previousCol];
            currentCol                    = previousCol;
        }
    }

    std::vector<std::size_t> rowToCol(rowCount, colCount);
    for (std::size_t col = 1; col <= colCount; ++col)
    {
        if (rowAssignedToCol[col] != 0)
        {
            rowToCol[rowAssignedToCol[col] - 1] = col - 1;
        }
    }
    return rowToCol;
}

/*! Matches truth rooms to generated rooms by minimum-total-cost one-to-one
 * centroid-distance assignment (see `solveAssignmentRowsLeqCols`), gating
 * each assigned pair by `kMaxRoomMatchDistM` only after the full assignment
 * is solved -- exactly mirroring `match_rooms()` in
 * `compare_sgraph_to_ground_truth.py`, which applies `MAX_ROOM_MATCH_DIST_M`
 * after `linear_sum_assignment()`. `solveAssignmentRowsLeqCols` requires
 * rows <= columns, so the smaller side is solved as rows and the result
 * transposed back when there are more truth rooms than generated rooms. */
std::vector<std::pair<std::size_t, std::size_t>>
    matchRoomsOptimal(const std::vector<RoomRecord> &truthRooms_in,
                      const std::vector<RoomRecord> &genRooms_in)
{
    if (truthRooms_in.empty() || genRooms_in.empty())
    {
        return {};
    }

    std::vector<std::vector<double>> costTruthByGen(
        truthRooms_in.size(),
        std::vector<double>(genRooms_in.size()));
    for (std::size_t truthIndex = 0; truthIndex < truthRooms_in.size();
         ++truthIndex)
    {
        for (std::size_t genIndex = 0; genIndex < genRooms_in.size();
             ++genIndex)
        {
            costTruthByGen[truthIndex][genIndex] =
                (truthRooms_in[truthIndex].centroidXy -
                 genRooms_in[genIndex].centroidXy)
                    .norm();
        }
    }

    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    if (truthRooms_in.size() <= genRooms_in.size())
    {
        const std::vector<std::size_t> truthToGen =
            solveAssignmentRowsLeqCols(costTruthByGen);
        for (std::size_t truthIndex = 0; truthIndex < truthToGen.size();
             ++truthIndex)
        {
            const std::size_t genIndex = truthToGen[truthIndex];
            if (costTruthByGen[truthIndex][genIndex] <= kMaxRoomMatchDistM)
            {
                pairs.emplace_back(truthIndex, genIndex);
            }
        }
    }
    else
    {
        std::vector<std::vector<double>> costGenByTruth(
            genRooms_in.size(),
            std::vector<double>(truthRooms_in.size()));
        for (std::size_t truthIndex = 0; truthIndex < truthRooms_in.size();
             ++truthIndex)
        {
            for (std::size_t genIndex = 0; genIndex < genRooms_in.size();
                 ++genIndex)
            {
                costGenByTruth[genIndex][truthIndex] =
                    costTruthByGen[truthIndex][genIndex];
            }
        }
        const std::vector<std::size_t> genToTruth =
            solveAssignmentRowsLeqCols(costGenByTruth);
        for (std::size_t genIndex = 0; genIndex < genToTruth.size(); ++genIndex)
        {
            const std::size_t truthIndex = genToTruth[genIndex];
            if (costTruthByGen[truthIndex][genIndex] <= kMaxRoomMatchDistM)
            {
                pairs.emplace_back(truthIndex, genIndex);
            }
        }
        std::sort(pairs.begin(), pairs.end());
    }
    return pairs;
}

/*! Mirrors wall_matches() from compare_sgraph_to_ground_truth.py: for each
 * truth wall, greedily takes the best-scoring still-unused generated wall
 * that clears both gates. Returns the matched truth-wall indices (within
 * truthWalls_in) and generated-wall indices (within genWalls_in). */
std::pair<std::set<std::size_t>, std::set<std::size_t>>
    matchWallsWithinRoomPair(const std::vector<WallRecord>  &truthWalls_in,
                             const std::vector<std::size_t> &truthIndices_in,
                             const std::vector<WallRecord>  &genWalls_in,
                             const std::vector<std::size_t> &genIndices_in)
{
    std::set<std::size_t> matchedTruth;
    std::set<std::size_t> matchedGen;
    std::set<std::size_t> usedGenLocal;

    for (const std::size_t truthIndex : truthIndices_in)
    {
        const WallRecord &truthWall = truthWalls_in[truthIndex];
        double            bestScore = std::numeric_limits<double>::infinity();
        std::size_t       bestGenLocal = 0;
        bool              haveBest     = false;

        for (std::size_t genLocal = 0; genLocal < genIndices_in.size();
             ++genLocal)
        {
            if (usedGenLocal.count(genLocal) > 0)
            {
                continue;
            }
            const WallRecord &genWall = genWalls_in[genIndices_in[genLocal]];
            const double      cosAngle =
                std::clamp(truthWall.normal.dot(genWall.normal), -1.0, 1.0);
            const double angleDeg = std::acos(cosAngle) * 180.0 / M_PI;
            const double offsetErrorM =
                std::abs(truthWall.offsetD - genWall.offsetD);
            if (angleDeg <= kMaxNormalAngleDeg && offsetErrorM <= kMaxOffsetM)
            {
                const double score =
                    angleDeg / kMaxNormalAngleDeg + offsetErrorM / kMaxOffsetM;
                if (score < bestScore)
                {
                    bestScore    = score;
                    bestGenLocal = genLocal;
                    haveBest     = true;
                }
            }
        }
        if (haveBest)
        {
            usedGenLocal.insert(bestGenLocal);
            matchedTruth.insert(truthIndex);
            matchedGen.insert(genIndices_in[bestGenLocal]);
        }
    }
    return {matchedTruth, matchedGen};
}

} // namespace

WallPrfResult computeGlobalWallMetrics(const nlohmann::json &truth_in,
                                       const nlohmann::json &generated_in)
{
    const std::vector<RoomRecord> truthRooms = parseRooms(truth_in);
    const std::vector<RoomRecord> genRooms   = parseRooms(generated_in);
    const std::vector<WallRecord> truthWalls = parseWalls(truth_in);
    const std::vector<WallRecord> genWalls   = parseWalls(generated_in);

    const std::vector<std::pair<std::size_t, std::size_t>> roomPairs =
        matchRoomsOptimal(truthRooms, genRooms);

    std::set<std::size_t> matchedTruthWallIndices;
    std::set<std::size_t> matchedGenWallIndices;

    for (const auto &[truthRoomIndex, genRoomIndex] : roomPairs)
    {
        const int truthRoomId = truthRooms[truthRoomIndex].id;
        const int genRoomId   = genRooms[genRoomIndex].id;

        std::vector<std::size_t> truthWallIndicesInRoom;
        for (std::size_t index = 0; index < truthWalls.size(); ++index)
        {
            if (truthWalls[index].roomId == truthRoomId)
            {
                truthWallIndicesInRoom.push_back(index);
            }
        }
        std::vector<std::size_t> genWallIndicesInRoom;
        for (std::size_t index = 0; index < genWalls.size(); ++index)
        {
            if (genWalls[index].roomId == genRoomId)
            {
                genWallIndicesInRoom.push_back(index);
            }
        }

        const auto [matchedTruthLocal, matchedGenLocal] =
            matchWallsWithinRoomPair(truthWalls,
                                     truthWallIndicesInRoom,
                                     genWalls,
                                     genWallIndicesInRoom);
        matchedTruthWallIndices.insert(matchedTruthLocal.begin(),
                                       matchedTruthLocal.end());
        matchedGenWallIndices.insert(matchedGenLocal.begin(),
                                     matchedGenLocal.end());
    }

    /* Global denominator: every truth/generated wall, not only those whose
     * room happened to match (the G17 fix). matchedTruthWallIndices and
     * matchedGenWallIndices necessarily have equal size, since
     * matchWallsWithinRoomPair only ever inserts one-to-one pairs. */
    WallPrfResult result;
    result.matched     = matchedTruthWallIndices.size();
    result.generated   = genWalls.size();
    result.groundTruth = truthWalls.size();
    result.recall      = result.groundTruth > 0
                             ? static_cast<double>(result.matched) /
                              static_cast<double>(result.groundTruth)
                             : 0.0;
    result.precision   = result.generated > 0
                             ? static_cast<double>(result.matched) /
                                 static_cast<double>(result.generated)
                             : 0.0;
    result.f1          = (result.recall + result.precision) > 0.0
                             ? 2.0 * result.recall * result.precision /
                          (result.recall + result.precision)
                             : 0.0;
    return result;
}

} // namespace test
} // namespace core
} // namespace vs_graphs
