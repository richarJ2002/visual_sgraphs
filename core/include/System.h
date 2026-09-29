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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <atomic>
#include <cstdint>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include <rclcpp/logging.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <unistd.h>

// JSON library
#include "SystemStatus.h"
#include "Thirdparty/nlohmann/json.hpp"
#include "VerboseStatus.h"

#include "Atlas.h"
#include "DatabaseParser.h"
#include "ImuTypes.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "ResetCause.h"
#include "Semantic/SemanticReportCache/objects/SemanticReportCacheEntry.h"
#include "Types/objects/SystemParams.h"
#include "Utils/Settings/objects/Settings.h"

namespace vs_graphs
{
namespace core
{
class KeyFrameDatabase;
} // namespace core
} // namespace vs_graphs

namespace vs_graphs
{
namespace core
{
namespace geometric
{
class Plane;
} // namespace geometric
} // namespace core
} // namespace vs_graphs

namespace vs_graphs
{
namespace core
{
namespace semantic
{
class Marker;
class Passage;
class Room;
} // namespace semantic
} // namespace core
} // namespace vs_graphs

namespace vs_graphs
{
namespace core
{

/*!
 * @brief       Logging severity used by the ORB-SLAM3 core. All core output is
 *              routed through ROS 2's logging framework on the global
 *              "visual_sgraphs" logger so it shares one severity control with
 *              the ROS-facing layer.
 */
class Verbose
{
  public:
    enum VerbosityLevel
    {
        /*!
         * @brief       Quiet mode: only warning-level messages are emitted.
         *              Default level that preserves quiet behaviour when no
         *              verbosity is explicitly set by the caller.
         */
        VERBOSITY_QUIET = 0,

        /*!
         * @brief       Normal mode: informational messages are emitted at
         *              RCLCPP_INFO severity.
         */
        VERBOSITY_NORMAL = 1,

        /*!
         * @brief       Verbose mode: detailed debug messages are emitted at
         *              RCLCPP_DEBUG severity.
         */
        VERBOSITY_VERBOSE = 2,

        /*!
         * @brief       Very verbose mode: additional debug detail beyond
         * Verbose, also emitted at RCLCPP_DEBUG severity.
         */
        VERBOSITY_VERY_VERBOSE = 3,

        /*!
         * @brief       Debug mode: maximum debug detail emitted at
         *              RCLCPP_DEBUG severity. Use for development diagnostics.
         */
        VERBOSITY_DEBUG = 4
    };

    static VerbosityLevel th;

  public:
    /*!
     * @brief       Emit a core diagnostic through the global "visual_sgraphs"
     *              logger with a severity derived from the requested level.
     *              The static gate `th` still filters by level: only messages
     *              with `lev <= th` are emitted.
     */
    [[nodiscard]] static VerboseStatus printMess(std::string    message_in,
                                                 VerbosityLevel level_in)
    {
        if (level_in <= th)
        {
            switch (level_in)
            {
            case VERBOSITY_QUIET:
                RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                            "%s",
                            message_in.c_str());
                break;
            case VERBOSITY_NORMAL:
                RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"),
                            "%s",
                            message_in.c_str());
                break;
            case VERBOSITY_VERBOSE:
            case VERBOSITY_VERY_VERBOSE:
            case VERBOSITY_DEBUG:
                RCLCPP_DEBUG(rclcpp::get_logger("visual_sgraphs"),
                             "%s",
                             message_in.c_str());
                break;
            }
        }

        return VerboseStatus::VERBOSE_STATUS_SUCCESS;
    }

    /*!
     * @brief       Set the global verbose threshold for core logging routed
     *              through ROS 2's "visual_sgraphs" logger. Only messages
     *              with a level less than or equal to this threshold will
     *              be emitted.
     *
     * @param[in]   threshold_in
     *              The verbose level threshold. See @ref
     * Verbose::VerbosityLevel "VERBOSITY_QUIET", VERBOSITY_NORMAL,
     * VERBOSITY_VERBOSE, VERBOSITY_VERY_VERBOSE, or VERBOSITY_DEBUG.
     */
    [[nodiscard]] static VerboseStatus setTh(VerbosityLevel threshold_in)
    {
        th = threshold_in;

        return VerboseStatus::VERBOSE_STATUS_SUCCESS;
    }

    /*!
     * @brief       Map a user-supplied log level string ("quiet", "error",
     *              "warn", "info", "debug") to the matching `VerbosityLevel`
     * threshold. Unknown strings default to `VERBOSITY_QUIET` so an unset or
     *              invalid value preserves the current quiet behaviour.
     */
    [[nodiscard]] static VerboseStatus
        parseVerbosityLevel(const std::string &level_in,
                            VerbosityLevel    &verbosityLevel_out)
    {
        if (level_in == "debug")
        {
            verbosityLevel_out = VERBOSITY_DEBUG;
            return VerboseStatus::VERBOSE_STATUS_SUCCESS;
        }
        if (level_in == "info")
        {
            verbosityLevel_out = VERBOSITY_NORMAL;
            return VerboseStatus::VERBOSE_STATUS_SUCCESS;
        }
        if (level_in == "warn")
        {
            verbosityLevel_out = VERBOSITY_QUIET;
            return VerboseStatus::VERBOSE_STATUS_SUCCESS;
        }
        if (level_in == "error")
        {
            verbosityLevel_out = VERBOSITY_QUIET;
            return VerboseStatus::VERBOSE_STATUS_SUCCESS;
        }
        if (level_in == "quiet")
        {
            verbosityLevel_out = VERBOSITY_QUIET;
            return VerboseStatus::VERBOSE_STATUS_SUCCESS;
        }

        verbosityLevel_out = VERBOSITY_QUIET;
        return VerboseStatus::VERBOSE_STATUS_SUCCESS;
    }
};

/*!
 * @brief       Emit an INFO line from the ORB-SLAM3 core through the global
 *              "visual_sgraphs" logger as a printf-style message.
 */
#define VSLAM_LOG_INFO(...)                                                    \
    RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"), __VA_ARGS__)

/*!
 * @brief       Emit a WARN line from the ORB-SLAM3 core through the global
 *              "visual_sgraphs" logger as a printf-style message.
 */
#define VSLAM_LOG_WARN(...)                                                    \
    RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), __VA_ARGS__)

/*!
 * @brief       Emit an ERROR line from the ORB-SLAM3 core through the global
 *              "visual_sgraphs" logger as a printf-style message.
 */
#define VSLAM_LOG_ERROR(...)                                                   \
    RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), __VA_ARGS__)

/*!
 * @brief       Emit a DEBUG line from the ORB-SLAM3 core through the global
 *              "visual_sgraphs" logger as a printf-style message.
 */
#define VSLAM_LOG_DEBUG(...)                                                   \
    RCLCPP_DEBUG(rclcpp::get_logger("visual_sgraphs"), __VA_ARGS__)

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
namespace utils
{
namespace settings
{
class Settings;
} // namespace settings
} // namespace utils
class SemanticSegmentation;
class SemanticsManager;

class System
{
  public:
    /*!
     * @brief       Enumerator to indicate which sensor is being used by the
     *              system.
     */
    enum SensorType
    {
        /*!
         * @brief       A sensor is not set. This is considered invalid.
         */
        NOT_SET = -1,

        /*!
         * @brief       Monocular Visual Camera setup.
         */
        MONOCULAR = 0,

        /*!
         * @brief       Stereo Visual Camera setup.
         */
        STEREO = 1,

        /*!
         * @brief       RGBD Visual Camera setup.
         */
        RGBD = 2,

        /*!
         * @brief       Visual Monocular Camera with IMU setup.
         */
        IMU_MONOCULAR = 3,

        /*!
         * @brief       Visual Stereo Camera with IMU setup.
         */
        IMU_STEREO = 4,

        /*!
         * @brief       RGBD Visual Camera with IMU setup.
         */
        IMU_RGBD = 5,
    };

    /*!
     * @brief       Enumerator to indicate which file type is being used.
     */
    enum FileType
    {
        /*!
         * @brief       Text file type is being used.
         */
        TEXT_FILE = 0,

        /*!
         * @brief       Binary file type is being used.
         */
        BINARY_FILE = 1,
    };

    /*!
     * @brief       Struct which indicates the health of the passage semantic
     *              element.
     */
    struct PassageHealth
    {
        /*!
         * @brief       ID of the passage semantic element.
         *
         * @frame       N/A
         * @unit        N/A
         */
        int id{-1};

        /*!
         * @brief       Flag to indicate if passage is passable or blocked. If
         *              `true` then the passage is considered to be passable.
         *
         * @frame       N/A
         * @unit        N/A
         */
        bool isPassable{false};

        /*!
         * @brief       Room id from which the passage was first observed
         *              (the primary/observing side).
         *
         *                                   primaryRoomId
         *                            (Room Passage Was Observed)
         *                                        ||
         *                                        \/
         *                                      Passage
         *                                        ||
         *                                        \/
         *                                  secondaryRoomId
         *                      (Room connecting to the observed passage)
         *
         * @frame       N/A
         * @unit        N/A
         */
        int primaryRoomId{-1};

        /*!
         * @brief       Room id of the room which the passage connects to
         *              from the observing/primary side.
         *
         *                                   primaryRoomId
         *                            (Room Passage Was Observed)
         *                                        ||
         *                                        \/
         *                                      Passage
         *                                        ||
         *                                        \/
         *                                  secondaryRoomId
         *                      (Room connecting to the observed passage)
         *
         * @frame       N/A
         * @unit        N/A
         */
        int secondaryRoomId{-1};

        /*!
         * @brief       Number of traversals from the primary/observing side
         *              into the secondary/far side.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t primaryTraversalCount{0U};

        /*!
         * @brief       Number of traversals from the secondary/far side
         *              back to the primary/observing side.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t secondaryTraversalCount{0U};

        /*!
         * @brief       Number of untraversed/unknown observations.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t unknownCount{0U};
    };

    /*!
     * @brief       Struct indicating the health of a room semantic element,
     *              listing associated passage IDs.
     *
     * @frame       N/A
     * @unit        N/A
     */
    struct RoomHealth
    {

        /*!
         * @brief       ID of the room semantic element.
         *
         * @frame       N/A
         * @unit        N/A
         */
        int id{-1};

        /*!
         * @brief       Vector of passage IDs associated with this room.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::vector<int> passageIds;
    };

    /*!
     * @brief       Struct indicating the health of a floor semantic element,
     *              listing associated room IDs.
     *
     * @frame       N/A
     * @unit        N/A
     */
    struct FloorHealth
    {

        /*!
         * @brief       ID of the floor semantic element.
         *
         * @frame       N/A
         * @unit        N/A
         */
        int id{-1};

        /*!
         * @brief       Vector of room IDs located on this floor.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::vector<int> roomIds;
    };

    /*!
     * @brief       Snapshot of the mission health status, capturing tracking,
     *              semantic, and loop closure state at a given point in time.
     *
     * This structure is populated by @ref GetMissionHealthSnapshot
     * "GetMissionHealthSnapshot()" and @ref GetSemanticReportCacheEntry
     * "GetSemanticReportCacheEntry()". It provides a comprehensive view of the
     * system's current state, including pose validity, map statistics, and
     * semantic segmentation progress. A default-constructed instance is
     * returned when no SemanticsManager exists or no complete semantic
     * evaluation cycle has completed yet.
     *
     * @frame       N/A
     * @unit        N/A
     */
    struct MissionHealthSnapshot
    {

        /*!
         * @brief       Timestamp of the frame at which this snapshot was taken.
         *
         * @frame       N/A
         * @unit        seconds
         */
        double frameTimestamp{0.0};

        /*!
         * @brief       Current tracking state code. Corresponds to the tracking
         *              state machine status (e.g. INITIALIZED, TRACKING, LOST).
         *
         * @frame       N/A
         * @unit        N/A
         */
        int trackingState{-1};

        /*!
         * @brief       Number of inliers from the most recent tracking frame.
         *              Higher values indicate more reliable tracking.
         *
         * @frame       N/A
         * @unit        N/A
         */
        int trackingInliers{0};

        /*!
         * @brief       Whether the IMU subsystem is active. When `true`, IMU
         * data is being used for pose estimation.
         *
         * @frame       N/A
         * @unit        N/A
         */
        bool isInertial{false};

        /*!
         * @brief       Whether the IMU has been initialized and its bias
         * estimates are valid. Initialization typically requires a period of
         *              stationary operation.
         *
         * @frame       N/A
         * @unit        N/A
         */
        bool isInertialInitialized{false};

        /*!
         * @brief       Whether the camera pose is currently valid. If `false`,
         * the pose should not be relied upon for navigation or planning.
         *
         * @frame       N/A
         * @unit        N/A
         */
        bool isPoseValid{false};

        /*!
         * @brief       Current camera pose in the world frame, representing the
         *              estimated position and orientation of the camera.
         *
         * @frame       World
         * @unit        meters / radians (Sophus SE3f convention)
         */
        Sophus::SE3f cameraPose_World;

        /*!
         * @brief       Identifier of the most recently processed map.
         * Incremented on map restarts or when a new map is loaded.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t mapId{0U};

        /*!
         * @brief       Total count of map instances or map reloads that have
         *              occurred since the system started.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint32_t mapCount{0U};

        /*!
         * @brief       Total number of keyframes currently in the map.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint32_t keyFrameCount{0U};

        /*!
         * @brief       Total number of system resets that have occurred since
         *              initialization. Incremented by @ref Reset "Reset()" and
         *              @ref RequestResetActiveMapWithCause
         * "RequestResetActiveMapWithCause().
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t resetCount{0U};

        /*! Number of coherent RGB-D packets accepted by the frontend. */
        std::uint64_t rgbdFrontendAcceptedCount{0U};

        /*! Number of accepted RGB-D packets reaching a terminal worker state.
         */
        std::uint64_t rgbdFrontendProcessedCount{0U};

        /*! Number of pending RGB-D packets replaced by a newer packet. */
        std::uint64_t rgbdFrontendOverwrittenCount{0U};

        /*! True while the RGB-D worker owns a packet. */
        bool isRgbdFrontendWorkerInFlight{false};

        /*! Sensor timestamp of the latest successfully tracked RGB-D packet. */
        std::int64_t rgbdFrontendLastProcessedSensorTimestampNanoseconds{0};

        /*!
         * @brief       Number of keyframes currently in-flight and queued for
         *              semantic segmentation (i.e. published to the segmenter
         * but not yet returned). See also @ref segmentationReturnedCount and
         * @ref lastReturnedKeyFrameId.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t segmentationPublishedCount{0U};

        /*!
         * @brief       Number of keyframes for which semantic segmentation
         * results have been received back from the segmentation pipeline. When
         * combined with @ref segmentationPublishedCount, the difference
         * indicates how many keyframes remain in-flight.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t segmentationReturnedCount{0U};

        /*!
         * @brief       Identifier of the last keyframe whose segmentation
         * result was returned. Keyframes with IDs greater than this value are
         * still in-flight or pending.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t lastReturnedKeyFrameId{0U};

        /*! Number of returned segmentations enqueued for plane processing. */
        std::uint64_t segmentationEnqueuedCount{0U};

        /*! Number of segmentation work items removed from the work queue. */
        std::uint64_t segmentationDequeuedCount{0U};

        /*! Number of segmentation work items reaching a terminal outcome. */
        std::uint64_t segmentationTerminalCount{0U};

        /*! Number of work items accepted into active-map plane association. */
        std::uint64_t segmentationAcceptedCount{0U};

        /*! Number of work items discarded by the bounded-queue policy. */
        std::uint64_t segmentationDroppedCount{0U};

        /*! Number of terminal results rejected for a missing keyframe. */
        std::uint64_t segmentationMissingKeyFrameCount{0U};

        /*! Number of terminal results rejected for unavailable cloud data. */
        std::uint64_t segmentationMissingCloudCount{0U};

        /*! Number of terminal results rejected because their map was stale. */
        std::uint64_t segmentationStaleMapCount{0U};

        /*! Identifier of the most recent terminally processed keyframe. */
        std::uint64_t lastTerminalKeyFrameId{0U};

        /*! Current number of returned results awaiting processing. */
        std::uint32_t segmentationQueueDepth{0U};

        /*! Largest processing queue depth observed during this process. */
        std::uint32_t segmentationQueueHighWatermark{0U};

        /*!
         * @brief       Whether the latest keyframe's pose is currently valid.
         * If `false`, the latest keyframe pose should not be used for critical
         * decisions.
         *
         * @frame       N/A
         * @unit        N/A
         */
        bool isLatestKeyFramePoseValid{false};

        /*!
         * @brief       Timestamp of the latest keyframe that has been
         * processed.
         *
         * @frame       N/A
         * @unit        seconds
         */
        double latestKeyFrameTimestamp{0.0};

        /*!
         * @brief       Pose of the latest keyframe in the world frame,
         * representing the estimated position and orientation at the time of
         * that keyframe's capture.
         *
         * @frame       World
         * @unit        meters / radians (Sophus SE3f convention)
         */
        Sophus::SE3f latestKeyFramePose_World;

        /*!
         * @brief       Identifier of the current room as determined by the
         *              semantic mapping subsystem. This room is the one most
         *              recently observed by the system.
         *
         * @frame       N/A
         * @unit        N/A
         */
        int currentRoomId{-1};

        /*!
         * @brief       Identifier of the last known room, saved before a
         * potential room change or reset. Used for carryover of room context
         *              across map restarts.
         *
         * @frame       N/A
         * @unit        N/A
         */
        int lastKnownRoomId{-1};

        /*!
         * @brief       Number of rooms that have been confirmed (i.e. their
         *              topology has been validated and they are part of the
         *              persistent map).
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint32_t confirmedRoomCount{0U};

        /*!
         * @brief       Number of rooms that are currently unresolved, meaning
         *              their topology has not yet been fully validated or they
         *              are still being mapped.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint32_t unresolvedRoomCount{0U};

        /*!
         * @brief       Number of floor-room linkage observations recorded. This
         *              counts how many times a room has been associated with a
         *              floor in the semantic map.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint32_t floorRoomLinkCount{0U};

        /*!
         * @brief       Vector of room health entries, each describing the
         *              passages associated with a room and its traversal stats.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::vector<RoomHealth> rooms;

        /*!
         * @brief       Vector of floor health entries, each listing the rooms
         *              that exist on that floor.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::vector<FloorHealth> floors;

        /*!
         * @brief       Vector of passage health entries, each describing the
         *              traversal counts and room connections for a passage.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::vector<PassageHealth> passages;

        /*!
         * @brief       Sequence number of the most recent loop closure event.
         *              Incremented each time a loop is closed.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t loopSequence{0U};

        /*!
         * @brief       Number of loop closures that have been accepted (i.e.
         *              their pose graph optimization converged successfully).
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint32_t acceptedLoopCount{0U};

        /*!
         * @brief       Number of loop closures that have been rejected (i.e.
         *              their pose graph optimization failed or was deemed
         *              inconsistent).
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint32_t rejectedLoopCount{0U};

        /*!
         * @brief       Whether a loop event has occurred since the last
         * snapshot. A loop closure was detected and processed.
         *
         * @frame       N/A
         * @unit        N/A
         */
        bool hasLoopEvent{false};

        /*!
         * @brief       Whether the most recent loop closure was accepted
         * (`true`) or rejected (`false`).
         *
         * @frame       N/A
         * @unit        N/A
         */
        bool wasLastLoopAccepted{false};

        /*!
         * @brief       Map ID associated with the most recent loop closure
         * event.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t lastLoopMapId{0U};

        /*!
         * @brief       Keyframe ID that was current at the time the most recent
         *              loop was initiated.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t lastLoopCurrentKeyFrameId{0U};

        /*!
         * @brief       Keyframe ID that was matched against the current frame
         *              during the most recent loop closure search.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::uint64_t lastLoopMatchedKeyFrameId{0U};

        /*!
         * @brief       Timestamp of the current frame when the most recent loop
         *              closure was initiated.
         *
         * @frame       N/A
         * @unit        seconds
         */
        double lastLoopCurrentTimestamp{0.0};

        /*!
         * @brief       Timestamp at which the loop closure match was found,
         *              corresponding to the matched keyframe's timestamp.
         *
         * @frame       N/A
         * @unit        seconds
         */
        double lastLoopMatchedTimestamp{0.0};

        /*!
         * @brief       Human-readable reason string for the most recent loop
         *              closure event. May describe why a loop was accepted or
         *              rejected, or provide details about the loop detection.
         *
         * @frame       N/A
         * @unit        N/A
         */
        std::string lastLoopReason;
    };

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief       Creates an empty system. Nothing is loaded and no thread
     *              runs until initialize() succeeds.
     */
    System();

    /*!
     * @brief       Loads the settings, vocabulary and (optionally) a saved
     *              map, then starts the Local Mapping, Loop Closing, semantic
     *              and (optionally) Viewer threads. Call it once, right after
     *              construction; the system is usable only if it succeeds.
     *
     * @param[in]   vocabularyFile_in
     *              Path to the ORB vocabulary file used for place recognition
     *              and feature matching (an ORB-SLAM3 binary vocabulary).
     * @param[in]   settingsFile_in
     *              Path to the settings YAML file (cameras, IMU and other
     *              system parameters).
     * @param[in]   sysParamsFile_in
     *              Path to the vS-Graphs system parameters file.
     * @param[in]   sensor_in
     *              Camera/IMU configuration; decides which track call is used.
     * @param[in]   shouldUseViewer_in
     *              True to start the Viewer thread; false for headless runs.
     * @param[in]   initialFr_in
     *              Index of the first frame to process (to resume a sequence).
     * @param[in]   sequence_in
     *              Optional name of the sequence, used in logs.
     * @param[in]   verboseLevel_in
     *              Threshold for core logging routed through ROS 2's
     *              "visual_sgraphs" logger.
     *
     * @return      SYSTEM_STATUS_SUCCESS when everything is loaded and running;
     *              SYSTEM_STATUS_SETTINGS_UNREADABLE,
     *              SYSTEM_STATUS_VOCABULARY_UNREADABLE or
     *              SYSTEM_STATUS_ATLAS_UNREADABLE when that file cannot be read
     *              (no thread has been started then).
     */
    [[nodiscard]] SystemStatus
        initialize(const string                 &vocabularyFile_in,
                   const string                 &settingsFile_in,
                   const string                 &sysParamsFile_in,
                   const SensorType              sensor_in,
                   const bool                    shouldUseViewer_in = true,
                   const int                     initialFr_in       = 0,
                   const string                 &sequence_in = std::string(),
                   const Verbose::VerbosityLevel verboseLevel_in =
                       Verbose::VERBOSITY_QUIET);

    /*!
     * @brief       Stops and joins all worker threads and frees the thread
     *              objects. Safe to run after Shutdown(); join() is only
     *              performed here, never in Shutdown().
     */
    ~System();

    /*!
     * @brief       Process the given stereo frame for tracking. Images must be
     *              synchronized and rectified.
     *
     * @param       imageLeft_in
     *              The input RGB image (CV_8UC3) or grayscale (CV_8U) from the
     *              left camera.
     *
     * @param       imageRight_in
     *              The input RGB image (CV_8UC3) or grayscale (CV_8U) from the
     *              right camera.
     *
     * @param       timestamp_in
     *              the timestamp of the frame.
     *
     * @param       imuMeas_in
     *              the vector of IMU measurements.
     *
     * @param       filename_in
     *              the name of the file.
     *
     * @param       markers_in
     *              the vector of fiducial markers.
     *
     * @param[out] cameraPose_out The camera pose (empty if tracking fails)
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus trackStereo(
        const cv::Mat                   &imageLeft_in,
        const cv::Mat                   &imageRight_in,
        const double                    &timestamp_in,
        Sophus::SE3f                    &cameraPose_out,
        const std::vector<IMU::Point>   &imuMeas_in = std::vector<IMU::Point>(),
        string                           filename_in = "",
        const vector<semantic::Marker *> markers_in =
            vector<semantic::Marker *>{});

    /*!
     * @brief       Process the given rgbd frame for tracking. The DepthMap must
     *              be registered to the RGB frame.
     *
     * @param       colorImage_in
     *              The input RGB image (CV_8UC3) or grayscale (CV_8U).
     *
     * @param       depthmap_in
     *              The input DepthMap (CV_32F).
     *
     * @param       p_mainCloud_in
     *              The main input PointCloud before filtering.
     *
     * @param       timestamp_in
     *              The timestamp of the frame.
     *
     * @param       imuMeas_in
     *              The vector of IMU measurements.
     *
     * @param       filename_in
     *              The name of the file.
     *
     * @param       markers_in
     *              The vector of fiducial markers.
     *
     * @param[out] cameraPose_out The camera pose (empty if tracking fails)
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus trackRGBD(
        const cv::Mat                                &colorImage_in,
        const cv::Mat                                &depthmap_in,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_mainCloud_in,
        const double                                 &timestamp_in,
        Sophus::SE3f                                 &cameraPose_out,
        const std::vector<IMU::Point>   &imuMeas_in = std::vector<IMU::Point>(),
        string                           filename_in = "",
        const vector<semantic::Marker *> markers_in =
            vector<semantic::Marker *>{});

    /*!
     * @brief       Process the given stereo frame for tracking. Images must be
     *              synchronized and rectified.
     *
     * @param       image_in
     *              The input RGB image (CV_8UC3) or grayscale (CV_8U) from the
     *              left camera.
     *
     * @param       timestamp_in
     *              The timestamp of the frame.
     *
     * @param       imuMeas_in
     *              The vector of IMU measurements.
     *
     * @param       filename_in
     *              The name of the file.
     *
     * @param       markers_in
     *              The vector of fiducial markers.
     *
     * @param[out] cameraPose_out The camera pose (empty if tracking fails)
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus trackMonocular(
        const cv::Mat                   &image_in,
        const double                    &timestamp_in,
        Sophus::SE3f                    &cameraPose_out,
        const std::vector<IMU::Point>   &imuMeas_in = std::vector<IMU::Point>(),
        string                           filename_in = "",
        const vector<semantic::Marker *> markers_in =
            vector<semantic::Marker *>{});

    /*!
     * @brief       This stops local mapping thread (map building) and performs
     *              only camera tracking.
     */
    [[nodiscard]] SystemStatus activateLocalizationMode();

    /*!
     * @brief        This resumes local mapping thread and performs SLAM again.
     */
    [[nodiscard]] SystemStatus deactivateLocalizationMode();

    /*!
     * @brief       Get the current active map in Atlas.
     *
     * @param[out] p_currentMap_out The current active map.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus
        getCurrentMap(vs_graphs::core::Map *&p_currentMap_out);

    /*!
     * @brief       Get the Atlas owning every map in the system.
     *
     *              Exposed so ROS-layer consumers (e.g. the SGraph JSON
     *              archiver) can enumerate all active maps coherently. The
     *              Atlas remains owned by the System; the caller must not
     *              delete it.
     *
     * @param[out] p_atlas_out Pointer to the Atlas, or nullptr before
     * initialisation.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getAtlas(vs_graphs::core::Atlas *&p_atlas_out);

    /*!
     * @brief       Returns true if there have been a big map change (loop
     *              closure, global BA) since last call to this function.
     */
    [[nodiscard]] SystemStatus mapChanged(bool &hasMapChanged_out);

    [[nodiscard]] SystemStatus getMissionHealthSnapshot(
        System::MissionHealthSnapshot &missionHealthSnapshot_out,
        bool                           includeSemantics_in = true);

    /*!
     * @brief       Updates RGB-D frontend progress exposed through mission
     *              health. Safe to call from the callback or worker thread.
     *
     * @param[in]   acceptedCount_in
     *              Coherent packets accepted into the bounded frontend.
     * @param[in]   processedCount_in
     *              Accepted packets that reached a terminal worker outcome.
     * @param[in]   overwrittenCount_in
     *              Pending packets replaced before worker ownership.
     * @param[in]   isWorkerInFlight_in
     *              Whether the frontend worker currently owns a packet.
     * @param[in]   lastProcessedSensorTimestampNanoseconds_in
     *              Latest successfully tracked sensor timestamp, in
     *              nanoseconds.
     */
    [[nodiscard]] SystemStatus updateRgbdFrontendHealth(
        std::uint64_t acceptedCount_in,
        std::uint64_t processedCount_in,
        std::uint64_t overwrittenCount_in,
        bool          isWorkerInFlight_in,
        std::int64_t  lastProcessedSensorTimestampNanoseconds_in) noexcept;

    /*!
     * @brief       Returns a copied snapshot of the latest complete semantic
     *              evaluation cycle (SemanticsManager's own
     *              SemanticReportCache), or a meaningless default value when
     *              no SemanticsManager exists yet or no cycle has completed
     *              -- check IsSemanticReportCacheAvailable() first.
     */
    [[nodiscard]] SystemStatus getSemanticReportCacheEntry(
        semantic::SemanticReportCacheEntry &getSemanticReportCacheEntry_out)
        const;

    /*!
     * @brief       True once p_semanticsManager exists and has cached at least
     *              one complete semantic evaluation cycle.
     */
    [[nodiscard]] SystemStatus isSemanticReportCacheAvailable(
        bool &isSemanticReportCacheAvailable_out) const;

    /*!
     * @brief       Reset the system (clear Atlas or the active map).
     */
    [[nodiscard]] SystemStatus reset();

    /*!
     * @brief       Reset the active map (clear the current map while retaining
     *              the overall Atlas state). This is useful for restarting the
     *              system with a fresh map while keeping map topology and
     *              previously built map points.
     */
    [[nodiscard]] SystemStatus resetActiveMap();

    /*!
     * @brief       Requests an active-map reset while retaining its
     *              package-internal cause. Multiple unlike requests coalesced
     *              by the existing boolean reset flag are reported as such
     *              rather than assigned one misleading cause.
     *
     * @param[in]   cause_in
     *              The cause of the reset request. This is used to report
     *              multiple unlike requests as a combined cause rather than
     *              assigning one misleading cause.
     */
    [[nodiscard]] SystemStatus
        requestResetActiveMapWithCause(ResetCause cause_in);

    /*!
     * @brief       All threads will be requested to finish. It waits until all
     *              threads have finished. This function must be called before
     *              saving the trajectory to ensure a clean shutdown.
     */
    [[nodiscard]] SystemStatus shutdown();

    /*!
     * @brief       Reset the system (clear Atlas or the active map).
     *
     * @param[out] isShutDown_out `true` if the system was successfully reset,
     * `false` otherwise.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus isShutDown(bool &isShutDown_out);

    /*!
     * @brief       Save camera trajectory in the TUM RGB-D dataset format.
     *              Only for stereo and RGB-D. This method does not work for
     *              monocular.
     *
     * @note        Call first Shutdown()
     *
     * @note        See format details at:
     *              http://vision.in.tum.de/data/datasets/rgbd-dataset
     */
    [[nodiscard]] SystemStatus saveTrajectoryTUM(const string &filename_in);

    /*!
     * @brief       Save keyframe poses in the TUM RGB-D dataset format. This
     *              method works for all sensor input.
     *
     * @note        Call first Shutdown()
     *
     * @note        See format details at:
     *              http://vision.in.tum.de/data/datasets/rgbd-dataset
     */
    [[nodiscard]] SystemStatus
        saveKeyFrameTrajectoryTUM(const string &filename_in);

    /*!
     * @brief       Save camera trajectory in the EuRoC MAV dataset format.
     *              Only for stereo and RGB-D. This method does not work for
     *              monocular. Call Shutdown() before saving.
     *
     * @param[in]   filename_in
     *              Path to the output file where the trajectory will be saved
     *              in EuRoC format.
     *
     * @note        Call first Shutdown()
     *
     * @see         https://github.com/ethz-asl/euroc-dataset for format details
     */
    [[nodiscard]] SystemStatus saveTrajectoryEuRoC(const string &filename_in);

    /*!
     * @brief       Save keyframe poses in the EuRoC MAV dataset format. This
     *              method works for all sensor input. Call Shutdown() before
     *              saving.
     *
     * @param[in]   filename_in
     *              Path to the output file where the keyframe poses will be
     *              saved in EuRoC format.
     *
     * @note        Call first Shutdown()
     *
     * @see         https://github.com/ethz-asl/euroc-dataset for format details
     */
    [[nodiscard]] SystemStatus
        saveKeyFrameTrajectoryEuRoC(const string &filename_in);

    /*!
     * @brief       Save camera trajectory in the EuRoC MAV dataset format,
     *              including map data. Only for stereo and RGB-D. Call
     *              Shutdown() before saving.
     *
     * @param[in]   filename_in
     *              Path to the output file where the trajectory will be saved.
     *
     * @param[in]   p_map_in
     *              Pointer to the map to include in the trajectory save.
     *
     * @note        Call first Shutdown()
     *
     * @see         https://github.com/ethz-asl/euroc-dataset for format details
     */
    [[nodiscard]] SystemStatus saveTrajectoryEuRoC(const string &filename_in,
                                                   Map          *p_map_in);

    /*!
     * @brief       Save keyframe poses in the EuRoC MAV dataset format,
     *              including map data. Works for all sensor input. Call
     *              Shutdown() before saving.
     *
     * @param[in]   filename_in
     *              Path to the output file where the keyframe poses will be
     *              saved.
     *
     * @param[in]   p_map_in
     *              Pointer to the map to include in the keyframe trajectory
     * save.
     *
     * @note        Call first Shutdown()
     *
     * @see         https://github.com/ethz-asl/euroc-dataset for format details
     */
    [[nodiscard]] SystemStatus
        saveKeyFrameTrajectoryEuRoC(const string &filename_in, Map *p_map_in);

    /*!
     * @brief       Save data used for initialization debug. This dump includes
     *              keyframe poses, map point positions, and other debugging
     *              information useful for diagnosing the initialization phase.
     *
     * @param[in]   initialIndex_in
     *              Index specifying which initialization debug data to save.
     *              Multiple debug dumps may be available for different
     *              initialization attempts.
     */
    [[nodiscard]] SystemStatus saveDebugData(const int &initialIndex_in);

    /*!
     * @brief       Save camera trajectory in the KITTI dataset format. Only for
     *              stereo and RGB-D. This method does not work for monocular.
     *
     * @note        Call first Shutdown()
     *
     * @note        See format details at:
     *              http://vision.in.tum.de/data/datasets/rgbd-dataset
     */
    [[nodiscard]] SystemStatus saveTrajectoryKITTI(const string &filename_in);

    /*!
     * @brief       Save the map to a file. The format (text or binary) is
     *              determined by the system configuration.
     *
     * @param[in]   filename_in
     *              Path to the output file where the map will be saved.
     *
     * @param[out] isSaved_out `true` if the map was saved successfully, `false`
     * otherwise. Returns `false` if the system is not properly initialized or
     * if saving is not supported for the current sensor configuration.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus saveMap(const string &filename_in,
                                       bool         &isSaved_out);

    /*!
     * @brief       Save map points as a PCD (Point Cloud Data) file. This can
     *              be used for offline analysis or visualization of the map
     *              points.
     *
     * @param[in]   filename_in
     *              Path to the output PCD file where map points will be saved.
     *
     * @param[out] isSaved_out `true` if the map points were saved successfully,
     * `false` otherwise. Returns `false` if the system is not properly
     * initialized or if there are no map points to save.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus saveMapPointsAsPCD(const string &filename_in,
                                                  bool         &isSaved_out);

    /*!
     * @brief       Get the current tracking state code. This reflects the
     *              current state of the tracking system (e.g. INITIALIZED,
     *              TRACKING, LOST). Updated after each frame processing.
     *
     * @param[out] trackingState_out Tracking state code integer.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getTrackingState(int &trackingState_out);

    /*!
     * @brief       Get a copy of the current frame image. The returned image
     *              is in the same format as the input (monocular, stereo, or
     *              RGB-D depending on the sensor configuration).
     *
     * @param[out] currentFrame_out Current frame as a cv::Mat. May be empty if
     * no frame has been processed yet.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getCurrentFrame(cv::Mat &currentFrame_out);

    /*!
     * @brief       Get all rooms in the current map. Rooms are semantic
     *              elements that group related passages and have associated
     *              traversal statistics.
     *
     * @param[out] allRooms_out Vector of pointers to Room objects. May be empty
     * if no rooms have been created yet.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus
        getAllRooms(std::vector<semantic::Room *> &allRooms_out);

    /*!
     * @brief       Get all floors in the current map. Floors are semantic
     *              elements that group related rooms.
     *
     * @param[out] allFloors_out Vector of pointers to Floor objects. May be
     * empty if no floors have been created yet.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getAllFloors(
        std::vector<vs_graphs::core::semantic::Floor *> &allFloors_out);

    /*!
     * @brief       Get all planes in the current map. Planes represent
     *              geometric planes detected in the environment (e.g., walls,
     *              floors, ceilings).
     *
     * @param[out] allPlanes_out Vector of pointers to Plane objects. May be
     * empty if no planes have been detected yet.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus
        getAllPlanes(std::vector<geometric::Plane *> &allPlanes_out);

    /*!
     * @brief       Get all doors in the current map. Doors are semantic
     *              elements representing doorways between rooms.
     *
     * @param[out] allDoors_out Vector of pointers to Door objects. May be empty
     * if no doors have been detected yet.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus
        getAllDoors(std::vector<vs_graphs::core::Door *> &allDoors_out);

    /*!
     * @brief       Get all markers (fiducial markers/AprilTags) in the current
     *              map. Markers are used for place recognition and
     *              localization.
     *
     * @param[out] allMarkers_out Vector of pointers to Marker objects. May be
     * empty if no markers have been detected yet.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus
        getAllMarkers(std::vector<semantic::Marker *> &allMarkers_out);

    /*!
     * @brief       Get all passages in the current map. Passages connect rooms
     *              and have traversal statistics tracking how many times they
     *              have been crossed.
     *
     * @param[out] allPassages_out Vector of pointers to Passage objects. May be
     * empty if no passages have been detected yet.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getAllPassages(
        std::vector<vs_graphs::core::semantic::Passage *> &allPassages_out);

    /*!
     * @brief       Get all keyframes in the current map. Keyframes represent
     *              key poses from which the map was built.
     *
     * @param[out] allKeyFrames_out Vector of pointers to KeyFrame objects. May
     * be empty if no keyframes have been created yet.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus
        getAllKeyFrames(std::vector<KeyFrame *> &allKeyFrames_out);

    /*!
     * @brief       Get all map points in the current map. Map points are
     *              3D points that have been triangulated and tracked.
     *
     * @param[out] allMapPoints_out Vector of pointers to MapPoint objects. May
     * be empty if no map points have been created yet.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus
        getAllMapPoints(std::vector<MapPoint *> &allMapPoints_out);

    /*!
     * @brief       Get only the map points that are currently being tracked.
     *              These are map points that have been observed in the most
     *              recent frame and are likely to remain in the map.
     *
     * @param[out] trackedMapPoints_out Vector of pointers to MapPoint objects
     * currently being tracked. May be empty if no points are being tracked.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus
        getTrackedMapPoints(std::vector<MapPoint *> &trackedMapPoints_out);

    /*!
     * @brief       Get all keyframe poses in the current map. Each pose
     *              represents the camera position and orientation at the
     *              time the keyframe was captured.
     *
     * @param[out] allKeyframePoses_out Vector of Sophus::SE3f poses, one per
     * keyframe. May be empty if no keyframes have been created yet.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus
        getAllKeyframePoses(std::vector<Sophus::SE3f> &allKeyframePoses_out);

    /*!
     * @brief       Get the un-tracked keypoints from the current frame. These
     *              are keypoints that were detected but not yet associated
     *              with MapPoints.
     *
     * @param[out] trackedKeyPointsUn_out Vector of cv::KeyPoint objects
     * representing un-tracked keypoints. May be empty if all keypoints are
     * tracked.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getTrackedKeyPointsUn(
        std::vector<cv::KeyPoint> &trackedKeyPointsUn_out);

    /*!
     * @brief       Get the pose of a specific keyframe.
     *
     * @param[in]   p_keyFrame_in
     *              Pointer to the KeyFrame whose pose is requested. Must not
     *              be null and must belong to the current map.
     *
     * @param[out] keyFramePose_out The camera pose (Sophus::SE3f) of the
     * requested keyframe. Returns an empty pose if the keyframe pointer is
     * invalid.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getKeyFramePose(KeyFrame     *p_keyFrame_in,
                                               Sophus::SE3f &keyFramePose_out);

    /*!
     * @brief       Get the camera pose in the world frame. This is the
     *              estimated position and orientation of the camera relative
     *              to the world origin.
     *
     * @param[out] camTwc_out Camera pose as Sophus::SE3f. May be invalid if the
     * system has not yet initialized the pose.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getCamTwc(Sophus::SE3f &camTwc_out);

    /*!
     * @brief       Get the IMU pose in the body frame. Represents the
     *              estimated position and orientation of the IMU relative
     *              to the body frame.
     *
     * @param[out] imuTwb_out IMU pose as Sophus::SE3f. May be invalid if IMU
     * data has not been sufficiently processed.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getImuTwb(Sophus::SE3f &imuTwb_out);

    /*!
     * @brief       Get the IMU velocity in the body frame. Represents the
     *              linear velocity of the IMU in the body frame.
     *
     * @param[out] imuVwb_out IMU velocity as Eigen::Vector3f. May be invalid if
     * IMU data has not been sufficiently processed.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getImuVwb(Eigen::Vector3f &imuVwb_out);

    /*!
     * @brief       Check whether IMU preintegration is active. When `true`,
     *              IMU preintegrated measurements are being used for pose
     *              estimation, which reduces drift between IMU updates.
     *
     * @param[out] isImuPreintegrated_out `true` if IMU preintegration is
     * enabled and active, `false` otherwise.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus isImuPreintegrated(bool &isImuPreintegrated_out);

    /*!
     * @brief       Get the time elapsed since IMU initialization. This
     *              represents how long the IMU has been running and
     *              accumulating data since it was first started.
     *
     * @param[out] timeFromIMUInit_out Time in seconds since IMU initialization.
     * A value greater than ~0.1 seconds typically indicates the IMU has
     * converged and is providing reliable data.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getTimeFromIMUInit(double &timeFromIMUInit_out);

    /*!
     * @brief       Check whether the system considers its current state as
     *              finished/initialized. The system is considered finished
     *              when the IMU has been initialized for a sufficient
     *              duration (typically > 0.1s) and the pose is valid.
     *
     * @param[out] isFinished_out `true` if the system is finished/initialized,
     * `false` otherwise. When `false`, the pose and tracking state should not
     * be relied upon for critical decisions.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus isFinished(bool &isFinished_out);

    /*!
     * @brief       Check if the system has lost the current pose estimate.
     *              The system is considered "lost" when the tracker state
     *              is Tracking::LOST, meaning the camera pose cannot be
     *              reliably estimated from the current frame.
     *
     * @param[out] isLost_out `true` if the system has lost tracking, `false`
     * otherwise. When `true`, the system needs to re-localize or restart.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus isLost(bool &isLost_out);

    /*!
     * @brief       Change the dataset being processed. This allows the system
     *              to switch between different datasets or configuration
     *              profiles without restarting.
     *
     * @note        This function is currently a placeholder. Full dataset
     *              switching support may be added in future versions.
     */
    [[nodiscard]] SystemStatus changeDataset();

    /*!
     * @brief       Get the image scale factor used by the system. This factor
     *              relates the image pixel coordinates to real-world metrics.
     *
     * @param[out] imageScale_out Image scale factor. The mapping from pixels to
     * meters depends on the specific sensor configuration and calibration.
     * @return SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus getImageScale(float &imageScale_out);

    /*!
     * @brief       Marks one keyframe as handed off to the semantic
     *              segmentation pipeline. Called from the publish site
     *              (`common.cc`) at the exact instant the keyframe image is
     *              queued for the segmenter, so the lockstep controller's
     *              `published - returned` backlog signal covers the whole
     *              in-flight window, not just what `System` can see
     *              internally.
     */
    [[nodiscard]] SystemStatus incrementSegmentationPublishedCount();

    /*!
     * @brief       Parse the JSON file containing the environment data
     *
     * @param[in]   jsonFilePath_in
     *              The path to the JSON file
     */
    [[nodiscard]] SystemStatus parseJsonDatabase(string jsonFilePath_in);

    /*!
     * @brief       Add the segmented image to the buffer in the
     *              SemanticSegmentation
     *
     * @param[in]   p_tuple_in
     *              The address of the tuple of segmented image and pointcloud
     */
    [[nodiscard]] SystemStatus addSegmentedImage(
        std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *p_tuple_in);

    /*!
     * @brief       Get the skeleton cluster coming from the current map
     */
    [[nodiscard]] SystemStatus getSkeletonCluster(
        std::vector<std::vector<Eigen::Vector3d>> &skeletonCluster_out);

    /*!
     * @brief       Update the skeleton cluster coming from `voxblox_skeleton`
     *              in the map.
     *
     * @param[in]   skeletonClusterPoints_World_m_in
     *              the skeleton cluster points
     */
    [[nodiscard]] SystemStatus
        setSkeletonCluster(const std::vector<std::vector<Eigen::Vector3d>>
                               &skeletonClusterPoints_World_m_in);

    /*!
     * @brief       Stores the latest connected Voxblox skeleton edges.
     *
     * @param[in]   skeletonEdges_World_m_in
     *              Start and end points of each connected skeleton edge.
     */
    [[nodiscard]] SystemStatus setSkeletonEdges(
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            &skeletonEdges_World_m_in);

    /*!
     * @brief       Update the GNN room candidates list
     */
    [[nodiscard]] SystemStatus setGNNRoomCandidates(
        const std::vector<vs_graphs::core::semantic::Room *>
            &gnnRoomCandidates_in);

#ifdef REGISTER_TIMES
    [[nodiscard]] SystemStatus insertRectTime(double &time_inout);
    [[nodiscard]] SystemStatus insertResizeTime(double &time_inout);
    [[nodiscard]] SystemStatus insertTrackTime(double &time_inout);
#endif

  private:
    /*!
     * @brief       Sensor type. Indicates the camera/IMU configuration used
     *              by the system (MONOCULAR, STEREO, RGBD, IMU_MONOCULAR,
     *              IMU_STEREO, IMU_RGBD). Determines which tracking method
     *              is used (TrackStereo, TrackRGBD, TrackMonocular).
     */
    SensorType sensor;

    /*!
     * @brief       ORB vocabulary used for place recognition and feature
     *              matching.
     */
    ORBVocabulary *p_vocabulary;

    /*!
     * @brief       KeyFrame database for place recognition (relocalization and
     *              loop detection).
     */
    KeyFrameDatabase *p_keyFrameDatabase;

    /*!
     * @brief       Applies the localization-mode and reset requests raised
     *              since the previous frame, before a tracking call feeds the
     *              next frame to the tracker.
     *
     *              Localization-mode activation blocks until Local Mapping has
     *              stopped. A full reset takes precedence over an active-map
     *              reset and clears both requests.
     */
    [[nodiscard]] SystemStatus applyPendingModeAndResetRequests();

    /*!
     * @brief       Reports whether every worker thread (local mapping, loop
     *              closing, semantic segmentation, semantics manager and,
     *              when present, the viewer) has finished.
     *
     * @param[out]  haveFinished_out
     *              True when all of them have finished.
     *
     * @return      SYSTEM_STATUS_SUCCESS.
     */
    [[nodiscard]] SystemStatus haveWorkersFinished(bool &haveFinished_out);

    /*!
     * @brief       Save the current Atlas to a file. The type parameter
     * determines the format (text or binary) and which map data to persist.
     *
     * @param[in]   type_in
     *              Format/type specifier for saving. See @ref FileType
     * "FileType" for valid values (TEXT_FILE, BINARY_FILE).
     *
     * @param[out] isSaved_out `true` if the Atlas was saved successfully,
     * `false`
     * @return SYSTEM_STATUS_SUCCESS.
     * otherwise.
     *
     * @frame       N/A
     * @unit        N/A
     */
    [[nodiscard]] SystemStatus saveAtlas(int type_in, bool &isSaved_out);

    /*!
     * @brief       Load an Atlas from a file. The type parameter determines the
     *              format and which map data to restore.
     *
     * @param[in]   type_in
     *              Format/type specifier for loading. See @ref FileType
     * "FileType" for valid values (TEXT_FILE, BINARY_FILE).
     *
     * @param[out] isLoaded_out `true` if the Atlas was loaded successfully,
     * `false`
     * @return SYSTEM_STATUS_SUCCESS.
     * otherwise.
     *
     * @frame       N/A
     * @unit        N/A
     */
    [[nodiscard]] SystemStatus loadAtlas(int type_in, bool &isLoaded_out);

    /*!
     * @brief       Calculate a checksum for a file to verify data integrity.
     *              Used when loading/saving Atlas data to ensure the file has
     *              not been corrupted.
     *
     * @param[in]   filename_in
     *              Path to the file for which to calculate the checksum.
     *
     * @param[in]   type_in
     *              Format/type specifier affecting the checksum algorithm. See
     *              @ref FileType "FileType" for valid values.
     *
     * @param[out] checkSum_out Checksum string representing the file's content
     * hash.
     * @return SYSTEM_STATUS_SUCCESS.
     *
     * @frame       N/A
     * @unit        N/A
     */
    [[nodiscard]] SystemStatus calculateCheckSum(string  filename_in,
                                                 int     type_in,
                                                 string &checkSum_out);

    /*!
     * @brief       Atlas pointer. Owned by the System class. Provides access to
     *              the global map, keyframes, and map points.
     */
    Atlas *p_atlas;

    /*!
     * @brief       Tracker pointer. Owned by the System class. Receives frames
     * and computes the associated camera pose. Also decides when to insert new
     * keyframes, create MapPoints, and perform relocalization if tracking
     * fails.
     */
    Tracking *p_tracker;

    /*!
     * @brief       Local Mapping pointer. Owned by the System class. Manages
     * the local map and performs local bundle adjustment.
     */
    LocalMapping *p_localMapper;

    /*!
     * @brief       Loop Closing pointer. Owned by the System class. Searches
     * for loops with every new keyframe. If a loop is found, performs pose
     * graph optimization and full bundle adjustment in a separate thread.
     */
    LoopClosing *p_loopCloser;

    /*!
     * @brief       Viewer pointer. Owned by the System class. Draws the map and
     *              the current camera pose using Pangolin. Set to `nullptr`
     * when
     *              @ref bUseViewer "bUseViewer" is `false`.
     */
    Viewer *p_viewer;

    /*!
     * @brief       Frame Drawer pointer. Owned by the System class. Handles the
     *              drawing of frames for visualization.
     */
    FrameDrawer *p_frameDrawer;

    /*!
     * @brief       Map Drawer pointer. Owned by the System class. Handles the
     *              drawing of the map structure.
     */
    MapDrawer *p_mapDrawer;

    /*!
     * @brief       Semantic Segmentation pointer. Owned by the System class.
     *              Processes RGB-D images to produce semantic segmentations.
     */
    SemanticSegmentation *p_semanticSegmentation;

    /*!
     * @brief       Semantics Manager pointer. Owned by the System class.
     * Manages the semantic evaluation pipeline, including room/floor/passage
     *              topology and segmentation result caching.
     */
    SemanticsManager *p_semanticsManager;

    /* ---------------------------------------------------------------------- *
     * SLAM SYSTEM THREADS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Viewer thread. Launched when @ref bUseViewer "bUseViewer" is
     *              `true`. Handles visualization of the map and camera pose.
     *              Set to `nullptr` when viewer is disabled.
     */
    std::thread *p_viewerThread;

    /*!
     * @brief       Loop Closing thread. Processes new keyframes for loop
     *              detection and, if a loop is found, performs pose graph
     *              optimization and full bundle adjustment in a separate
     * thread.
     */
    std::thread *p_loopClosingThread;

    /*!
     * @brief       Local Mapping thread. Performs local bundle adjustment and
     *              manages the local map. Runs concurrently with tracking.
     */
    std::thread *p_localMappingThread;

    /*!
     * @brief       Semantic Segmentation thread. Processes incoming RGB-D
     * frames and produces semantic segmentations. Communicates with the main
     * system through shared atomic counters.
     */
    std::thread *p_semanticSegmentationThread;

    /*!
     * @brief       Semantics Manager thread. Manages the semantic evaluation
     *              pipeline, including room/floor/passage topology updates and
     *              segmentation result caching.
     */
    std::thread *p_semanticsManagerThread;

    /*!
     * @brief       Geometric Segmentation thread. Handles geometric scene
     *              segmentation (e.g., plane, floor, wall detection).
     */
    std::thread *p_geometricSegmentationThread;

    /*!
     * @brief       Reset mutex. Protects the reset operation to ensure thread-
     *              safe shutdown and map clearing.
     */
    std::mutex resetMutex;

    /*!
     * @brief       Reset flag. When `true`, requests a full system reset
     * (clears Atlas and active map). Must be handled carefully across threads.
     */
    bool isResetRequested;

    /*!
     * @brief       Reset active map flag. When `true`, requests a reset of only
     *              the active map while retaining the overall Atlas state.
     */
    bool isResetActiveMapRequested;

    /*!
     * @brief       Mode mutex. Protects mode transitions (e.g. localization
     *              mode activation/deactivation) to ensure thread safety.
     */
    std::mutex modeMutex;

    /*!
     * @brief       Localization mode activation flag. When `true`, the local
     *              mapping thread is paused and only camera tracking runs.
     */
    bool isLocalizationModeActivationRequested;

    /*!
     * @brief       Localization mode deactivation flag. When `true`, resumes
     * the local mapping thread and resumes full SLAM operation.
     */
    bool isLocalizationModeDeactivationRequested;

    /*!
     * @brief       Shutdown flag. When `true`, requests all threads to finish.
     *              After all threads have been joined via @ref Shutdown(), the
     *              system is fully shut down.
     */
    bool isShutdownRequested;

    /*!
     * @brief       True once initialize() has succeeded. The destructor only
     *              stops and joins worker threads that initialize() started.
     */
    bool isInitialized{false};

    /*!
     * @brief       Current tracking state. Updated by the tracker and read by
     *              various services. Values correspond to the tracking state
     *              machine enumeration.
     */
    int trackingState{-1};

    /*!
     * @brief       Number of inliers from the most recent tracking frame.
     *              Used to assess tracking quality.
     */
    int trackingInliers{0};

    /*!
     * @brief       Timestamp of the last processed frame. Used for timing
     *              analysis and frame-to-frame consistency checks.
     */
    double lastFrameTimestamp{0.0};

    /*!
     * @brief       Current camera pose in the world frame. Updated by the
     * tracker after each frame processing. Represents the estimated position
     * and orientation of the camera.
     */
    Sophus::SE3f currentCameraPose_World;

    /*!
     * @brief       Whether the current camera pose is valid. If `false`, the
     * pose should not be relied upon for navigation or planning.
     */
    bool isCurrentCameraPoseValid{false};

    /*!
     * @brief       Reset counter. Atomic counter tracking the number of system
     *              resets. Written from the reset thread and read by the
     * mission health service.
     */
    std::atomic<std::uint64_t> resetCount{0U};

    /*! RGB-D frontend counters sampled by the mission-health service. */
    std::atomic<std::uint64_t> rgbdFrontendAcceptedCount{0U};
    std::atomic<std::uint64_t> rgbdFrontendProcessedCount{0U};
    std::atomic<std::uint64_t> rgbdFrontendOverwrittenCount{0U};
    std::atomic<bool>          isRgbdFrontendWorkerInFlight{false};
    std::atomic<std::int64_t>
        rgbdFrontendLastProcessedSensorTimestampNanoseconds{0};

    /*!
     * @brief       In-flight semantic-segmentation keyframe accounting.
     *              `mSegmentationPublishedCount` increments when a keyframe is
     *              queued for the segmenter; `mSegmentationReturnedCount` and
     *              `mLastReturnedKeyFrameId` increment/advance in
     *              `addSegmentedImage` once the result comes back, on every
     *              code path -- including the GEO-mode early return, which does
     *              not reach the semantic buffer but still ends that keyframe's
     *              time in flight. `std::atomic` rather than a mutex: written
     *              from the publish thread and the segmentation-callback
     * thread, read from the mission health service thread at ~10 Hz, and the
     * two writers need no ordering relative to each other beyond eventual
     * consistency.
     */
    std::atomic<std::uint64_t> segmentationPublishedCount{0U};

    /*!
     * @brief       In-frame semantic-segmentation result counting.
     *              Increments when the semantic segmentation pipeline returns a
     *              result for a keyframe. Combined with
     *              @ref mSegmentationPublishedCount, the difference indicates
     * how many keyframes remain in-flight.
     */
    std::atomic<std::uint64_t> segmentationReturnedCount{0U};

    /*!
     * @brief       Last returned keyframe ID. Advances in
     *              `addSegmentedImage` when a segmentation result is received.
     *              Used together with @ref mSegmentationPublishedCount and
     *              @ref mSegmentationReturnedCount to track in-flight
     * keyframes.
     */
    std::atomic<std::uint64_t> lastReturnedKeyFrameId{0U};

    /*!
     * @brief       Tracked map points. List of MapPoint pointers currently
     * being tracked. Maintained for quick access without traversing the full
     * Atlas.
     */
    std::vector<MapPoint *> trackedMapPoints;

    /*!
     * @brief       Un-tracked keypoints from the current frame. These are
     * keypoints that were detected but not yet associated with MapPoints.
     */
    std::vector<cv::KeyPoint> trackedKeyPointsUn;

    /*!
     * @brief       State mutex. Protects shared state related to tracking and
     *              map management. Ensures exclusive access when modifying
     *              tracking-related data structures.
     */
    std::mutex stateMutex;

    /*!
     * @brief       Map ID of the most recently processed frame, used to detect
     *              map restarts for room-context carryover.
     */
    long unsigned int lastProcessedMapId{0};

    /*!
     * @brief       Initial map initialization flag. When `true`, the first map
     *              build is in progress. Set to `false` once the initial map
     *              has been built and the system is tracking.
     */
    bool isAwaitingFirstMap{true};

    /*!
     * @brief       File path for loading an Atlas from disk. Used to resume
     *              processing from a saved map state.
     */
    string loadAtlasFile;

    /*!
     * @brief       File path for saving the Atlas to disk. Used to persist the
     *              map state for later resumption.
     */
    string saveAtlasFile;

    /*!
     * @brief       File path for the ORB vocabulary. Used by the System
     *              constructor to locate the vocabulary file for place
     *              recognition and feature matching.
     */
    string vocabularyFilePath;

    /*!
     * @brief       Vector of Room pointers from the environment. Maintained for
     *              quick access to room objects without traversing the Atlas.
     */
    std::vector<vs_graphs::core::semantic::Room *> envRooms;

    /*!
     * @brief       Settings object. Contains all configuration parameters for
     * the SLAM system, read from the YAML settings file.
     */
    utils::settings::Settings *p_settings;
};

} // namespace core
} // namespace vs_graphs

#endif // SYSTEM_H
