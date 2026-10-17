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
 * @file            VerboseClass.h
 *
 * @brief           Declares Verbose, the core logging severity control.
 */

#ifndef VS_GRAPHS_CORE_SRC_SYSTEM_OBJECTS_VERBOSE_CLASS_H
#define VS_GRAPHS_CORE_SRC_SYSTEM_OBJECTS_VERBOSE_CLASS_H

#include "system/objects/VerboseStatusEnum.h"

#include <rclcpp/logging.hpp>

#include <string>

namespace vs_graphs
{
namespace core
{
namespace system
{

/*!
 * @brief           Logging severity used by the ORB-SLAM3 core. All core output
 *                  is routed through ROS 2's logging framework on the global
 *                  "visual_sgraphs" logger so it shares one severity control
 *                  with the ROS-facing layer.
 */
class Verbose
{
  public:
    /*!
     * @brief           Severity levels for core messages, from quietest to most
     *                  detailed.
     */
    enum VerbosityLevel
    {
        /*!
         * @brief           Quiet mode: only warning-level messages are emitted.
         *                  Default level that preserves quiet behaviour when no
         *                  verbosity is explicitly set by the caller.
         */
        VERBOSITY_QUIET = 0,

        /*!
         * @brief           Normal mode: informational messages are emitted at
         *                  RCLCPP_INFO severity.
         */
        VERBOSITY_NORMAL = 1,

        /*!
         * @brief           Verbose mode: detailed debug messages are emitted at
         *                  RCLCPP_DEBUG severity.
         */
        VERBOSITY_VERBOSE = 2,

        /*!
         * @brief           Very verbose mode: additional debug detail beyond
         *                  Verbose, also emitted at RCLCPP_DEBUG severity.
         */
        VERBOSITY_VERY_VERBOSE = 3,

        /*!
         * @brief           Debug mode: maximum debug detail emitted at
         *                  RCLCPP_DEBUG severity. Use for development
         *                  diagnostics.
         */
        VERBOSITY_DEBUG = 4
    };

    /*!
     * @brief           Highest level that is still emitted; messages with a
     *                  higher level are dropped. Shared by all callers.
     */
    static VerbosityLevel th;

  public:
    /*!
     * @brief           Emit a core diagnostic through the global
     *                  "visual_sgraphs" logger with a severity derived from the
     *                  requested level. The static gate `th` still filters by
     *                  level: only messages with `lev <= th` are emitted.
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
     * @brief           Set the global verbose threshold for core logging routed
     *                  through ROS 2's "visual_sgraphs" logger. Only messages
     *                  with a level less than or equal to this threshold will
     *                  be emitted.
     *
     * @param[in]       threshold_in
     *                               The verbose level threshold. See @ref
     *                  Verbose::VerbosityLevel "VERBOSITY_QUIET",
     *                  VERBOSITY_NORMAL, VERBOSITY_VERBOSE,
     *                  VERBOSITY_VERY_VERBOSE, or VERBOSITY_DEBUG.
     */
    [[nodiscard]] static VerboseStatus setTh(VerbosityLevel threshold_in)
    {
        th = threshold_in;

        return VerboseStatus::VERBOSE_STATUS_SUCCESS;
    }

    /*!
     * @brief           Map a user-supplied log level string ("quiet", "error",
     *                               "warn", "info", "debug") to the matching
     *                               `VerbosityLevel`
     *                  threshold. Unknown strings default to `VERBOSITY_QUIET`
     *                  so an unset or
     *                               invalid value preserves the current quiet
     *                               behaviour.
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
 * @brief           Emit an INFO line from the ORB-SLAM3 core through the global
 *                  "visual_sgraphs" logger as a printf-style message.
 */
#define VSLAM_LOG_INFO(...)                                                    \
    RCLCPP_INFO(rclcpp::get_logger("visual_sgraphs"), __VA_ARGS__)

/*!
 * @brief           Emit a WARN line from the ORB-SLAM3 core through the global
 *                  "visual_sgraphs" logger as a printf-style message.
 */
#define VSLAM_LOG_WARN(...)                                                    \
    RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"), __VA_ARGS__)

/*!
 * @brief           Emit an ERROR line from the ORB-SLAM3 core through the
 *                  global "visual_sgraphs" logger as a printf-style message.
 */
#define VSLAM_LOG_ERROR(...)                                                   \
    RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"), __VA_ARGS__)

/*!
 * @brief           Emit a DEBUG line from the ORB-SLAM3 core through the global
 *                  "visual_sgraphs" logger as a printf-style message.
 */
#define VSLAM_LOG_DEBUG(...)                                                   \
    RCLCPP_DEBUG(rclcpp::get_logger("visual_sgraphs"), __VA_ARGS__)

} /* namespace system */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SYSTEM_OBJECTS_VERBOSE_CLASS_H */
