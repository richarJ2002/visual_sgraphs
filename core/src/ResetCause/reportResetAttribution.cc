/*!
 * @file         reportResetAttribution.cc
 *
 * @brief        Implements reportResetAttribution declared in ResetCause.h.
 */

#include "ResetCause.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

ResetCauseStatus reportResetAttribution(const ResetCause  cause_in,
                                        const ResetAction action_in)
{
    std::string resetAttribution{};
    if (formatResetAttribution(cause_in, action_in, resetAttribution) !=
        ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: formatResetAttribution returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << resetAttribution << std::endl;

    return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
