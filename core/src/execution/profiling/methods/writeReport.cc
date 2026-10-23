/*!
 * @file            writeReport.cc
 *
 * @brief           Implements TimerRegistry::writeReport() declared in
 *                  TimerRegistryClass.h.
 */

#ifdef VS_GRAPHS_BUILT_TIMERS

#include "execution/profiling/objects/TimerRegistryClass.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <system_error>

#include <rclcpp/logging.hpp>

namespace vs_graphs::core::profiling
{

void TimerRegistry::writeReport(const std::string &testRunDirectory_in)
{
    if (testRunDirectory_in.empty())
    {
        return;
    }

    const std::filesystem::path timerDirectory =
        std::filesystem::path(testRunDirectory_in) / "archive" / "profiling" /
        "timers";
    std::error_code directoryError;
    std::filesystem::create_directories(timerDirectory, directoryError);
    const std::filesystem::path reportPath = timerDirectory / "vs_graphs.csv";
    std::ofstream               reportFile(reportPath);
    if (directoryError || !reportFile)
    {
        RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                    "Cannot write the timer report %s.",
                    reportPath.c_str());
        return;
    }

    /* VS_GRAPHS_SOURCE_ROOT is the package source folder, set by the build
     * for this file only, so the report names files inside the package. */
    const std::filesystem::path packageSourceRoot(VS_GRAPHS_SOURCE_ROOT);

    reportFile << "function,file,calls,total_ms,mean_ms,max_ms\n"
               << std::fixed << std::setprecision(3);
    const std::lock_guard<std::mutex> registryLock(registryMutex);
    for (const auto &[functionKey, functionStatistics] : timerStatistics)
    {
        reportFile << functionKey.second << ','
                   << std::filesystem::path(functionKey.first)
                          .lexically_relative(packageSourceRoot)
                          .string()
                   << ',' << functionStatistics.callCount << ','
                   << functionStatistics.totalDuration_ms << ','
                   << functionStatistics.totalDuration_ms /
                          static_cast<double>(functionStatistics.callCount)
                   << ',' << functionStatistics.maximumDuration_ms << '\n';
    }
}

} /* namespace vs_graphs::core::profiling */

#endif /* VS_GRAPHS_BUILT_TIMERS */
