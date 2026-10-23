/*!
 * @file            writeReport.cc
 *
 * @brief           Tests TimerRegistry::writeReport: the CSV header, one row
 *                  per (file, function) with calls, total, mean and maximum,
 *                  files relative to the package root, and no file for an
 *                  empty test-run folder.
 */

/* Matching Declaration Include */
#include "execution/profiling/objects/TimerRegistryClass.h"

/* C++ Standard Library Includes */
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

/* External Library Includes */
#include <gtest/gtest.h>

TEST(TimerRegistryWriteReport, WritesOneRowPerFunction)
{
    const std::string sourceFile =
        std::string(VS_GRAPHS_SOURCE_ROOT) + "/core/src/example.cc";
    vs_graphs::core::profiling::TimerRegistry &timerRegistry =
        vs_graphs::core::profiling::TimerRegistry::getInstance();
    timerRegistry.recordCall(sourceFile.c_str(), "exampleFunction", 1.0);
    timerRegistry.recordCall(sourceFile.c_str(), "exampleFunction", 2.0);

    const std::filesystem::path testRunDirectory =
        std::filesystem::temp_directory_path() / "vs_graphs_timer_report_test";
    std::filesystem::remove_all(testRunDirectory);
    timerRegistry.writeReport(testRunDirectory.string());

    std::ifstream reportFile(testRunDirectory / "archive" / "profiling" /
                             "timers" / "vs_graphs.csv");
    ASSERT_TRUE(reportFile.good());
    std::stringstream reportText;
    reportText << reportFile.rdbuf();
    EXPECT_EQ(reportText.str(),
              "function,file,calls,total_ms,mean_ms,max_ms\n"
              "exampleFunction,core/src/example.cc,2,3.000,1.500,2.000\n");
    std::filesystem::remove_all(testRunDirectory);
}

TEST(TimerRegistryWriteReport, WritesNothingForAnEmptyFolder)
{
    const std::filesystem::path workingDirectory =
        std::filesystem::current_path();
    vs_graphs::core::profiling::TimerRegistry::getInstance().writeReport("");
    EXPECT_FALSE(std::filesystem::exists(workingDirectory / "archive"));
}
