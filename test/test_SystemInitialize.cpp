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
 * @file            test_GeometricToolsStatus.cpp
 *
 * @brief           Statuses of GeometricTools::triangulate: a point at a
 *                  finite depth succeeds; a solution at infinity (zero
 *                  homogeneous scale) is a numerical failure that leaves the
 *                  output unchanged.
 */

/*!
 * @file            test_SystemInitialize.cpp
 *
 * @brief           Checks that System::initialize() reports an unreadable
 *                  settings or vocabulary file as a status (it used to end the
 *                  process with exit(-1)) and that a system that never
 *                  initialised can be destroyed safely.
 */

#include "System.h"

#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

namespace vs_graphs
{
namespace core
{
namespace
{

/*! Settings file that opens but holds no camera, so the run stops at the
 *  vocabulary step. Removed when the test ends. */
class ReadableSettingsFile
{
  public:
    ReadableSettingsFile() :
        path(testing::TempDir() + "system_initialize_settings.yaml")
    {
        std::ofstream file(path);
        file << "%YAML:1.0\n---\nunused: 1\n";
    }
    ~ReadableSettingsFile()
    {
        std::remove(path.c_str());
    }
    const std::string path;
};

} // namespace

TEST(SystemInitialize, ReportsAnUnreadableSettingsFile)
{
    System system;
    EXPECT_EQ(system.initialize("/nonexistent/vocabulary.bin",
                                "/nonexistent/settings.yaml",
                                "/nonexistent/system_params.yaml",
                                System::RGBD,
                                false),
              SystemStatus::SYSTEM_STATUS_SETTINGS_UNREADABLE);
}

TEST(SystemInitialize, ReportsAnUnreadableVocabularyFile)
{
    const ReadableSettingsFile settings;
    System                     system;
    EXPECT_EQ(system.initialize("/nonexistent/vocabulary.bin",
                                settings.path,
                                "/nonexistent/system_params.yaml",
                                System::RGBD,
                                false),
              SystemStatus::SYSTEM_STATUS_VOCABULARY_UNREADABLE);
}

TEST(SystemInitialize, DestroysASystemThatNeverInitialised)
{
    /* No thread was started, so the destructor must not wait for or join
     * any worker. Reaching the end of the scope is the check. */
    {
        System system;
    }
    SUCCEED();
}

} // namespace core
} // namespace vs_graphs
