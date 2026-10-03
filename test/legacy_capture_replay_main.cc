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
 * @file            legacy_capture_replay_main.cc
 *
 * @brief           Command-line entry point for
 *                  vs_graphs::core::semantic::replayLegacyCaptures() (semantic-
 *                  axiom-reliability-plan.md P1.9): usage
 *                  `legacy_capture_replay <corpus_dir> <output_report.json>`.
 *                  Writes the bounded LegacyReplayResult::report to \p
 *                  output_report.json and exits nonzero only on a usage or
 *                  filesystem-write error -- malformed input files are a
 *                  normal, reported outcome, not a tool failure.
 */

#include <fstream>
#include <iostream>

#include "test/LegacyCaptureReplay.h"

/*!
 * @brief           Replays every legacy capture file in a corpus directory and
 *                  writes the bounded JSON report.
 *
 * @param[in]       argc
 *                  Number of command-line arguments; exactly 3 (program, corpus
 *                  directory, report path) is accepted.
 *
 * @param[in]       argv
 *                  Command-line arguments: argv[1] is the corpus directory,
 *                  argv[2] the report path to write.
 *
 * @return          0 after the report is written; 1 on a wrong argument count
 *                  or when the report file cannot be opened. Malformed capture
 *                  files do not change the exit code.
 */
int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "usage: " << argv[0]
                  << " <corpus_dir> <output_report.json>" << std::endl;
        return 1;
    }

    const std::filesystem::path corpusDir(argv[1]);
    const std::filesystem::path outputPath(argv[2]);

    const vs_graphs::core::semantic::LegacyReplayResult result =
        vs_graphs::core::semantic::replayLegacyCaptures(corpusDir);

    std::ofstream outputFile(outputPath);
    if (!outputFile.is_open())
    {
        std::cerr << "could not open output file: " << outputPath << std::endl;
        return 1;
    }
    outputFile << result.report.dump(2);

    std::cout << "totalFiles=" << result.totalFiles
              << " okFiles=" << result.okFiles
              << " malformedFiles=" << result.malformedFiles
              << " report=" << outputPath << std::endl;

    return 0;
}
