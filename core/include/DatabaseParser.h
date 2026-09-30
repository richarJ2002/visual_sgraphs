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

#ifndef DBPARSER_H
#define DBPARSER_H

/*!
 * @file         DatabaseParser.h
 *
 * @brief        Declares the ground-truth environment JSON parser.
 */

#include "DBParserStatus.h"
#include "Thirdparty/nlohmann/json.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Semantic/Room.h"

using Json = nlohmann::json;

namespace vs_graphs
{
namespace core
{
/*!
 * @brief        Parses ground-truth environment data from JSON files.
 */
class DBParser
{
  private:
    /*!
     * @brief        Rooms created from the last JSON input and
     *               retained here. The parser allocates each room;
     *               pointers handed out are non-owning views.
     */
    std::vector<semantic::Room *> environmentRooms;

  public:
    /*!
     * @brief        Loads and parses the JSON file at the given path.
     *
     *               Terminates the process when the file cannot be
     *               parsed.
     *
     * @param[in]    jsonFilePath_in
     *               Path of the JSON file to read.
     *
     * @param[out] json_out Parsed JSON document.
     * @return DBPARSER_STATUS_SUCCESS.
     */
    [[nodiscard]] DBParserStatus parseJsonFile(std::string jsonFilePath_in,
                                               Json       &json_out);

    /*!
     * @brief        Builds the environment rooms described by parsed
     *               JSON data.
     *
     *               Replaces any previously retained rooms with one
     *               room per entry of the "rooms" object.
     *
     * @param[in]    environmentData_in
     *               Parsed JSON document holding the rooms data.
     *
     * @param[out] environmentRooms_out Non-owning views of the parser-retained
     * rooms.
     * @return DBPARSER_STATUS_SUCCESS.
     */
    [[nodiscard]] DBParserStatus getEnvironmentRooms(
        Json                           environmentData_in,
        std::vector<semantic::Room *> &environmentRooms_out);
};
} // namespace core
} // namespace vs_graphs
#endif