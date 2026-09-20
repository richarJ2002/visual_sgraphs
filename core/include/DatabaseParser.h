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

#include "Thirdparty/nlohmann/json.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Semantic/Room.h"

using json = nlohmann::json;

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
     * @brief        Creates an empty parser.
     */
    DBParser();
    /*!
     * @brief        Destroys the parser.
     */
    ~DBParser();

    /*!
     * @brief        Loads and parses the JSON file at the given path.
     *
     *               Terminates the process when the file cannot be
     *               parsed.
     *
     * @param[in]    jsonFilePath_in
     *               Path of the JSON file to read.
     *
     * @return       Parsed JSON document.
     */
    json parseJsonFile(std::string jsonFilePath_in);

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
     * @return       Non-owning views of the parser-retained rooms.
     */
    std::vector<semantic::Room *> getEnvironmentRooms(json environmentData_in);
};
} // namespace core
} // namespace vs_graphs
#endif