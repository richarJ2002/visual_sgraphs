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
 * @file            nlohmann.h
 *
 * @brief           Exposes the JSON module during its staged migration.
 *
 * @note            This move stage preserves the original implementation and
 *                  API. Full type/method splitting and renaming remain pending;
 *                  this aggregate is not a permanent upstream wrapper.
 *                  The retained implementation's MIT terms are in LICENSE.MIT.
 */

#ifndef VS_GRAPHS_CORE_SRC_NLOHMANN_NLOHMANN_H
#define VS_GRAPHS_CORE_SRC_NLOHMANN_NLOHMANN_H

#include "nlohmann/objects/json.hpp"

#if NLOHMANN_JSON_VERSION_MAJOR != 3 || NLOHMANN_JSON_VERSION_MINOR != 11 ||   \
    NLOHMANN_JSON_VERSION_PATCH != 2
#error "The JSON migration must retain the pinned 3.11.2 implementation."
#endif

#endif /* VS_GRAPHS_CORE_SRC_NLOHMANN_NLOHMANN_H */
