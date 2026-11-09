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
 * @file            eraseEntriesByPointer.tpp
 *
 * @brief           Implements eraseEntriesByPointer(), declared in
 *                  semantics_manager/graph/private_functions/private_functions.h.
 */

#ifndef VS_GRAPHS_SCENE_GRAPH_ERASE_ENTRIES_BY_POINTER_TPP
#define VS_GRAPHS_SCENE_GRAPH_ERASE_ENTRIES_BY_POINTER_TPP

#include <iterator>
#include <map>

namespace vs_graphs
{
namespace core
{

template <class EntryType>
void eraseEntriesByPointer(std::map<int, EntryType *> &entriesById_inout,
                           const EntryType            *p_entry_in)
{
    for (typename std::map<int, EntryType *>::iterator entryIterator =
             entriesById_inout.begin();
         entryIterator != entriesById_inout.end();)
    {
        entryIterator = entryIterator->second == p_entry_in
                            ? entriesById_inout.erase(entryIterator)
                            : std::next(entryIterator);
    }
}

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SCENE_GRAPH_ERASE_ENTRIES_BY_POINTER_TPP */
