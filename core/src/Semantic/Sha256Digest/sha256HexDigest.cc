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

#include "Semantic/Sha256Digest.h"

#include <array>
#include <cstddef>
#include <cstdio>

#include <openssl/evp.h>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::string sha256HexDigest(const std::string &bytes_in)
{
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned int                               digestLength = 0U;

    /* One-shot EVP digest, OpenSSL 3.x's non-deprecated SHA-256 path
     * (SHA256_Init/Update/Final from <openssl/sha.h> are legacy low-level
     * calls OpenSSL 3.0 discourages in favour of EVP_Digest()). */
    EVP_Digest(bytes_in.data(),
               bytes_in.size(),
               digest.data(),
               &digestLength,
               EVP_sha256(),
               nullptr);

    std::string hex;
    hex.reserve(static_cast<std::size_t>(digestLength) * 2U);
    static const char kHexChars[] = "0123456789abcdef";
    for (unsigned int i = 0U; i < digestLength; ++i)
    {
        hex.push_back(kHexChars[(digest[i] >> 4U) & 0x0FU]);
        hex.push_back(kHexChars[digest[i] & 0x0FU]);
    }
    return hex;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
