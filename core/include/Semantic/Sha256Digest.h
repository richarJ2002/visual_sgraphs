/**
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
 * @file            Sha256Digest.h
 *
 * @brief           Declares sha256HexDigest(), a genuine OpenSSL SHA-256
 *                   digest over canonical serialized bytes.
 */

#ifndef SEMANTIC_SHA256_DIGEST_H
#define SEMANTIC_SHA256_DIGEST_H

#include <string>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*! @brief Returns the 64-character lowercase hex SHA-256 digest of \p
 *  bytes_in, computed with OpenSSL's EVP digest API (already linked into
 *  this target via -lcrypto for System.cc's MD5 usage). A genuine SHA-256
 *  digest, not a placeholder: callers must never label a different or
 *  weaker hash "SHA-256". Deterministic and free of wall-clock data,
 *  pointer addresses, or unordered iteration -- suitable as a canonical
 *  content digest for two byte-identical inputs to always match. */
std::string sha256HexDigest(const std::string &bytes_in);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_SHA256_DIGEST_H
