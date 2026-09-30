/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "System.h"

#include <iomanip>
#include <memory>
#include <openssl/evp.h>

namespace vs_graphs
{
namespace core
{

SystemStatus System::calculateCheckSum(std::string  filename_in,
                                       int          type_in,
                                       std::string &checkSum_out)
{
    std::string checksum = "";

    std::ios_base::openmode flags = std::ios::in;
    if (type_in == BINARY_FILE) // Binary file
        flags = std::ios::in | std::ios::binary;

    std::ifstream f(filename_in.c_str(), flags);
    if (!f.is_open())
    {
        std::cout << "[E] Unable to open the in file " << filename_in
                  << " for Md5 hash." << std::endl;
        checkSum_out = checksum;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }

    /*
     * OpenSSL 3 deprecates the MD5_* calls, so the identical MD5 digest is
     * taken through the EVP interface. The context is owned for the whole
     * scope so that every early return releases it.
     */
    const std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>
        p_digestContext(EVP_MD_CTX_new(), &EVP_MD_CTX_free);

    if (!p_digestContext)
    {
        std::cout << "[E] Unable to allocate the Md5 context for "
                  << filename_in << "." << std::endl;
        checkSum_out = checksum;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }

    if (EVP_DigestInit_ex(p_digestContext.get(), EVP_md5(), nullptr) != 1)
    {
        std::cout << "[E] Unable to start the Md5 hash of " << filename_in
                  << "." << std::endl;
        checkSum_out = checksum;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }

    char buffer[1024];

    while (int count = f.readsome(buffer, sizeof(buffer)))
    {
        if (EVP_DigestUpdate(p_digestContext.get(),
                             buffer,
                             static_cast<std::size_t>(count)) != 1)
        {
            std::cout << "[E] Unable to hash the contents of " << filename_in
                      << "." << std::endl;
            checkSum_out = checksum;
            return SystemStatus::SYSTEM_STATUS_SUCCESS;
        }
    }

    f.close();

    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int  digestLengthBytes = 0U;

    if (EVP_DigestFinal_ex(p_digestContext.get(), digest, &digestLengthBytes) !=
        1)
    {
        std::cout << "[E] Unable to finish the Md5 hash of " << filename_in
                  << "." << std::endl;
        checkSum_out = checksum;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }

    for (unsigned int byteIndex = 0; byteIndex < digestLengthBytes; byteIndex++)
    {
        char aux[10];
        sprintf(aux, "%02x", digest[byteIndex]);
        checksum = checksum + aux;
    }

    checkSum_out = checksum;
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
