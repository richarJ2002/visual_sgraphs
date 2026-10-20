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
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            readParameter.cc
 *
 * @brief           Implements all Settings::readParameter() overloads.
 */

#include "configuration/settings/objects/SettingsClass.h"

#include <string>

#include <opencv2/core/persistence.hpp>

#include "execution/system/objects/VerboseClass.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{
/*!
 * @brief           Reads one floating-point parameter: the
 *                  readParameter<float>() specialization whose contract is on
 *                  its declaration in Settings.h.
 */
template <>
SettingsStatus Settings::readParameter<float>(cv::FileStorage   &storage_in,
                                              const std::string &name_in,
                                              bool              &found_out,
                                              float             &parameter_out,
                                              const bool         required_in)
{
    cv::FileNode node = storage_in[name_in];
    if (node.empty())
    {
        if (required_in)
        {
            VSLAM_LOG_ERROR(
                "\t- Required parameter '%s' does not exist! Aborting...\n",
                name_in.c_str());
            exit(-1);
        }
        else
        {
            VSLAM_LOG_WARN("\t- Skipping optional parameter '%s' ...\n",
                           name_in.c_str());
            found_out     = false;
            parameter_out = 0.0f;
            return SettingsStatus::SETTINGS_STATUS_SUCCESS;
        }
    }
    else if (!node.isReal())
    {
        VSLAM_LOG_ERROR(
            "\t- Parameter '%s' is not a real number! Aborting...\n",
            name_in.c_str());
        exit(-1);
    }
    else
    {
        found_out     = true;
        parameter_out = node.real();
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
}

/*!
 * @brief           Reads one integer parameter: the readParameter<int>()
 *                  specialization whose contract is on its declaration in
 *                  Settings.h.
 */
template <>
SettingsStatus Settings::readParameter<int>(cv::FileStorage   &storage_in,
                                            const std::string &name_in,
                                            bool              &found_out,
                                            int               &parameter_out,
                                            const bool         required_in)
{
    cv::FileNode node = storage_in[name_in];
    if (node.empty())
    {
        if (required_in)
        {
            VSLAM_LOG_ERROR(
                "\t- Required parameter '%s' does not exist! Aborting...\n",
                name_in.c_str());
            exit(-1);
        }
        else
        {
            VSLAM_LOG_WARN("\t- Skipping optional parameter '%s' ...\n",
                           name_in.c_str());
            found_out     = false;
            parameter_out = 0;
            return SettingsStatus::SETTINGS_STATUS_SUCCESS;
        }
    }
    else if (!node.isInt())
    {
        VSLAM_LOG_ERROR("\t- Parameter '%s' is not an integer! Aborting...\n",
                        name_in.c_str());
        exit(-1);
    }
    else
    {
        found_out     = true;
        parameter_out = node.operator int();
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
}

template <>
SettingsStatus Settings::readParameter<std::string>(cv::FileStorage &storage_in,
                                                    const std::string &name_in,
                                                    bool        &found_out,
                                                    std::string &parameter_out,
                                                    const bool   required_in)
{
    cv::FileNode node = storage_in[name_in];
    if (node.empty())
    {
        if (required_in)
        {
            VSLAM_LOG_ERROR(
                "\t- Required parameter '%s' does not exist! Aborting...\n",
                name_in.c_str());
            exit(-1);
        }
        else
        {
            VSLAM_LOG_WARN("\t- Skipping optional parameter '%s' ...\n",
                           name_in.c_str());
            found_out     = false;
            parameter_out = std::string();
            return SettingsStatus::SETTINGS_STATUS_SUCCESS;
        }
    }
    else if (!node.isString())
    {
        VSLAM_LOG_ERROR("\t- Parameter '%s' is not a string! Aborting...\n",
                        name_in.c_str());
        exit(-1);
    }
    else
    {
        found_out     = true;
        parameter_out = node.string();
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
}

template <>
SettingsStatus Settings::readParameter<cv::Mat>(cv::FileStorage   &storage_in,
                                                const std::string &name_in,
                                                bool              &found_out,
                                                cv::Mat   &parameter_out,
                                                const bool required_in)
{
    cv::FileNode node = storage_in[name_in];
    if (node.empty())
    {
        if (required_in)
        {
            VSLAM_LOG_ERROR(
                "\t- Required parameter '%s' does not exist! Aborting...\n",
                name_in.c_str());
            exit(-1);
        }
        else
        {
            VSLAM_LOG_WARN("\t- Skipping optional parameter '%s' ...\n",
                           name_in.c_str());
            found_out     = false;
            parameter_out = cv::Mat();
            return SettingsStatus::SETTINGS_STATUS_SUCCESS;
        }
    }
    else
    {
        found_out     = true;
        parameter_out = node.mat();
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
}
} /* namespace settings */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
