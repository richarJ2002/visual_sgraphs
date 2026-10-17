/*!
 * @file            parseViewerParamFile.cc
 *
 * @brief           Implements the live MapDrawer settings-file parser.
 */

#include "map_drawer/objects/MapDrawerClass.h"

#include <iostream>

#include <opencv2/core/persistence.hpp>

namespace vs_graphs::core::map_drawer
{

MapDrawerStatus
    MapDrawer::parseViewerParamFile(cv::FileStorage &settingsFileStorage_inout,
                                    bool            &isParsed_out)
{
    /* Preserve partial updates and historical presence-only validation. */
    bool isParameterMissing = false;

    cv::FileNode node = settingsFileStorage_inout["Viewer.KeyFrameSize"];
    if (!node.empty())
    {
        keyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settingsFileStorage_inout["Viewer.KeyFrameLineWidth"];
    if (!node.empty())
    {
        keyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr
            << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not "
               "a real number*"
            << std::endl;
        isParameterMissing = true;
    }

    node = settingsFileStorage_inout["Viewer.GraphLineWidth"];
    if (!node.empty())
    {
        graphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not "
                     "a real number*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settingsFileStorage_inout["Viewer.PointSize"];
    if (!node.empty())
    {
        pointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settingsFileStorage_inout["Viewer.CameraSize"];
    if (!node.empty())
    {
        cameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settingsFileStorage_inout["Viewer.CameraLineWidth"];
    if (!node.empty())
    {
        cameraLineWidth = node.real();
    }
    else
    {
        std::cerr
            << "*Viewer.CameraLineWidth parameter doesn't exist or is not "
               "a real number*"
            << std::endl;
        isParameterMissing = true;
    }

    isParsed_out = !isParameterMissing;
    return MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS;
}

} /* namespace vs_graphs::core::map_drawer */
