/*!
 * @file         parseViewerParamFile.cc
 *
 * @brief        Implements MapDrawer::parseViewerParamFile declared in
 *               MapDrawer.h.
 */

#include "MapDrawer.h"

#include <cerrno>

namespace vs_graphs
{
namespace core
{

bool MapDrawer::parseViewerParamFile(cv::FileStorage &settings_in)
{
    bool isParameterMissing = false;

    cv::FileNode node = settings_in["Viewer.KeyFrameSize"];
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

    node = settings_in["Viewer.KeyFrameLineWidth"];
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

    node = settings_in["Viewer.GraphLineWidth"];
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

    node = settings_in["Viewer.PointSize"];
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

    node = settings_in["Viewer.CameraSize"];
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

    node = settings_in["Viewer.CameraLineWidth"];
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

    return !isParameterMissing;
}

} // namespace core
} // namespace vs_graphs
