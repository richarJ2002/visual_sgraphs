/*!
 * @file         drawTextInfo.cc
 *
 * @brief        Implements FrameDrawer::drawTextInfo declared in FrameDrawer.h.
 */

#include "FrameDrawer.h"

#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace vs_graphs
{
namespace core
{

void FrameDrawer::drawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if (nState == Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if (nState == Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if (nState == Tracking::OK)
    {
        if (!onlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nMaps = p_atlas->countMaps();
        int nKFs  = p_atlas->getKeyFrameCount();
        int nMPs  = p_atlas->getMapPointCount();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs
          << ", Matches: " << trackedCount;
        if (trackedVOCount > 0)
            s << ", + VO matches: " << trackedVOCount;
    }
    else if (nState == Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if (nState == Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int      baseline = 0;
    cv::Size textSize =
        cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

    imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
    im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
    imText.rowRange(im.rows, imText.rows) =
        cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
    cv::putText(imText,
                s.str(),
                cv::Point(5, imText.rows - 5),
                cv::FONT_HERSHEY_PLAIN,
                1,
                cv::Scalar(255, 255, 255),
                1,
                8);
}

} // namespace core
} // namespace vs_graphs
