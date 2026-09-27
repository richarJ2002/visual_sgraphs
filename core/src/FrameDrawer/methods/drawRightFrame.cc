/*!
 * @file         drawRightFrame.cc
 *
 * @brief        Implements FrameDrawer::drawRightFrame declared in
 *               FrameDrawer.h.
 */

#include "FrameDrawer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace vs_graphs
{
namespace core
{

cv::Mat FrameDrawer::drawRightFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint>
        vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int>
        vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool>         vbVO, vbMap;  // Tracked MapPoints in current frame
    int                  drawState;    // Tracking state

    // Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        drawState = state;
        if (state == Tracking::SYSTEM_NOT_READY)
            state = Tracking::NO_IMAGES_YET;

        imageRight.copyTo(im);

        if (state == Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = currentKeysRight;
            vIniKeys     = iniKeys;
            vMatches     = iniMatches;
        }
        else if (state == Tracking::OK)
        {
            vCurrentKeys = currentKeysRight;
            vbVO         = mvbVO;
            vbMap        = mvbMap;
        }
        else if (state == Tracking::LOST)
        {
            vCurrentKeys = currentKeysRight;
        }
    } // destroy scoped mutex -> release mutex

    if (imageScale != 1.f)
    {
        int imWidth  = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if (im.channels() < 3) // this should be always true
        cvtColor(im, im, cv::COLOR_GRAY2BGR);

    // Draw
    if (drawState == Tracking::NOT_INITIALIZED) // INITIALIZING
    {
        for (unsigned int i = 0; i < vMatches.size(); i++)
        {
            if (vMatches[i] >= 0)
            {
                cv::Point2f pt1, pt2;
                if (imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }

                cv::line(im, pt1, pt2, cv::Scalar(0, 255, 0));
            }
        }
    }
    else if (drawState == Tracking::OK) // TRACKING
    {
        trackedCount      = 0;
        trackedVOCount    = 0;
        const float r     = 5;
        const int   n     = currentKeysRight.size();
        const int   Nleft = currentKeys.size();

        for (int i = 0; i < n; i++)
        {
            if (vbVO[i + Nleft] || vbMap[i + Nleft])
            {
                cv::Point2f pt1, pt2;
                cv::Point2f point;
                if (imageScale != 1.f)
                {
                    point    = currentKeysRight[i].pt / imageScale;
                    float px = currentKeysRight[i].pt.x / imageScale;
                    float py = currentKeysRight[i].pt.y / imageScale;
                    pt1.x    = px - r;
                    pt1.y    = py - r;
                    pt2.x    = px + r;
                    pt2.y    = py + r;
                }
                else
                {
                    point = currentKeysRight[i].pt;
                    pt1.x = currentKeysRight[i].pt.x - r;
                    pt1.y = currentKeysRight[i].pt.y - r;
                    pt2.x = currentKeysRight[i].pt.x + r;
                    pt2.y = currentKeysRight[i].pt.y + r;
                }

                // This is a match to a MapPoint in the map
                if (vbMap[i + Nleft])
                {
                    cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
                    cv::circle(im, point, 2, cv::Scalar(0, 255, 0), -1);
                    trackedCount++;
                }
                else // This is match to a "visual odometry" MapPoint created in
                     // the last frame
                {
                    cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
                    cv::circle(im, point, 2, cv::Scalar(255, 0, 0), -1);
                    trackedVOCount++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    drawTextInfo(im, drawState, imWithInfo);

    return imWithInfo;
}

} // namespace core
} // namespace vs_graphs
