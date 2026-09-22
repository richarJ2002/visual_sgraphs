/*!
 * @file         drawFrame.cc
 *
 * @brief        Implements FrameDrawer::drawFrame declared in FrameDrawer.h.
 */

#include "FrameDrawer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

namespace vs_graphs
{
namespace core
{

cv::Mat FrameDrawer::drawFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint>
        vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int>
        vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool>         vbVO, vbMap;  // Tracked MapPoints in current frame
    vector<pair<cv::Point2f, cv::Point2f>> vTracks;
    int                                    drawState; // Tracking state
    vector<float>                          vCurrentDepth;
    float                                  thDepth;

    Frame                               drawnFrame;
    vector<MapPoint *>                  vpLocalMap;
    vector<cv::KeyPoint>                vMatchesKeys;
    vector<MapPoint *>                  vpMatchedMPs;
    vector<cv::KeyPoint>                vOutlierKeys;
    vector<MapPoint *>                  vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    cv::Scalar standardColor(0, 255, 0);
    cv::Scalar odometryColor(255, 0, 0);

    // Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        drawState = state;
        if (state == Tracking::SYSTEM_NOT_READY)
            state = Tracking::NO_IMAGES_YET;

        image.copyTo(im);

        if (state == Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = currentKeys;
            vIniKeys     = iniKeys;
            vMatches     = iniMatches;
            vTracks      = tracks;
        }
        else if (state == Tracking::OK)
        {
            vCurrentKeys = currentKeys;
            vbVO         = mvbVO;
            vbMap        = mvbMap;

            drawnFrame      = currentFrame;
            vpLocalMap      = localMap;
            vMatchesKeys    = matchedKeys;
            vpMatchedMPs    = matchedMPs;
            vOutlierKeys    = outlierKeys;
            vpOutlierMPs    = outlierMPs;
            mProjectPoints  = projectPoints;
            mMatchedInImage = matchedInImage;

            vCurrentDepth = currentDepths;
            thDepth       = depthThreshold;
        }
        else if (state == Tracking::LOST)
        {
            vCurrentKeys = currentKeys;
        }
    }

    if (imageScale != 1.f)
    {
        int imWidth  = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if (im.channels() < 3) // this should be always true
        cvtColor(im, im, cv::COLOR_GRAY2BGR);

    // Draw
    if (drawState == Tracking::NOT_INITIALIZED)
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
                cv::line(im, pt1, pt2, standardColor);
            }
        }
        for (vector<pair<cv::Point2f, cv::Point2f>>::iterator it =
                 vTracks.begin();
             it != vTracks.end();
             it++)
        {
            cv::Point2f pt1, pt2;
            if (imageScale != 1.f)
            {
                pt1 = (*it).first / imageScale;
                pt2 = (*it).second / imageScale;
            }
            else
            {
                pt1 = (*it).first;
                pt2 = (*it).second;
            }
            cv::line(im, pt1, pt2, standardColor, 5);
        }
    }
    else if (drawState == Tracking::OK) // TRACKING
    {
        trackedCount   = 0;
        trackedVOCount = 0;
        const float r  = 5;
        int         n  = vCurrentKeys.size();
        for (int i = 0; i < n; i++)
        {
            if (vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1, pt2;
                cv::Point2f point;
                if (imageScale != 1.f)
                {
                    point    = vCurrentKeys[i].pt / imageScale;
                    float px = vCurrentKeys[i].pt.x / imageScale;
                    float py = vCurrentKeys[i].pt.y / imageScale;
                    pt1.x    = px - r;
                    pt1.y    = py - r;
                    pt2.x    = px + r;
                    pt2.y    = py + r;
                }
                else
                {
                    point = vCurrentKeys[i].pt;
                    pt1.x = vCurrentKeys[i].pt.x - r;
                    pt1.y = vCurrentKeys[i].pt.y - r;
                    pt2.x = vCurrentKeys[i].pt.x + r;
                    pt2.y = vCurrentKeys[i].pt.y + r;
                }

                // This is a match to a MapPoint in the map
                if (vbMap[i])
                {
                    cv::rectangle(im, pt1, pt2, standardColor);
                    cv::circle(im, point, 2, standardColor, -1);
                    trackedCount++;
                }
                else // This is match to a "visual odometry" MapPoint created in
                     // the last frame
                {
                    cv::rectangle(im, pt1, pt2, odometryColor);
                    cv::circle(im, point, 2, odometryColor, -1);
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
