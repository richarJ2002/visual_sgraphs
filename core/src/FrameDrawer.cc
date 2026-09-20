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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

namespace vs_graphs
{
namespace core
{

FrameDrawer::FrameDrawer(Atlas *pAtlas) :
    both(false),
    p_atlas(pAtlas)
{
    state      = Tracking::SYSTEM_NOT_READY;
    image      = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    imageRight = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

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

void FrameDrawer::update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->imageGray.copyTo(image);
    currentKeys    = pTracker->currentFrame.keyPoints;
    depthThreshold = pTracker->currentFrame.depthThreshold;
    currentDepths  = pTracker->currentFrame.depths;

    if (both)
    {
        currentKeysRight = pTracker->currentFrame.keyPointsRight;
        pTracker->imageRight.copyTo(imageRight);
        N = currentKeys.size() + currentKeysRight.size();
    }
    else
    {
        N = currentKeys.size();
    }

    mvbVO        = vector<bool>(N, false);
    mvbMap       = vector<bool>(N, false);
    onlyTracking = pTracker->onlyTracking;

    // Variables for the new visualization
    currentFrame  = pTracker->currentFrame;
    projectPoints = currentFrame.projectedPoints;
    matchedInImage.clear();

    localMap = pTracker->getLocalMapPoints();
    matchedKeys.clear();
    matchedKeys.reserve(N);
    matchedMPs.clear();
    matchedMPs.reserve(N);
    outlierKeys.clear();
    outlierKeys.reserve(N);
    outlierMPs.clear();
    outlierMPs.reserve(N);

    if (pTracker->lastProcessedState == Tracking::NOT_INITIALIZED)
    {
        iniKeys    = pTracker->initialFrame.keyPoints;
        iniMatches = pTracker->iniMatches;
    }
    else if (pTracker->lastProcessedState == Tracking::OK)
    {
        for (int i = 0; i < N; i++)
        {
            MapPoint *pMP = pTracker->currentFrame.mapPoints[i];
            if (pMP)
            {
                if (!pTracker->currentFrame.outlierFlags[i])
                {
                    if (pMP->getObservationCount() > 0)
                        mvbMap[i] = true;
                    else
                        mvbVO[i] = true;

                    matchedInImage[pMP->mnId] = currentKeys[i].pt;
                }
                else
                {
                    outlierMPs.push_back(pMP);
                    outlierKeys.push_back(currentKeys[i]);
                }
            }
        }
    }
    state = static_cast<int>(pTracker->lastProcessedState);
}

} // namespace core
} // namespace vs_graphs
