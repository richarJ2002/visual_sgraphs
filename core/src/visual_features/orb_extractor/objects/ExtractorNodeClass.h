/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

/*!
 * @file            ExtractorNodeClass.h
 *
 * @brief           Declares ExtractorNode, the image region node of the octree
 *                  used to spread keypoints.
 */

#ifndef VS_GRAPHS_CORE_SRC_ORB_EXTRACTOR_OBJECTS_EXTRACTORNODE_CLASS_H
#define VS_GRAPHS_CORE_SRC_ORB_EXTRACTOR_OBJECTS_EXTRACTORNODE_CLASS_H

#include "visual_features/orb_extractor/objects/ExtractorNodeStatusEnum.h"

#include <list>
#include <opencv2/core.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace orb_extractor
{

/*!
 * @brief           Image region node of the octree used to spread
 *                  keypoints.
 */
class ExtractorNode
{
  public:
    /*!
     * @brief           Creates an unexhausted node.
     */
    ExtractorNode() :
        isExhausted(false)
    {}

    /*!
     * @brief           Splits the node into four quadrant children.
     *
     * @param[in,out]   node1_inout
     *                  Top-left child.
     *
     * @param[in,out]   node2_inout
     *                  Top-right child.
     *
     * @param[in,out]   node3_inout
     *                  Bottom-left child.
     *
     * @param[in,out]   node4_inout
     *                  Bottom-right child.
     */
    [[nodiscard]] ExtractorNodeStatus divideNode(ExtractorNode &node1_inout,
                                                 ExtractorNode &node2_inout,
                                                 ExtractorNode &node3_inout,
                                                 ExtractorNode &node4_inout);

    /*!
     * @brief           Keypoints falling inside the node.
     */
    std::vector<cv::KeyPoint>          keys;
    /*!
     * @brief           Region corners in pixels.
     */
    cv::Point2i                        topLeft;
    /*!
     * @brief           Top-right corner of the node region, in pixels.
     */
    cv::Point2i                        topRight;
    /*!
     * @brief           Bottom-left corner of the node region, in pixels.
     */
    cv::Point2i                        bottomLeft;
    /*!
     * @brief           Bottom-right corner of the node region, in pixels.
     */
    cv::Point2i                        bottomRight;
    /*!
     * @brief           Position of the node in the level list.
     */
    std::list<ExtractorNode>::iterator nodeIterator;
    /*!
     * @brief           True once the node holds no distributable
     *                  keypoints.
     */
    bool                               isExhausted;
};

} /* namespace orb_extractor */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_ORB_EXTRACTOR_OBJECTS_EXTRACTORNODE_CLASS_H */
