/*!
 * @File:         publishTopics.cpp
 *
 * @Brief:
 *
 * @Date:         20/07/2026
 *
 */

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

void publishTopics(const rclcpp::Time    &msgTime_s_in,
                   const Eigen::Vector3f &angularVelocity_body_radps_in,
                   const sensor_msgs::msg::PointCloud2::ConstSharedPtr
                       &pointCloud_cameraMessage_in)
{
    /* Confirm that the SLAM system has been initialised */
    if (pSLAM == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("visual_sgraphs"),
                     "Cannot publish topics: SLAM system is not initialised.");

        return;
    }

    /* Obtain the current camera pose relative to the world frame */
    const Sophus::SE3f T_world_camera_SE3f = pSLAM->GetCamTwc();

    /* Prevent invalid camera transformations from entering ROS messages */
    if (!T_world_camera_SE3f.translation().allFinite() ||
        !T_world_camera_SE3f.rotationMatrix().allFinite())
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot publish topics: camera pose contains non-finite values.");

        return;
    }

    /* ---------------------------------------------------------------------- *
     * CAMERA AND FRAME TOPICS
     * ---------------------------------------------------------------------- */

    publishCameraPose(T_world_camera_SE3f, msgTime_s_in);

    publishTFTransform(T_world_camera_SE3f,
                       frameWorld,
                       frameCamera,
                       msgTime_s_in);

    publishFramePointCloud(T_world_camera_SE3f,
                           pointCloud_cameraMessage_in,
                           msgTime_s_in);

    /*!
     * A static transform only needs to be published once. The static TF
     * broadcaster stores the transform for future subscribers.
     */
    static bool hasPublishedWorldMapStaticTransform = false;

    if (pubStaticTransform && !hasPublishedWorldMapStaticTransform &&
        staticTfBroadcaster != nullptr)
    {
        publishStaticTFTransform(frameWorld, frameMap, msgTime_s_in);

        hasPublishedWorldMapStaticTransform = true;
    }

    /* ---------------------------------------------------------------------- *
     * OBTAIN A SNAPSHOT OF THE CURRENT MAP COLLECTIONS
     * ---------------------------------------------------------------------- */

    const std::vector<ORB_SLAM3::KeyFrame *> mappedKeyFrames =
        pSLAM->GetAllKeyFrames();

    const std::vector<ORB_SLAM3::Marker *> mappedFiducialMarkers =
        pSLAM->GetAllMarkers();

    const std::vector<ORB_SLAM3::Room *> mappedRooms = pSLAM->GetAllRooms();

    const std::vector<ORB_SLAM3::Floor *> mappedFloors = pSLAM->GetAllFloors();

    const std::vector<ORB_SLAM3::Passage *> mappedPassages =
        pSLAM->GetAllPassages();

    const std::vector<ORB_SLAM3::Plane *> mappedPlanes = pSLAM->GetAllPlanes();

    /* ---------------------------------------------------------------------- *
     * KEYFRAMES, TRACKING, AND STRUCTURAL ELEMENTS
     * ---------------------------------------------------------------------- */

    publishKeyFrameImages(mappedKeyFrames, msgTime_s_in);
    publishKeyFrameMarkers(mappedKeyFrames, msgTime_s_in);
    publishFiducialMarkers(mappedFiducialMarkers, msgTime_s_in);
    publishTrackingImage(pSLAM->GetCurrentFrame(), msgTime_s_in);
    publishStructuralElements(mappedRooms,
                              mappedFloors,
                              mappedPassages,
                              msgTime_s_in);

    /*!
     * Publish mapped walls independently of the point-cloud visualisation
     * setting because they are consumed by GNN-based room detection.
     */
    publishAllMappedWalls(mappedPlanes, msgTime_s_in);

    /* ------------------------------------------------------------------ *
     * POINT-CLOUD TOPICS
     * ------------------------------------------------------------------ */

    if (pubPointClouds)
    {
        publishSegmentedCloud(mappedKeyFrames);
        publishPlanes(mappedPlanes, msgTime_s_in);
        publishAllPoints(pSLAM->GetAllMapPoints(), msgTime_s_in);
        publishTrackedPoints(pSLAM->GetTrackedMapPoints(), msgTime_s_in);
        publishFreeSpaceClusters(pSLAM->getSkeletonCluster(), msgTime_s_in);
    }
    else
    {
        /*!
         * Semantic class clouds are temporary and should be cleared when their
         * publication has been disabled.
         */
        clearKFClsClouds(mappedKeyFrames);
    }

    /* ---------------------------------------------------------------------- *
     * INERTIAL TOPICS
     * ---------------------------------------------------------------------- */

    const bool usesInertialSensor =
        sensorType == ORB_SLAM3::System::IMU_MONOCULAR ||
        sensorType == ORB_SLAM3::System::IMU_STEREO ||
        sensorType == ORB_SLAM3::System::IMU_RGBD;

    if (!usesInertialSensor)
    {
        return;
    }

    /* T_world_body_SE3f describes the body pose relative to the world frame */
    const Sophus::SE3f T_world_body_SE3f = pSLAM->GetImuTwb();

    /*!
     * ORB-SLAM3 supplies the body linear velocity expressed in the world
     * frame.
     */
    const Eigen::Vector3f linearVelocity_world_mps = pSLAM->GetImuVwb();

    /* Validate all inertial quantities before publication */
    if (!T_world_body_SE3f.translation().allFinite() ||
        !T_world_body_SE3f.rotationMatrix().allFinite() ||
        !linearVelocity_world_mps.allFinite() ||
        !angularVelocity_body_radps_in.allFinite())
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot publish inertial topics: inertial state contains "
                    "non-finite values.");

        return;
    }

    publishTFTransform(T_world_body_SE3f, frameWorld, frameImu, msgTime_s_in);

    /*!
     * The angular velocity is already expressed in the body frame and should
     * not be transformed into the world frame.
     *
     * publishBodyOdometry() should rotate the world-frame linear velocity into
     * the body frame before filling the Odometry twist field.
     */
    publishBodyOdometry(T_world_body_SE3f,
                        linearVelocity_world_mps,
                        angularVelocity_body_radps_in,
                        msgTime_s_in);
}