/*!
 * @file            getImage.cc
 *
 * @brief           Converts an inertial frontend image through cv_bridge.
 *
 * @date            09/10/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

#include "image_grabber/objects/ImageGrabberClass.h"

namespace vs_graphs::image_grabber
{

cv::Mat ImageGrabber::getImage(
    const sensor_msgs::msg::Image::ConstSharedPtr &p_imageMessage_in)
{
    /*Init variable to store image as an openCv object */
    cv_bridge::CvImageConstPtr p_cvImage;

    /* Copy the ros image message to cv::Mat. */
    try
    {
        p_cvImage = cv_bridge::toCvShare(p_imageMessage_in);
    }
    catch (const cv_bridge::Exception &exception)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "[Error] Problem occured while running `cv_bridge`: %s",
                     exception.what());
        return cv::Mat();
    }

    /* Return pointer containing clone of image */
    return p_cvImage->image.clone();
}

} /* namespace vs_graphs::image_grabber */
