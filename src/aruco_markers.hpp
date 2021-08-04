/**
 * @file sim/aruco_marker.hpp
 * @brief Aruco markers creation and insertion into simulation
 * @author Vaishnavi Dintakurthi vaishnavi.dintakurthi@de.kaercher.com
 * @copyright 2020 Alfred Kaercher SE & Co. KG
 */

#ifndef ARUCO_MARKER_HPP
#define ARUCO_MARKER_HPP
#include "ros/ros.h"
#include "sim/utils.hpp"
namespace Sim {
class ArucoMarker
{
public:
    using plane_size_t = std::array<float, 2>;
    /** @brief Constructor.
     *  @param ROS node to register the publisher. */
    ArucoMarker(ros::NodeHandle& nh);

    /** @brief loads the marker's description from config file.*/
    void LoadMarkerDescriptionFromRos();

    /** @brief generate images of markers
     *  @param outputDirectory directory to store the images .
     */
    void GenerateImages(const std::string& outputDirectory) const;

    /** @brief loads the marker and places in the scene.*/
    void CreateAndPlaceMarker();

    /** @brief create markers in simulation and place them at respective location and orientation
     *  @param outputDirectory directory to store the images .
     */
    void PlaceMarkersIntoSim(const std::string& outputDirectory) const;

private:
    struct Marker
    {
        int id;
        sim_vector_t pos;
        sim_vector_t orientation;
    };
    std::vector<Marker> m_markers;
    ros::NodeHandle& m_nh;
    plane_size_t m_size; /**Marker size in m */
    std::string m_outputDirectory{"/tmp/aruco"};
};
} // namespace Sim
#endif // ARUCOMARKER_HPP
