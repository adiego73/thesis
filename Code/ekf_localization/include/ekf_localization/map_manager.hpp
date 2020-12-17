#ifndef EKF_LOCALIZATION_MAP_MANAGER_HPP
#define EKF_LOCALIZATION_MAP_MANAGER_HPP

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <utility>

#include <octomap/ColorOcTree.h>
#include <yaml-cpp/yaml.h>

#include "commons.hpp"
#include "marker.hpp"
#include "pole.hpp"

class MapManager
{

private:
    /**
     * Vector of landmarks. It can store Poles or Markers.
     */
    std::vector<std::unique_ptr<Landmark>> m_landmarks;

    /**
     * Path to map file.
     */
    std::string m_path;

    /**
     * Octomap binary map path
     */
    std::string m_octomap_path;

    /**
     * Octomap tree.
     */
    octomap::ColorOcTree *m_3d_map = nullptr;

public:
    explicit MapManager(const std::string &path) :
        m_path(path){};

    MapManager(const std::string &path, const std::string &octomap_path);
    /**
     * Save map to file
     */
    void
    save() const;

    /**
     * Load map from file
     */
    void
    load();

    /**
     * Add landmark to map
     * @tparam T
     * @param landmark
     * @return
     */
    template<class T>
    T &
    addLandmark(std::unique_ptr<T> landmark);

    /**
     * Finds and retunrs a landmark.
     * @tparam T
     * @param id
     * @return
     */
    template<class T>
    Landmark &
    getLandmark(int id) const;

    /**
     * Returns true if there exist a landmark with ID = id
     * @tparam T
     * @param id
     * @return
     */
    template<class T>
    bool
    exists(int id) const;

    /**
     * Resturns a vector with all the stored landmarks of type T
     * @tparam T
     * @return
     */
    template<class T>
    std::vector<T *>
    getAll() const;

    void
    updateOctomap(octomap::ColorOcTree *map);

    bool
    getVoxelFrom(const Eigen::Vector3d &origin, Eigen::Vector3d &out_voxel);

    ~MapManager();

    // remove copy and move constructors and operators.
    MapManager(const MapManager &) = delete;

    MapManager(MapManager &&other) = delete;

    MapManager &
    operator=(const MapManager &) = delete;

    MapManager &
    operator=(MapManager &&other) = delete;

    struct LandmarkNode {
        int id;
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };
};

#endif//EKF_LOCALIZATION_MAP_MANAGER_HPP
