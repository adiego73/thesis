#include "ekf_localization/map_manager.hpp"

namespace YAML
{
template<>
struct convert<MapManager::LandmarkNode> {
    static bool
    decode(const Node &node, MapManager::LandmarkNode &landmark)
    {
        landmark.id = node["id"].as<int>();
        landmark.x = node["x"].as<double>();
        landmark.y = node["y"].as<double>();
        landmark.z = node["z"].as<double>();
        landmark.roll = node["roll"].as<double>();
        landmark.pitch = node["pitch"].as<double>();
        landmark.yaw = node["yaw"].as<double>();
        return true;
    }

    static Node
    encode(const MapManager::LandmarkNode &landmark)
    {
        Node node;
        node["id"] = landmark.id;
        node["x"] = landmark.x;
        node["y"] = landmark.y;
        node["z"] = landmark.z;
        node["roll"] = landmark.roll;
        node["pitch"] = landmark.pitch;
        node["yaw"] = landmark.yaw;
        return node;
    }
};
}// namespace YAML

MapManager::MapManager(const std::string &path, const std::string &octomap_path) :
    m_path(path), m_octomap_path(octomap_path)
{
    if (!octomap_path.empty())
    {
        m_3d_map = new octomap::ColorOcTree(0.05);
        if (!m_3d_map->readBinary(octomap_path)) {
            ROS_ERROR("Binary octomap couldn't be read. Octomap path is: %s", octomap_path.c_str());
        }
    }
}

template<>
Pole &
MapManager::addLandmark<Pole>(std::unique_ptr<Pole> landmark)
{
    m_landmarks.emplace_back(std::move(landmark));
    return (Pole &) (*m_landmarks.back());
}

template<>
Marker &
MapManager::addLandmark<Marker>(std::unique_ptr<Marker> landmark)
{
    m_landmarks.emplace_back(std::move(landmark));
    return (Marker &) (*m_landmarks.back());
}

template<>
bool
MapManager::exists<Pole>(int id) const
{
    for (const auto &landmark_ptr : m_landmarks)
    {
        if (LandmarkType::POLE == landmark_ptr->TYPE && id == landmark_ptr->ID)
        {
            return true;
        }
    }
    return false;
}

template<>
bool
MapManager::exists<Marker>(int id) const
{
    for (const auto &landmark_ptr : m_landmarks)
    {
        if (LandmarkType::MARKER == landmark_ptr->TYPE && id == landmark_ptr->ID)
        {
            return true;
        }
    }
    return false;
}

template<>
Landmark &
MapManager::getLandmark<Marker>(int id) const
{
    for (const auto &landmark_ptr : m_landmarks)
    {
        if (LandmarkType::MARKER == landmark_ptr->TYPE && id == landmark_ptr->ID)
        {
            return *landmark_ptr;
        }
    }
    throw std::out_of_range("Marker " + std::to_string(id) + " not found as landmark. Consider adding it");
}

template<>
Landmark &
MapManager::getLandmark<Pole>(int id) const
{
    for (const auto &landmark_ptr : m_landmarks)
    {
        if (LandmarkType::POLE == landmark_ptr->TYPE && id == landmark_ptr->ID)
        {
            return *landmark_ptr;
        }
    }

    throw std::out_of_range("Pole " + std::to_string(id) + " not found as landmark. Consider adding it");
}

template<>
std::vector<Marker *>
MapManager::getAll() const
{
    std::vector<Marker *> markers;
    for (const auto &landmark_ptr : m_landmarks)
    {
        if (LandmarkType::MARKER == landmark_ptr->TYPE)
        {
            markers.emplace_back((Marker *) landmark_ptr.get());
        }
    }
    return markers;
}

template<>
std::vector<Pole *>
MapManager::getAll() const
{
    std::vector<Pole *> poles;
    for (const auto &landmark_ptr : m_landmarks)
    {
        if (LandmarkType::POLE == landmark_ptr->TYPE)
        {
            poles.emplace_back((Pole *) landmark_ptr.get());
        }
    }
    return poles;
}

void
MapManager::save() const
{
    YAML::Node map_yaml;
    YAML::Node poles_seq;
    YAML::Node markers_seq;
    for (const auto &landmark_ptr : m_landmarks)
    {
        if (LandmarkType::POLE == landmark_ptr->TYPE) {
            LandmarkNode pole{landmark_ptr->ID, landmark_ptr->getState(0), landmark_ptr->getState(1), landmark_ptr->getState(2)};
            poles_seq.push_back(pole);
        }
        else {
            LandmarkNode marker{landmark_ptr->ID, landmark_ptr->getState(0), landmark_ptr->getState(1), landmark_ptr->getState(2),
                                landmark_ptr->getState(3), landmark_ptr->getState(4), landmark_ptr->getState(5)};
            markers_seq.push_back(marker);
        }
    }

    if (poles_seq.size() > 0) {
        map_yaml["poles"] = poles_seq;
    }

    if (markers_seq.size() > 0) {
        map_yaml["markers"] = markers_seq;
    }

    std::ofstream map_file(m_path, std::ios::out | std::ios::trunc);
    if (map_file.is_open()) {
        map_file << map_yaml << std::endl;
        map_file.close();
    }
}

void
MapManager::load()
{
    YAML::Node map_file = YAML::LoadFile(m_path);
    YAML::Node poles_node = map_file["poles"];
    YAML::Node markers_node = map_file["markers"];

    if (poles_node) {
        for (YAML::const_iterator it = poles_node.begin(); it != poles_node.end(); ++it) {
            LandmarkNode pole = it->as<LandmarkNode>();
            m_landmarks.emplace_back(std::make_unique<Pole>(pole.id, pole.x, pole.y, pole.z));
        }
    }

    if (markers_node) {
        for (YAML::const_iterator it = markers_node.begin(); it != markers_node.end(); ++it) {
            LandmarkNode marker = it->as<LandmarkNode>();
            m_landmarks.emplace_back(std::make_unique<Marker>(marker.id, marker.x, marker.y, marker.z, marker.roll, marker.pitch, marker.yaw));
        }
    }
}
MapManager::~MapManager()
{
    delete m_3d_map;
}

void
MapManager::updateOctomap(octomap::ColorOcTree *map)
{
    // if the octomap path is empty, update the octomap tree.
    if (m_octomap_path.empty())
    {
        delete m_3d_map;
        m_3d_map = map;
    }
}

bool
MapManager::getVoxelFrom(const Eigen::Vector3d &origin, Eigen::Vector3d &out_voxel)
{
    out_voxel << origin;

    if (m_3d_map == nullptr) {
        return false;
    }

    const octomap::point3d ray_origin(origin(0), origin(1), origin(2));
    const octomap::point3d direction(0, 0, -1);
    octomap::point3d ray_end;

    if (m_3d_map->castRay(ray_origin, direction, ray_end, true, 3)) {
//        ROS_INFO("Occupied cell at %f %f %f", ray_end(0), ray_end(1), ray_end(2));
        out_voxel << ray_end(0), ray_end(1), ray_end(2);
        return true;
    }

    return false;
}