// Reads a ROS 1 bag recorded from DALI-SLAM (DA-LIO) containing:
//   /Odometry          (nav_msgs/Odometry)
//   /cloud_registered  (sensor_msgs/PointCloud2 in world/map frame)
// and writes an HDMapping session (chunked LAZ + trajectory CSVs + session.json).
//
// DA-LIO is FAST-LIO2-based: it publishes the deskewed scan already registered
// into the global frame on /cloud_registered, and the body pose on /Odometry,
// so the converter only chunks the world points and rebuilds the per-chunk
// trajectory from the odometry poses (no extra transform applied to points).

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <Eigen/Dense>
#include "laszip_api.h"
#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <cstring>

#include "laz_writer.hpp"


struct TrajectoryPose
{
    uint64_t timestamp_ns;
    double x_m;
    double y_m;
    double z_m;
    double qw;
    double qx;
    double qy;
    double qz;
    Eigen::Affine3d pose;
    double om_rad;  // Roll (omega)
    double fi_rad;  // Pitch (phi)
    double ka_rad;  // Yaw (kappa)
};

struct TaitBryanPose
{
    double px;
    double py;
    double pz;
    double om;
    double fi;
    double ka;
};

inline TaitBryanPose pose_tait_bryan_from_affine_matrix(Eigen::Affine3d m){
    TaitBryanPose pose;

    pose.px = m(0,3);
    pose.py = m(1,3);
    pose.pz = m(2,3);

    if (m(0,2) < 1) {
        if (m(0,2) > -1) {
            pose.fi = asin(m(0,2));
            pose.om = atan2(-m(1,2), m(2,2));
            pose.ka = atan2(-m(0,1), m(0,0));
            return pose;
        }
        else
        {
            pose.fi = -M_PI / 2.0;
            pose.om = -atan2(m(1,0), m(1,1));
            pose.ka = 0;
            return pose;
        }
    }
    else {
        pose.fi = M_PI / 2.0;
        pose.om = atan2(m(1,0), m(1,1));
        pose.ka = 0.0;
        return pose;
    }

    return pose;
}

namespace fs = std::filesystem;
std::vector<Point3Di> points_global;

std::vector<TrajectoryPose> trajectory;
std::vector<std::vector<TrajectoryPose>> chunks_trajectory;

bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.good())
    {
        std::cout << "can not save file: '" << file_name << "'" << std::endl;
        return false;
    }

    outfile << m_poses.size() << std::endl;
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        outfile << filenames[i] << std::endl;
        outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3) << std::endl;
        outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3) << std::endl;
        outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3) << std::endl;
        outfile << "0 0 0 1" << std::endl;
    }
    outfile.close();

    return true;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_bag> <output_directory> [odom_topic] [cloud_topic]" << std::endl;
        std::cout << "  Defaults: odom_topic=/Odometry  cloud_topic=/cloud_registered" << std::endl;
        std::cout << "  Expects topics: /Odometry (nav_msgs/Odometry)" << std::endl;
        std::cout << "                  /cloud_registered (sensor_msgs/PointCloud2, world frame)" << std::endl;
        return 1;
    }

    const std::string input_bag = argv[1];
    const std::string output_directory = argv[2];

    const std::string odom_topic  = (argc > 3) ? argv[3] : "/Odometry";
    const std::string cloud_topic = (argc > 4) ? argv[4] : "/cloud_registered";

    std::cout << "Processing bag: " << input_bag << std::endl;
    std::cout << "  odom topic  : " << odom_topic << std::endl;
    std::cout << "  cloud topic : " << cloud_topic << std::endl;

    // ── Open ROS 1 bag ───────────────────────────────────────────────────────
    rosbag::Bag bag;
    try {
        bag.open(input_bag, rosbag::bagmode::Read);
    } catch (const rosbag::BagException &e) {
        std::cerr << "Failed to open bag: " << e.what() << std::endl;
        return 1;
    }

    std::vector<std::string> topics{odom_topic, cloud_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    struct CloudEntry {
        uint64_t timestamp_ns;
        sensor_msgs::PointCloud2::ConstPtr cloud;
    };
    std::vector<CloudEntry> clouds;

    std::cout << "Reading odometry and clouds..." << std::endl;
    for (const rosbag::MessageInstance &m : view)
    {
        if (m.getTopic() == odom_topic)
        {
            nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
            if (!odom_msg) continue;

            double x = odom_msg->pose.pose.position.x;
            double y = odom_msg->pose.pose.position.y;
            double z = odom_msg->pose.pose.position.z;

            double qx = odom_msg->pose.pose.orientation.x;
            double qy = odom_msg->pose.pose.orientation.y;
            double qz = odom_msg->pose.pose.orientation.z;
            double qw = odom_msg->pose.pose.orientation.w;

            TrajectoryPose pose;
            // Prefer the message's own header stamp; fall back to bag stamp.
            ros::Time stamp = odom_msg->header.stamp;
            if (stamp.isZero()) stamp = m.getTime();
            pose.timestamp_ns = static_cast<uint64_t>(stamp.toNSec());

            pose.x_m = x;
            pose.y_m = y;
            pose.z_m = z;
            pose.qw = qw;
            pose.qx = qx;
            pose.qy = qy;
            pose.qz = qz;

            pose.pose = Eigen::Affine3d::Identity();
            Eigen::Vector3d trans(x, y, z);
            Eigen::Quaterniond q(qw, qx, qy, qz);
            pose.pose.translation() = trans;
            pose.pose.linear() = q.toRotationMatrix();

            TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(pose.pose);
            pose.om_rad = tb.om;
            pose.fi_rad = tb.fi;
            pose.ka_rad = tb.ka;

            trajectory.push_back(pose);
        }
        else if (m.getTopic() == cloud_topic)
        {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (!cloud_msg) continue;

            CloudEntry entry;
            ros::Time stamp = cloud_msg->header.stamp;
            if (stamp.isZero()) stamp = m.getTime();
            entry.timestamp_ns = static_cast<uint64_t>(stamp.toNSec());
            entry.cloud = cloud_msg;
            clouds.push_back(std::move(entry));
        }
    }
    bag.close();

    // Sort trajectory in case messages were out of order
    std::sort(trajectory.begin(), trajectory.end(),
        [](const TrajectoryPose &a, const TrajectoryPose &b){ return a.timestamp_ns < b.timestamp_ns; });
    std::sort(clouds.begin(), clouds.end(),
        [](const CloudEntry &a, const CloudEntry &b){ return a.timestamp_ns < b.timestamp_ns; });

    std::cout << "Read " << trajectory.size() << " odometry poses and "
              << clouds.size() << " point clouds." << std::endl;

    if (!trajectory.empty() && !clouds.empty()) {
        std::cout << "DEBUG: odom  timestamp range: [" << trajectory.front().timestamp_ns
                  << " .. " << trajectory.back().timestamp_ns << "]" << std::endl;
        std::cout << "DEBUG: cloud timestamp range: [" << clouds.front().timestamp_ns
                  << " .. " << clouds.back().timestamp_ns << "]" << std::endl;
    }

    if (trajectory.empty() || clouds.empty()) {
        std::cerr << "Error: no odometry or cloud data found in bag!" << std::endl;
        return 1;
    }

    // ── Collect global (world-frame) points ──────────────────────────────────
    // DA-LIO's /cloud_registered is already published in the global (world)
    // frame, so no pose transform is needed here.
    std::cout << "Collecting world-frame point clouds..." << std::endl;

    for (const auto &ce : clouds) {
        const auto &cloud_msg = *ce.cloud;
        size_t num_points = cloud_msg.width * cloud_msg.height;
        if (num_points == 0) continue;

        int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;
        for (const auto &field : cloud_msg.fields) {
            if (field.name == "x") x_offset = field.offset;
            if (field.name == "y") y_offset = field.offset;
            if (field.name == "z") z_offset = field.offset;
            if (field.name == "intensity") intensity_offset = field.offset;
        }
        if (x_offset < 0 || y_offset < 0 || z_offset < 0) continue;

        for (size_t i = 0; i < num_points; ++i) {
            size_t byte_offset = i * cloud_msg.point_step;
            float px, py, pz;
            std::memcpy(&px, &cloud_msg.data[byte_offset + x_offset], sizeof(float));
            std::memcpy(&py, &cloud_msg.data[byte_offset + y_offset], sizeof(float));
            std::memcpy(&pz, &cloud_msg.data[byte_offset + z_offset], sizeof(float));

            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

            float intensity = 0.0f;
            if (intensity_offset >= 0) {
                std::memcpy(&intensity, &cloud_msg.data[byte_offset + intensity_offset], sizeof(float));
            }

            Point3Di point_global;
            point_global.timestamp = ce.timestamp_ns;
            point_global.point = Eigen::Vector3d(px, py, pz);
            point_global.intensity = intensity;
            point_global.index_pose = static_cast<int>(i);
            point_global.lidarid = 0;
            point_global.index_point = static_cast<int>(i);

            points_global.push_back(point_global);
        }
    }

    std::cout << "Total global points: " << points_global.size() << std::endl;

    // ── Chunk point clouds ───────────────────────────────────────────────────
    std::vector<std::vector<Point3Di>> chunks_pc;
    int counter = 0;
    std::vector<Point3Di> chunk;

    for (size_t i = 0; i < points_global.size(); i++)
    {
        chunk.push_back(points_global[i]);
        if (chunk.size() > 2000000)
        {
            counter++;
            chunks_pc.push_back(chunk);
            chunk.clear();
            std::cout << "adding chunk [" << counter << "]" << std::endl;
        }
    }

    std::cout << "remaining points: " << chunk.size() << std::endl;
    if (chunk.size() > 1000000)
    {
        chunks_pc.push_back(chunk);
    }

    std::cout << "cleaning points" << std::endl;
    points_global.clear();
    std::cout << "points cleaned" << std::endl;

    // ── Index trajectory into chunks ─────────────────────────────────────────
    std::cout << "start indexing chunks_trajectory" << std::endl;
    chunks_trajectory.resize(chunks_pc.size());

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        if (i % 1000 == 0){
            std::cout << "computing [" << i + 1 << "] of: " << trajectory.size() << std::endl;
        }
        for (size_t j = 0; j < chunks_pc.size(); j++)
        {
            if (!chunks_pc[j].empty())
            {
                if (trajectory[i].timestamp_ns >= chunks_pc[j].front().timestamp &&
                    trajectory[i].timestamp_ns <= chunks_pc[j].back().timestamp)
                {
                    chunks_trajectory[j].push_back(trajectory[i]);
                    break;
                }
            }
        }
    }

    for (const auto &trj : chunks_trajectory)
    {
        std::cout << "number of trajectory elements: " << trj.size() << std::endl;
    }

    // ── Transform chunks to local coordinate system (relative to first pose) ─
    std::cout << "start transforming chunks_pc to local coordinate system" << std::endl;
    for (size_t i = 0; i < chunks_pc.size(); i++)
    {
        std::cout << "computing [" << i + 1 << "] of: " << chunks_pc.size() << std::endl;
        if (chunks_trajectory[i].empty()){
            continue;
        }

        Eigen::Vector3d trans(chunks_trajectory[i][0].x_m, chunks_trajectory[i][0].y_m, chunks_trajectory[i][0].z_m);
        Eigen::Quaterniond q(chunks_trajectory[i][0].qw, chunks_trajectory[i][0].qx, chunks_trajectory[i][0].qy, chunks_trajectory[i][0].qz);

        Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
        first_affine.translation() = trans;
        first_affine.linear() = q.toRotationMatrix();

        Eigen::Affine3d first_affine_inv = first_affine.inverse();

        for (auto &p : chunks_pc[i])
        {
            p.point = first_affine_inv * p.point;
        }
    }

    // ── Create output directory ──────────────────────────────────────────────
    if (fs::exists(output_directory)) {
        std::cout << "Directory already exists." << std::endl;
    } else {
        try {
            if (fs::create_directory(output_directory)) {
                std::cout << "Directory has been created." << std::endl;
            } else {
                std::cerr << "Failed to create directory " << std::endl;
                return 1;
            }
        } catch (const fs::filesystem_error &e) {
            std::cerr << "Error creating directory: " << e.what() << std::endl;
            return 1;
        }
    }

    fs::path outwd = output_directory;

    // ── Compute offset (mean trajectory position) ────────────────────────────
    Eigen::Vector3d offset(0, 0, 0);
    int cc = 0;
    for (size_t i = 0; i < chunks_trajectory.size(); i++)
    {
        for (size_t j = 0; j < chunks_trajectory[i].size(); j++)
        {
            Eigen::Vector3d trans_curr(chunks_trajectory[i][j].x_m, chunks_trajectory[i][j].y_m, chunks_trajectory[i][j].z_m);
            offset += trans_curr;
            cc++;
        }
    }
    if (cc > 0) {
        offset /= cc;
    } else {
        std::cerr << "WARNING: No trajectory elements matched any chunk! Using offset=0." << std::endl;
    }

    // ── Save LAZ files and trajectory CSVs ───────────────────────────────────
    std::vector<Eigen::Affine3d> m_poses;
    std::vector<std::string> file_names;

    for (size_t i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_pc[i].empty()) continue;
        if (chunks_trajectory[i].empty()) continue;

        fs::path path(outwd);
        std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
        path /= filename;
        std::cout << "saving to: " << path << " number of points: " << chunks_pc[i].size() << std::endl;
        saveLaz(path.string(), chunks_pc[i]);
        file_names.push_back(filename);

        std::string trajectory_filename = ("trajectory_lio_" + std::to_string(i) + ".csv");
        fs::path pathtrj(outwd);
        pathtrj /= trajectory_filename;
        std::cout << "saving to: " << pathtrj << std::endl;

        std::ofstream outfile;
        outfile.open(pathtrj);
        if (!outfile.good())
        {
            std::cout << "can not save file: " << pathtrj << std::endl;
            return 1;
        }

        outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds om_rad fi_rad ka_rad" << std::endl;

        Eigen::Vector3d trans(chunks_trajectory[i][0].x_m, chunks_trajectory[i][0].y_m, chunks_trajectory[i][0].z_m);
        Eigen::Quaterniond q(chunks_trajectory[i][0].qw, chunks_trajectory[i][0].qx, chunks_trajectory[i][0].qy, chunks_trajectory[i][0].qz);

        Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
        first_affine.translation() = trans;
        first_affine.linear() = q.toRotationMatrix();

        Eigen::Affine3d first_affine_inv = first_affine.inverse();
        m_poses.push_back(first_affine);

        for (size_t j = 0; j < chunks_trajectory[i].size(); j++)
        {
            Eigen::Vector3d trans_curr(chunks_trajectory[i][j].x_m, chunks_trajectory[i][j].y_m, chunks_trajectory[i][j].z_m);
            Eigen::Quaterniond q_curr(chunks_trajectory[i][j].qw, chunks_trajectory[i][j].qx, chunks_trajectory[i][j].qy, chunks_trajectory[i][j].qz);

            Eigen::Affine3d first_affine_curr = Eigen::Affine3d::Identity();
            first_affine_curr.translation() = trans_curr;
            first_affine_curr.linear() = q_curr.toRotationMatrix();

            auto pose = first_affine_inv * first_affine_curr;
            outfile
                << chunks_trajectory[i][j].timestamp_ns << " " << std::setprecision(10)
                << pose(0, 0) << " "
                << pose(0, 1) << " "
                << pose(0, 2) << " "
                << pose(0, 3) << " "
                << pose(1, 0) << " "
                << pose(1, 1) << " "
                << pose(1, 2) << " "
                << pose(1, 3) << " "
                << pose(2, 0) << " "
                << pose(2, 1) << " "
                << pose(2, 2) << " "
                << pose(2, 3) << " "
                << chunks_trajectory[i][j].timestamp_ns << " "
                << std::setprecision(20)
                << chunks_trajectory[i][j].om_rad << " "
                << chunks_trajectory[i][j].fi_rad << " "
                << chunks_trajectory[i][j].ka_rad << " "
                << std::endl;
        }
        outfile.close();
    }

    for (auto &m : m_poses)
    {
        m.translation() -= offset;
    }

    // ── Save pose files ──────────────────────────────────────────────────────
    fs::path path(outwd);
    path /= "lio_initial_poses.reg";
    save_poses(path.string(), m_poses, file_names);
    fs::path path2(outwd);
    path2 /= "poses.reg";
    save_poses(path2.string(), m_poses, file_names);

    // ── Save session.json ────────────────────────────────────────────────────
    fs::path path3(outwd);
    path3 /= "session.json";

    std::cout << "saving file: '" << path3 << "'" << std::endl;

    nlohmann::json jj;
    nlohmann::json j;
    j["offset_x"] = 0.0;
    j["offset_y"] = 0.0;
    j["offset_z"] = 0.0;
    j["folder_name"] = outwd.string();
    j["out_folder_name"] = outwd.string();
    j["poses_file_name"] = (outwd / "poses.reg").string();
    j["initial_poses_file_name"] = (outwd / "lio_initial_poses.reg").string();
    j["out_poses_file_name"] = (outwd / "poses.reg").string();
    j["lidar_odometry_version"] = "HdMap";

    jj["Session Settings"] = j;

    nlohmann::json jlaz_file_names;
    for (size_t i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_pc[i].empty()) continue;
        if (chunks_trajectory[i].empty()) continue;

        fs::path p(outwd);
        std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
        p /= filename;
        std::cout << "adding file: " << p << std::endl;
        nlohmann::json jfn{ {"file_name", p.string()} };
        jlaz_file_names.push_back(jfn);
    }
    jj["laz_file_names"] = jlaz_file_names;

    std::ofstream fs(path3.string());
    fs << jj.dump(2);
    fs.close();

    return 0;
}
