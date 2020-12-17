#!/usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib as mpl
from rosbag import Bag
import numpy as np

# disable interactive mode
plt.ioff()
#  setup common styling
plt.style.use('seaborn-paper')
plt.rc('axes', axisbelow=True, grid=True)
plt.rc('grid', color='gray', linestyle='dotted')
plt.rc('xtick', direction='out', color='gray')
plt.rc('ytick', direction='out', color='gray')
plt.rc('patch', edgecolor='#E6E6E6')
plt.rc('lines', linewidth=2)


def quat2euler(quat):
    norm = np.sqrt(quat[0] ** 2 + quat[1] ** 2 + quat[2] ** 2 + quat[3] ** 2)

    # Normalize
    if norm != 0:

        for i in range(0, len(quat)):
            quat[i] = quat[i] / norm

        A11 = quat[0] ** 2 - quat[1] ** 2 - quat[2] ** 2 + quat[3] ** 2
        A12 = 2 * quat[0] * quat[1] + 2 * quat[2] * quat[3]
        A13 = 2 * quat[0] * quat[2] - 2 * quat[1] * quat[3]
        A23 = 2 * quat[0] * quat[3] + 2 * quat[1] * quat[2]
        A33 = - quat[0] ** 2 - quat[1] ** 2 + quat[2] ** 2 + quat[3] ** 2

        phi = np.arctan2(A23, A33)
        theta = -np.arcsin(A13)
        psi = np.arctan2(A12, A11)

        euler = [phi, theta, psi]

    else:

        euler = [0, 0, 0]

    return euler


def parse_bag(bag_file_path):
    bagfile = Bag(bag_file_path)
    msgs_topics = ["/drone/pose", "/gazebo/ground_truth", "/mavros/local_position/odom",
                   "/mavros/px4flow/ground_distance"]
    msgs_drone_pose = {"y": [], "z": [], "x": [], "roll": [], "pitch": [], "yaw": [], "time": [],
                       "label": "EKF-SLAM"}
    msgs_ground_truth = {"y": [], "z": [], "x": [], "roll": [], "pitch": [], "yaw": [], "time": [],
                         "label": "Ground Truth"}
    msgs_mavros_odom = {"y": [], "z": [], "x": [], "roll": [], "pitch": [], "yaw": [], "time": [],
                        "label": "MAVROS"}
    msgs_mavros_laser = {"range": [], "time": [], "label": "Laser Range"}

    for topic, message, time in bagfile.read_messages(msgs_topics):
        if topic == msgs_topics[0]:
            msgs_drone_pose["x"].append(message.pose.pose.position.x)
            msgs_drone_pose["y"].append(message.pose.pose.position.y)
            msgs_drone_pose["z"].append(message.pose.pose.position.z)

            quat = message.pose.pose.orientation
            euler = quat2euler([quat.x, quat.y, quat.z, quat.w])
            msgs_drone_pose["roll"].append(euler[0])
            msgs_drone_pose["pitch"].append(euler[1])
            msgs_drone_pose["yaw"].append(euler[2])

            msgs_drone_pose["time"].append(time.to_sec())
        if topic == msgs_topics[1]:
            msgs_ground_truth["x"].append(message.pose.pose.position.x)
            msgs_ground_truth["y"].append(message.pose.pose.position.y)
            msgs_ground_truth["z"].append(message.pose.pose.position.z)

            quat = message.pose.pose.orientation
            euler = quat2euler([quat.x, quat.y, quat.z, quat.w])
            msgs_ground_truth["roll"].append(euler[0])
            msgs_ground_truth["pitch"].append(euler[1])
            msgs_ground_truth["yaw"].append(euler[2])

            msgs_ground_truth["time"].append(time.to_sec())
        if topic == msgs_topics[2]:
            msgs_mavros_odom["x"].append(message.pose.pose.position.x - 3)
            msgs_mavros_odom["y"].append(message.pose.pose.position.y)
            msgs_mavros_odom["z"].append(message.pose.pose.position.z)

            quat = message.pose.pose.orientation
            euler = quat2euler([quat.x, quat.y, quat.z, quat.w])
            msgs_mavros_odom["roll"].append(euler[0])
            msgs_mavros_odom["pitch"].append(euler[1])
            msgs_mavros_odom["yaw"].append(euler[2])

            msgs_mavros_odom["time"].append(time.to_sec())
        if topic == msgs_topics[3]:
            msgs_mavros_laser["range"].append(message.range)
            msgs_mavros_laser["time"].append(time.to_sec())

    bagfile.close()
    return {"ground_truth": msgs_ground_truth, "mavros_odom": msgs_mavros_odom, "mavros_laser": msgs_mavros_laser,
            "ekf2": msgs_drone_pose}


def plot_position_3d(msgs_drone_pose, msgs_ground_truth):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(msgs_ground_truth["x"], msgs_ground_truth["y"], msgs_ground_truth["z"], linewidth=1,
              color=msgs_ground_truth["color"])
    ax.plot3D(msgs_drone_pose["x"], msgs_drone_pose["y"], msgs_drone_pose["z"], linewidth=1,
              color=msgs_drone_pose["color"])


def plot_orientation_yaw(msgs_drone_pose, msgs_ground_truth, msgs_mavros_odom):
    fig, ax = plt.subplots()
    ax.plot(msgs_ground_truth["time"], msgs_ground_truth["yaw"], linewidth=0.7,
            label=msgs_ground_truth["label"])
    ax.plot(msgs_drone_pose["time"], msgs_drone_pose["yaw"], linewidth=0.7,
            label=msgs_drone_pose["label"])
    ax.plot(msgs_mavros_odom["time"], msgs_mavros_odom["yaw"], linewidth=0.7,
            color=msgs_mavros_odom["color"],
            label=msgs_mavros_odom["label"])

    ax.set_xlabel("Time")
    ax.set_ylabel("Yaw orientation")
    ax.legend()


def plot_position(msgs):
    msgs_ground_truth = msgs["ground_truth"]
    msgs_drone_pose = msgs["ekf2"]
    msgs_mavros_odom = msgs["mavros_odom"]
    msgs_mavros_laser = msgs["mavros_laser"]

    fig, axs = plt.subplots(3, 1)
    ax_pos_x = axs[0]
    ax_pos_y = axs[1]
    ax_pos_z = axs[2]

    ax_pos_x.plot(msgs_ground_truth["time"], msgs_ground_truth["x"], linewidth=0.7, label=msgs_ground_truth["label"])
    ax_pos_x.plot(msgs_drone_pose["time"], msgs_drone_pose["x"], linewidth=0.7,
                  label=msgs_drone_pose["label"])
    ax_pos_x.plot(msgs_mavros_odom["time"], msgs_mavros_odom["x"], linewidth=0.7,
                  label=msgs_mavros_odom["label"])
    ax_pos_x.set(xlim=(53.5, np.max(msgs_ground_truth["time"])))

    ax_pos_y.plot(msgs_ground_truth["time"], msgs_ground_truth["y"], linewidth=0.7, label=msgs_ground_truth["label"])
    ax_pos_y.plot(msgs_drone_pose["time"], msgs_drone_pose["y"], linewidth=0.7,
                  label=msgs_drone_pose["label"])
    ax_pos_y.plot(msgs_mavros_odom["time"], msgs_mavros_odom["y"], linewidth=0.7,
                  label=msgs_mavros_odom["label"])
    ax_pos_y.set(xlim=(53.5, np.max(msgs_ground_truth["time"])))

    ax_pos_z.plot(msgs_ground_truth["time"], msgs_ground_truth["z"], linewidth=0.7, label=msgs_ground_truth["label"])
    ax_pos_z.plot(msgs_drone_pose["time"], msgs_drone_pose["z"], linewidth=0.7,
                  label=msgs_drone_pose["label"])
    ax_pos_z.plot(msgs_mavros_odom["time"], msgs_mavros_odom["z"], linewidth=0.7,
                  label=msgs_mavros_odom["label"])
    ax_pos_z.plot(msgs_mavros_laser["time"], msgs_mavros_laser["range"], linewidth=0.7,
                  label=msgs_mavros_laser["label"])
    ax_pos_z.set(xlim=(53.5, np.max(msgs_ground_truth["time"])))

    ax_pos_x.minorticks_on()
    ax_pos_x.set_ylabel("X position")
    ax_pos_x.legend()

    ax_pos_y.minorticks_on()
    ax_pos_y.set_ylabel("Y position")
    ax_pos_y.legend()

    ax_pos_z.minorticks_on()
    ax_pos_z.set_xlabel("Time")
    ax_pos_z.set_ylabel("Z position")
    ax_pos_z.legend()


def main(bag_file_path):
    msgs = parse_bag(bag_file_path)

    plot_position(msgs)

    # plot_position_3d(msgs_drone_pose, msgs_ground_truth)

    plt.show()


if __name__ == "__main__":
    main("/home/diego/dev/ros/src/ekf_localization/rosbags/auto_2020-11-03-16-05-33.bag")
