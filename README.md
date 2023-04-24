# 3D-SLAM_UAV
3D SLAM for UAVs using EKF
The repository contains simulation of a UAV in an environment of point landmarks.
3D_Known is the main file that runs the simulation where the position of landmarks are known. Here, EKF is used for UAV localization and is controlled to follow trajectory using LQR.
3D_Unknown is the main file that runs the simulation where the position of landmarks are unknown. Here, EKF is used for simultaneaous localization of UAV and mapping or unknown landmarks while being controlled using LQR.
