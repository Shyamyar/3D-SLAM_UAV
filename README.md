# 3D-SLAM_UAV
3D SLAM for UAVs using EKF
The repository contains simulation of a UAV in an environment of point landmarks.
EKF_SLAM_Known is the file that runs the simulation where the position of landmarks are known. Here, EKF is used for UAV localization and is controlled to follow trajectory using LQR.
EKF_SLAM is the main file that runs the simulation where the position of landmarks are unknown. Here, EKF is used for simultaneaous localization of UAV and mapping or unknown landmarks while being controlled using LQR.

Please refer to the journal paper based on which the algorithm here is developed.
Rauniyar, S.; Bhalla, S.; Choi, D.; Kim, D. EKF-SLAM for Quadcopter Using Differential Flatness-Based LQR Control. Electronics 2023, 12, 1113. https://doi.org/10.3390/electronics12051113
