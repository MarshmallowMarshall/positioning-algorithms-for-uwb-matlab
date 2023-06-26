clear;close all;

uwblocation= UWB_Localization_EKF('..\exp_data\UWB_data_Ranges\range_uwb_moveleft.yaml','..\exp_data\Optitrack_yaml\vrpn_pose_moveleft.yaml');

R_ref=0.1*eye(4);
ekfobj= FilterInitialization_(uwblocation, R_ref);

uwblocation.PositionEstimate_(ekfobj);