close all; 
clear; clc;

% add directory to the path
addpath('..\');
addpath('..\helper_functions');    % add "helper_functions" to the path
addpath('..\exp_data\UWB_data_Ranges');
addpath('..\exp_data\Optitrack_yaml\')

%%%%%%%%%%%%%%%%%%%% REAL MEASUREMENT DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load the logged Data 
% getRangeUWB = importfile_Ranges('..\exp_data\UWB_data_Ranges\output_range_uwb_m2r.txt');
getRangeUWB = extract_uwbrange_yaml('..\exp_data\UWB_data_Ranges\range_uwb_moveleft.yaml');
% getRangeUWB = extract_uwbrange_yaml('exp_data\UWB_data_Ranges\range_uwb_static1.yaml');
getRangeUWB = cellfun(@(arr) arr.', getRangeUWB, 'UniformOutput', false);  % 将每个数组转置为列向量
getRangeUWB = [getRangeUWB{:}];  % 将列向量拼接为矩阵，每一列对应一个数组的元素，另外，每个元素的单位是毫米

[rowR, colR] = size(getRangeUWB);
% ts_R = getRangeUWB.ts;
% tid  = getRangeUWB.tagID;       % tag ID no.
r_t2A0 = getRangeUWB(1,:);      % tag to Anc0 measured values
r_t2A1 = getRangeUWB(2,:);      % tag to Anc1 measured values
r_t2A2 = getRangeUWB(3,:);      % tag to Anc2 measured values
r_t2A3 = getRangeUWB(4,:);      % tag to Anc3 measured values
%}

% Rescale the measured ranges into the original values in meter
r_t2A0 = r_t2A0 ./ 1000;        % the data are scaled with 1000 in the log file
r_t2A1 = r_t2A1 ./ 1000;
r_t2A2 = r_t2A2 ./ 1000;
r_t2A3 = r_t2A3 ./ 1000;

% Range values matrix. Each ranges from tag to each anchors is stored in
% the columns of the matrix 
t2A_4R = [r_t2A0' r_t2A1' r_t2A2' r_t2A3']; % use 4 ranges
% uwb_range_bias=[-0.0801 -0.0461 0.0783 0.0816];
% t2A_4R = t2A_4R- uwb_range_bias;

dimKF = 2;
% dimKF = 3;

%%%%%%%%%%% Initialization of state parameters %%%%%%%%%%%%%
% [xk, A, Pk, Q, Hkf, R] = initConstAcceleration_KF(dimKF); 
[xk, A, Pk, Q, Hkf, R] = initConstVelocity_KF(dimKF); 


disp(issymmetric(Q));
d = eig(Q);
disp(all(d>0));
disp("The eigen values of process noise (Q) are:");
disp(d);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% EXTENDED KALMAN FILTER IMPLEMENTATION USING CONTROL SYSTEM TOOLBOX
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify an initial guess for the two states
initialStateGuess = [1; 1; 0; 0];   % the state vector [x, y, vx, vy];
% initialStateGuess = xk ;

%%% Create the extended Kalman filter ekfObject
%%% Use function handles to provide the state transition and measurement functions to the ekfObject.
ekfObj = extendedKalmanFilter(@citrackStateFcn,@citrackMeasurementFcn,initialStateGuess);

% Jacobians of the state transition and measurement functions
ekfObj.StateTransitionJacobianFcn = @citrackStateJacobianFcn;
ekfObj.MeasurementJacobianFcn = @citrackMeasurementJacobianFcn;

% Measurement noise v[k] and process noise w[k]
% R_ekf = diag([0.0151 0.0151 0.0151 0.0151]);     % based on the exp data by finding var of the spread
R_ekf = diag([0.0011    0.0009    0.0003    0.0016]);   % spread of the data directly
ekfObj.MeasurementNoise = R_ekf;

% Q_ekf = diag([0.01 0.01 0.01 0.01 0.01 0.01]); % process noise regarding ranges is different from pose data 
Q_ekf = diag([0.1 0.1 0.1 0.1]);  % use precision from datasheet directly 
% Q_ekf = diag(Q);
ekfObj.ProcessNoise = Q_ekf;

[Nsteps, n] = size(t2A_4R); 
xCorrectedEKFObj = zeros(Nsteps, length(xk)); % Corrected state estimates
PCorrectedEKF = zeros(Nsteps, length(xk), length(xk)); % Corrected state estimation error covariances
figure(1);
hold on
UWB_frame;
for k=1 : Nsteps-200
    % 转成二维距离
%     t2A_4R(k, 1)=sqrt(t2A_4R(k, 1)^2-(1.575-1.219)^2)-0.0436;
%     t2A_4R(k, 2)=sqrt(t2A_4R(k, 2)^2-(1.510-1.219)^2)-0.0148;
%     t2A_4R(k, 3)=sqrt(t2A_4R(k, 3)^2-(1.845-1.219)^2)+0.1405;
%     t2A_4R(k, 4)=sqrt(t2A_4R(k, 4)^2-(1.845-1.219)^2)+0.1423;
    t2A_4R(k, 1)=sqrt(t2A_4R(k, 1)^2-(1.575-1.219)^2);
    t2A_4R(k, 2)=sqrt(t2A_4R(k, 2)^2-(1.510-1.219)^2);
    t2A_4R(k, 3)=sqrt(t2A_4R(k, 3)^2-(1.845-1.219)^2);
    t2A_4R(k, 4)=sqrt(t2A_4R(k, 4)^2-(1.845-1.219)^2);

    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.    
%     [xCorrectedekfObj(k,:), PCorrected(k,:,:)] = correct(ekfObj,yMeas(:, k)); 
    [xCorrectedEKFObj(k,:), PCorrectedEKF(k,:,:)] = correct(ekfObj,t2A_4R(k, :)); 
    
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    predict(ekfObj);

%     plot(xCorrectedEKFObj(k,1), xCorrectedEKFObj(k,2),'bo');
%     covar = [PCorrectedEKF(k,1:2,1);PCorrectedEKF(k,1:2,2)];
%     error_ellipse(covar,[xCorrectedEKFObj(k,1), xCorrectedEKFObj(k,2)]);
      plot(xCorrectedEKFObj(k,1), xCorrectedEKFObj(k,2),'bo');
%     if k>4 && mod(k,4)==0
%         plot([xCorrectedEKFObj(k,1),xCorrectedEKFObj(k-4,1)],[xCorrectedEKFObj(k,2),xCorrectedEKFObj(k-4,2)],'b')
%     end
end

vrpn_pose= extract_vrpn_yaml('..\exp_data\Optitrack_yaml\vrpn_pose_moveleft.yaml');
for k =300:length(vrpn_pose(:,1))
    plot(-vrpn_pose(k,1)+4.7449,-vrpn_pose(k,2)+0.0945,'ro');
end

hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% PLOTTING THE RESULTS SECTION
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Using Vicon camera as reference 
% figure
% hold on
% plot(xCorrectedEKFObj(:,1), xCorrectedEKFObj(:,2), 'LineWidth', 1.5);
% plot(xCorrectedEKFObj(length(xCorrectedEKFObj),1), xCorrectedEKFObj(length(xCorrectedEKFObj),2),'ro');
% grid on;