classdef UWB_Localization_EKF
    % 该类的目的是以对象的方式管理一个UWB二维定位过程
    %   Todo List
    %   1 能够绘制当前实验环境的平面图，以及相应的锚点位置
    %   2 输入指定yaml文件，能够读取其中的位置信息和时间戳信息
    
    properties
        uwb_ranges % 二维测距
        vrpn_pos
        uwb_ts
        vrpn_ts
    end
    properties(Constant)
        % basic parameter
        A_2d = [4.22, -0.09; 3.72,  1.71; 0,  1.5; 0,  0];
        delta_t= 0.08; % 单位为秒
    end
    
    methods
        function obj = UWB_Localization_EKF(uwbyamlfile,vrpnyamlfile)
            
            % find the necessary file
            addpath('..\');
            addpath('..\helper_functions');
            addpath('..\exp_data\UWB_data_Ranges');
            addpath('..\exp_data\Optitrack_yaml')
            
            % Load the logged Data 
            % e.g. uwb_positionstamped = extract_uwbrange_ts_yaml('..\exp_data\UWB_data_Ranges\range_uwb_moveleft.yaml');
            uwb_rangestamped = extract_uwbrange_ts_yaml(uwbyamlfile);
            vrpn_positionstamped= extract_vrpn_ts_yaml(vrpnyamlfile);
            
            % 提取指定信息
            % 先是二维测距信息
            obj.uwb_ranges=UWB_Localization_EKF.ExtractRange_(uwb_rangestamped);
            obj.uwb_ranges=obj.uwb_ranges./1000;
            squared_distances= obj.uwb_ranges .^ 2;
            offsets= [1.575-1.219, 1.510-1.219, 1.845-1.219, 1.845-1.219];
            inner_content = squared_distances - offsets.^2;
            inner_content = max(inner_content, 0);  % 将负值替换为零
            distances2D = sqrt(inner_content);
%             obj.uwb_ranges = distances2D(50:end,:);
            obj.uwb_ranges = distances2D;
            % 然后是作为参考的vrpn估计位置
            obj.vrpn_pos= UWB_Localization_EKF.ExtractPosition_(vrpn_positionstamped);
            obj.vrpn_pos= transformV2U(obj.vrpn_pos);
            obj.vrpn_pos= obj.vrpn_pos(:,1:2);
            % 提取测量时间戳，但暂时没用到就先注释了
            obj.uwb_ts= UWB_Localization_EKF.ExtractTimestamp_(uwb_rangestamped);
            obj.vrpn_ts= UWB_Localization_EKF.ExtractTimestamp_(vrpn_positionstamped);
            
        end
        
        function ekfObj = FilterInitialization_(obj, R_ekf)
            arguments
                obj
                R_ekf = diag([0.0011    0.0009    0.0003    0.0016])
            end
            initialStateGuess = [3.7; 0.9; 0; 0];

%             ekfObj= extendedKalmanFilter(@obj.StateTransitionFcn_CV_, @obj.MeasurementFcn_CV_, initialStateGuess);
%             ekfObj.StateTransitionJacobianFcn = @obj.StateJacobianFcn_CV_;
%             ekfObj.MeasurementJacobianFcn = @obj.MeasurementJacobianFcn_CV_;

%             ekfObj= extendedKalmanFilter(@citrackStateFcn,@citrackMeasurementFcn,initialStateGuess);
%             ekfObj.StateTransitionJacobianFcn = @citrackStateJacobianFcn;
%             ekfObj.MeasurementJacobianFcn = @citrackMeasurementJacobianFcn;

            ekfObj= extendedKalmanFilter(@UWB_Localization_EKF.StateTransitionFcn_CV_, @UWB_Localization_EKF.MeasurementFcn_CV_, initialStateGuess);
            ekfObj.StateTransitionJacobianFcn = @UWB_Localization_EKF.StateJacobianFcn_CV_;
            ekfObj.MeasurementJacobianFcn = @UWB_Localization_EKF.MeasurementJacobianFcn_CV_;

%             ekfObj= extendedKalmanFilter(@StateTransitionFcn_CV_, @MeasurementFcn_CV_, initialStateGuess);
%             ekfObj.StateTransitionJacobianFcn = @StateJacobianFcn_CV_;
%             ekfObj.MeasurementJacobianFcn = @MeasurementJacobianFcn_CV_;
                    
            ekfObj.MeasurementNoise = R_ekf;
            Q_ekf = diag([0.1 0.1 0.1 0.1]);
            ekfObj.ProcessNoise = Q_ekf;
        end

        function PositionEstimate_(obj,ekfObj)
            
            [Nsteps, n] = size(obj.uwb_ranges); 
            xCorrectedEKFObj = zeros(Nsteps, length(ekfObj.State));          % posteriori state estimates
            PCorrectedEKF = zeros(Nsteps, length(ekfObj.State), length(ekfObj.State)); % posteriori state estimation error covariances
            figure(1);hold on
%             UWB_Localization_EKF.Draw_Uframe;
            UWB_frame;
            for k=1 : Nsteps-100
                [xCorrectedEKFObj(k,:), PCorrectedEKF(k,:,:)] = correct(ekfObj,obj.uwb_ranges(k, :)); 
                predict(ekfObj);
                plot(xCorrectedEKFObj(k,1), xCorrectedEKFObj(k,2),'bo');
            end
            obj.PlotVrpnTrack_;
            hold off
            PositionError= CalPositionError_(obj,xCorrectedEKFObj(:,1:2),obj.vrpn_pos);
            disp(['定位误差（RMSE）为 ', num2str(PositionError),'米']);
%             if isequal(xCorrectedEKFObj,obj.vrpn_pos)
%                 PositionError= CalPositionError_(obj,xCorrectedEKFObj,obj.vrpn_pos);
%                 disp(['定位误差（RMSE）为 ', num2str(PositionError),'米']);
%             else
%                 fprintf('错误：维数不相等!，前者的维数为%d，后者的维数为%d\n', size(xCorrectedEKFObj),size(obj.vrpn_pos));
%             end
        end

        function PositionRMSE=CalPositionError_(obj,UWBPosition,VrpnPosition)
            ClosestIndex= FindClosestElements_(obj);
            ComparePosition= VrpnPosition(ClosestIndex,:);
            if isequal(size(UWBPosition),size(ComparePosition))
                PositionRMSE= rmse(UWBPosition, ComparePosition);
            else
%                 fprintf('错误：维数不相等!，前者的维数为%d，后者的维数为%d\n', size(xCorrectedEKFObj),size(obj.vrpn_pos));
                size(UWBPosition)
                size(ComparePosition)
            end
        end

        function PlotVrpnTrack_(obj)
            [nSamples,nDim]= size(obj.vrpn_pos);
            for kk= 1:nSamples
                plot(obj.vrpn_pos(kk,1),obj.vrpn_pos(kk,2),'go');
            end
        end
        function closestElements= FindClosestElements_(obj)
            
            closestElements = zeros(size(obj.uwb_ts));  % 存储最接近元素的数组
     
            for i = 1:numel(obj.uwb_ts)
                [~, index] = min(abs(obj.vrpn_ts - obj.uwb_ts(i)));  % 找到差值最小的元素的索引
                closestElements(i) = index;  % 存储最接近的元素
            end
        end 
    end

    methods(Static) % 在这里放了一些辅助函数，调用的方式是UWB_Localization_EKF.fcn()

%         function Draw_Uframe(obj)
%             figure(1);
% %             hold on
%             [nAnc, ~] = size(obj.A_2d);
%             for k = 1:nAnc
%                 plot(obj.A_2d(k,1),obj.A_2d(k,2),'ro');
%             end
%             grid on;
%             axis([-0.5,5,-1,2]);axis equal;
%             set(gca,'XTick',-0.5:0.5:5);
%             set(gca,'YTick',-1:0.5:4);
%         end

        % 将V系的坐标转换为U系的坐标（z轴数据无处理，因此只在二维上有效）
        function p_fU= TransformV2U(p_fV)
        %将V系的坐标变换到U系
        % 数据来源于UWB_EKF_2D_static实验的结果
            p_fU=[-p_fV(:,1)+4.7449,-p_fV(:,2)+0.0945,p_fV(:,3)];
        end
        % 提取positionstamped元胞数组的timestamp
        function timestamps= ExtractTimestamp_(positionstamped)

            % 初始化一个空数组来存储提取的 timestamp_sec
            timestamps = [];
            
            % 遍历 positionstamped 元胞数组
            for i = 1:numel(positionstamped)
                % 获取当前结构体的 timestamp_sec
                timestamp = positionstamped{i}.timestamp_sec;
                
                % 将 timestamp 添加到 timestamps 数组
                timestamps = [timestamps, timestamp];
            end
        end
        % 提取positionstamped元胞数组的range
        function ranges= ExtractRange_(positionstamped)
            
            % 初始化一个空数组来存储提取的 range
            ranges = [];

            % 遍历 positionstamped 元胞数组
            for i = 1:numel(positionstamped)
                % 获取当前结构体的 timestamp_sec
                range = positionstamped{i}.range_raw(1:4);
                
                % 将 timestamp 添加到 timestamps 数组
                ranges = [ranges;range];
            end
        end
        % 提取positionstamped元胞数组的position
        function positions= ExtractPosition_(positionstamped)
            
            % 初始化一个空数组来存储提取的 range
            positions = [];

            % 遍历 positionstamped 元胞数组
            for i = 1:numel(positionstamped)
                % 获取当前结构体的 timestamp_sec
                position = positionstamped{i}.vrpn_position;
                
                % 将 timestamp 添加到 timestamps 数组
                positions = [positions;position];
            end
        end
    end
    
    % EKF对象所用到的各种函数分开放在这里
    methods(Static)
        function x = StateTransitionFcn_CV_(x)
           dt =UWB_Localization_EKF.delta_t;
           A = [1  0   dt  0;
                0  1   0   dt;
                0  0   1   0;
                0  0   0   1];
           x = A * x;
        end
        function yk= MeasurementFcn_CV_(xk)
            Anc_2D= UWB_Localization_EKF.A_2d;
            [nAnc, nDim] = size(Anc_2D);
            yk = zeros(nAnc, 1);
            for jj = 1 : nAnc
                yk(jj) = sqrt((Anc_2D(jj, 1) - xk(1)).^2 + (Anc_2D(jj, 2) - xk(2)).^2);    
            end
        end
        function dfdx= StateJacobianFcn_CV_(xk)
            dt= UWB_Localization_EKF.delta_t;
            dfdx = [1  0   dt  0;
                    0  1   0   dt;
                    0  0   1   0;
                    0  0   0   1];
        end
        function dhdx = MeasurementJacobianFcn_CV_(xk)
            Anc_2D= UWB_Localization_EKF.A_2d;
            [nAnc, nDim] = size(Anc_2D);
            state_vec_len = length(xk);
            dhdx = zeros(nAnc, state_vec_len);
            ri_0 = zeros(nAnc, 1);
            for jj = 1 : nAnc                
                ri_0(jj) = sqrt((Anc_2D(jj, 1) - xk(1)).^2 + (Anc_2D(jj, 2) - xk(2)).^2);
                dhdx(jj, 1) = (xk(1) - Anc_2D(jj, 1))./ ri_0(jj);
                dhdx(jj, 2) = (xk(2) - Anc_2D(jj, 2))./ ri_0(jj);
                dhdx(jj, 3) = 0;    % no velocity measurement data
                dhdx(jj, 4) = 0;
            end
        end
    end
end
