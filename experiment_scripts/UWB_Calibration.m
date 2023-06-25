classdef UWB_Calibration
    %UWB_CALIBRATION 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties(Constant)
%         uwb_positionstamped
%         vrpn_positionstamped
        uwb_positionstamped = extract_uwbrange_ts_yaml('..\exp_data\UWB_data_Ranges\range_uwb_moveleft.yaml');
        vrpn_positionstamped= extract_vrpn_ts_yaml('..\exp_data\Optitrack_yaml\vrpn_pose_moveleft.yaml');

        A_2d = [4.22, -0.09;
                3.72,  1.71;
                0,      1.5;
                0,        0];
    end
    methods
%         function obj = UWB_Calibration(inputArg1,inputArg2)
%         end
    end
    
    methods(Static)
        % 提取posestamped元胞数组的timestamp
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
        % 给定插值点求插值的函数
        function interpolatedPoint=Interpolation_(x,y,time,interpolationTime)
        % 已知时间和对应的二维坐标
            
            % 执行线性插值
            interpolatedX = interp1(time, x, interpolationTime, 'linear');
            interpolatedY = interp1(time, y, interpolationTime, 'linear');
            interpolatedPoint= [interpolatedX,interpolatedY];
            
        end
        function uwb_bias= CalUwbBias_()
            uwb_ts= UWB_Calibration.ExtractTimestamp_(UWB_Calibration.uwb_positionstamped);
            vrpn_ts= UWB_Calibration.ExtractTimestamp_(UWB_Calibration.vrpn_positionstamped);
            uwb_ranges=UWB_Calibration.ExtractRange_(UWB_Calibration.uwb_positionstamped);
            uwb_ranges=uwb_ranges./1000;
            squared_distances = uwb_ranges .^ 2;
            offsets = [1.575-1.219, 1.510-1.219, 1.845-1.219, 1.845-1.219];
            inner_content = squared_distances - offsets.^2;
            inner_content = max(inner_content, 0);  % 将负值替换为零
            distances2D = sqrt(inner_content);
            uwb_ranges = distances2D;

            vrpn_pos= UWB_Calibration.ExtractPosition_(UWB_Calibration.vrpn_positionstamped);
            vrpn_pos= transformV2U(vrpn_pos);
            vrpn_pos= vrpn_pos(:,1:2);

            uwb_errors= zeros(size(uwb_ranges));

            closestElements= UWB_Calibration.FindClosestElements_(UWB_Calibration.uwb_positionstamped,UWB_Calibration.vrpn_positionstamped);
   
            for k = 2:numel(uwb_ts)-20
                if ~any( uwb_ranges(k,:) ==0)
                    % 获取和当前uwb测距时间上最接近的vrpn时间戳和序号
                    closestIndex= closestElements(k);
                    % 根据它们的相对大小决定插值区间
                    if uwb_ts(k) < vrpn_ts(closestIndex)
                        time=vrpn_ts(closestIndex-1:closestIndex);
                        x=[vrpn_pos(closestIndex-1,1),vrpn_pos(closestIndex,1)];
                        y=[vrpn_pos(closestIndex-1,2),vrpn_pos(closestIndex,2)];
                        reliable_pos= UWB_Calibration.Interpolation_(x,y,time,uwb_ts(k));
                    else
                        time=vrpn_ts(closestIndex:closestIndex+1);
                        x=[vrpn_pos(closestIndex,1),vrpn_pos(closestIndex+1,1)];
                        y=[vrpn_pos(closestIndex,2),vrpn_pos(closestIndex+1,2)];
                        reliable_pos= UWB_Calibration.Interpolation_(x,y,time,uwb_ts(k));
                    end
                    % 使用vrpn计算的位置轮流和四个锚点的坐标计算更高精度的距离值，然后计算对应误差
                    for i =1:4
                        reliable_range= norm(reliable_pos-UWB_Calibration.A_2d(i,:));
                        uwb_errors(k,i)= reliable_range-uwb_ranges(k,i);
                    end
                end
            end
            uwb_bias=[mean(uwb_errors(:,1)),mean(uwb_errors(:,2)),mean(uwb_errors(:,3)),mean(uwb_errors(:,4))];
        end
        function closestElements= FindClosestElements_(positionstampedA,positionstampedB)

            uwb_ts= UWB_Calibration.ExtractTimestamp_(positionstampedA);
            vrpn_ts= UWB_Calibration.ExtractTimestamp_(positionstampedB);
            closestElements = zeros(size(uwb_ts));  % 存储最接近元素的数组
     
            for i = 1:numel(uwb_ts)
                [~, index] = min(abs(vrpn_ts - uwb_ts(i)));  % 找到差值最小的元素的索引
                closestElements(i) = index;  % 存储最接近的元素
            end
        end 
        
       
    end
end