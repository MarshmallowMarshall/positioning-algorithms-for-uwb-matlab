classdef UWB_ECL_EKF
    %UWB_ECL_EKF 参考ECL-EKF算法对UWB/IMU实现导航
    %   使用离线数据对融合算法进行调试
    %   参考：https://www.yuque.com/mikeyleee/gm2fhm/xgkxi9l5n6xrsct3
    
    properties
        uwb     struct
        imu     struct
        vrpn    struct
        ground_truth struct
        pos_anc
    end
    
    methods
        function obj = UWB_ECL_EKF(inputArg1,inputArg2)
            %UWB_ECL_EKF 构造此类的实例
            %   读取离线数据
            obj.Property1 = inputArg1 + inputArg2;
            obj.State=[];
        end
        function obj = UWB_ECL_EKF(dg)
            %UWB_ECL_EKF 构造此类的实例 
            %   使用仿真数据运行ECL，其中dg是DataGenerator类
            [positions, speeds, true_accelerations, acceleration_measurement_times, measured_accelerations, time] = dg.generate();
            obj.ground_truth.acc= true_accelerations;
            obj.ground_truth.pos= positions;
            obj.ground_truth.spe= speeds;
            obj.ground_truth.time=time;
            obj.imu.acc_time= acceleration_measurement_times;
            obj.imu.acc= measured_accelerations;
            obj.imu.dt= 0.01;
        end
        function ekfObj = FilterInitialization_(obj)
            %FilterInitialization_ 构造滤波器对象
            %   读取离线数据
            %   构造函数
            %   obj = extendedKalmanFilter(stateTransitionFcn,measurementFcn,initialState)
            %   obj = extendedKalmanFilter(___,Name,Value)
            ekfObj= extendedKalmanFilter(@UWB_ECL_EKF.StateTransitionFcn_ECL_, @MeasurementFcn_ECL_);
            ekfObj.MeasurementJacobianFcn= @UWB_ECL_EKF.MeasurementJacobianFcn_ECL_;
            %   这里如果不声明F的雅可比矩阵会出错吗？
        end

        function Initial_State = GetInitialState_(obj,inputArg)
            %GetInitialState_ 从参考数据当中较准确地估计滤波器的初始状态值
            %   之后可能会改成使用高斯牛顿法
            Initial_State = obj.Property1 + inputArg;
        end

        function PredictStates_(obj)
            %PredictStates_ 
            %   使用IMU测量计算状态增量
        end

        function RunFilter_(obj, ekfobj)
            %RunFilter_ 控制滤波器的运行
            %ToDo 设计一个机制避免断层的情况
            imuBuffer = fifoQueue(100);
            %没有为UWB设计FIFO缓存器
            accTime = obj.imu.acc_time;
            accData = obj.imu.acc;
            timeEnd = obj.ground_truth.time(end);
        
            for time = 0:0.005:timeEnd
                current_imu_index = find(accTime - time < 0.005, 1);
                if ~isempty(current_imu_index)
                    current_imu = accData(current_imu_index);
                    imuBuffer.enqueue(current_imu);
                end
                
            end
        
            obj.imu.buffer = imuBuffer; % update obj.imu.buffer with the modified imuBuffer
        end


    end


    methods(Static)
        function x= StateTransitionFcn_ECL_(x,a,w)
            %StateTransitionFcn_ECL_ 构造状态转移方程
            
            delat_theta_meas
            
            %ToDo: 具体化q,delta_t,delta_theta_meas和delta_theta_bias的来历
            delta_theta_truth= delta_theta_meas- delta_theta_bias;
            F_qq= calculate_Fqq_(delta_theta_truth);
            F_vq= calculate_Fvq_(q, delta_V_truth);
            F_vp= calculate_Fvp_(delta_t);
            F_qtb=calculate_Fqtb_(q);
            F_vab=-1*quat2rotm(q);

            % 得到标准的含磁场状态和风速的原版雅可比矩阵
            F_full = [F_qq, zeros(4,3), zeros(4,3), F_qtb, zeros(4,3), zeros(4,3), zeros(4,3), zeros(4,2);
                 F_vq, eye(3), zeros(3,3), zeros(3,3), F_vab, zeros(3,3), zeros(3,3), zeros(3,2);
                 zeros(3,4), F_vp, eye(3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,2);
                 zeros(3,4), zeros(3,3), zeros(3,3), eye(3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,2);
                 zeros(3,4), zeros(3,3), zeros(3,3), zeros(3,3), eye(3), zeros(3,3), zeros(3,3), zeros(3,2);
                 zeros(3,4), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), eye(3), zeros(3,3), zeros(3,2);
                 zeros(3,4), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), eye(3), zeros(3,2);
                 zeros(2,4), zeros(2,3), zeros(2,3), zeros(2,3), zeros(2,3), zeros(2,3), zeros(2,3), eye(2,2)];
            
            % 但是在这里不需要后三个状态量,24-(3+3+2)=16
            F = F_full(1:16,1:16);
            x= F*x;
        end
        function yk= MeasurementFcn_ECL_(xk)
            
        end
        function dhdx = MeasurementJacobianFcn_ECL_(xk)
            
        end
        
        function F_qq =  calculate_Fqq_(delta_theta_truth)
            % 输入: delta_theta_truth 是一个 3x1 的列向量
            % 输出: F 是一个 4x4 的矩阵
        
            % 计算 θ 矩阵
            theta = delta_theta_truth / 2;
        
            % 计算 M 矩阵
            M = [0 -theta(1) -theta(2) -theta(3);
                 theta(1) 0 theta(3) -theta(2);
                 theta(2) -theta(3) 0 theta(1);
                 theta(3) theta(2) -theta(1) 0];
        
            % 计算 F 矩阵
            F_qq = eye(4) + M;
        end
        function F_vq =  calculate_Fvq_(q, delta_V_truth)
            % 输入: delta_V_truth 是一个 3x1 的列向量
            %       q 是一个 4x1 的列向量 (quaternion)
            % 输出: F_vq 是一个 3x4 的矩阵
        
            % 计算 Φ 矩阵
            Phi_1 = [q(1) -q(4) q(3);
                     q(2) q(3) q(4);
                    -q(3) q(2) q(1);
                    -q(4) -q(1) q(2)];
        
            Phi_2 = [q(4) q(1) -q(2);
                     q(3) -q(2) -q(1);
                     q(2) q(3) q(4);
                     q(1) -q(4) q(3)];
        
            Phi_3 = [-q(3) q(2) q(1);
                     q(4) q(1) -q(2);
                    -q(1) q(4) -q(3);
                     q(2) q(3) q(4)];
        
            % 计算 Fvq 矩阵
            F_vq = 2 * [Phi_1 * delta_V_truth; 
                        Phi_2 * delta_V_truth; 
                        Phi_3 * delta_V_truth];
        end
        function F_vp =  calculate_Fvp_(delta_t)
            % 输入: delta_t 是时间间隔
            % 输出: F_vp 是一个 3x3 矩阵
        
            F_vp = eye(3) * delta_t; 
        end
        function F_qtb = calculate_Fqtb_(q)
            % 输入: q 是一个四元数，形式为一个 4x1 列向量
            % 输出: F_qtb 是一个 4x3 矩阵
        
            F_qtb = -0.5 * Xi(q);
        end
%         function Q    = 
%         
%         end

        function res = Xi_(q)
            % 输入: q 是一个四元数，形式为一个 4x1 列向量
            % 输出: res 是一个 4x3 矩阵
            
            q0 = q(1);
            q_vec = q(2:4);
        
            res = [-q_vec.'; q0 * eye(3) + skew_symmetric(q_vec)];
        end
        function res = skew_symmetric_(v)
            % 输入: v 是一个 3x1 列向量
            % 输出: res 是一个 3x3 矩阵
            
            res = [0, -v(3), v(2); 
                   v(3), 0, -v(1); 
                  -v(2), v(1), 0];
        end
        
        function closest_index = Measurment_match_(vector, num, threshold)
            % vector 是由递增元素组成的向量
            % num 是要查找的数字
            
            % 找到第一个大于或等于 num 的元素的索引
            index = find(vector >= num, 1);
            
            % 判断 num 是否等于这个元素
            if vector(index) == num
              interval = [vector(index), vector(index)];
              closest_index= index;
            else
              interval = [vector(index - 1), vector(index)];
              closest_index= find(abs(interval-num) < threshold);
            end  
        end
    end
end