classdef UWB_ECL_EKF
    %UWB_ECL_EKF 参考ECL-EKF算法对UWB/IMU实现导航
    %   使用离线数据对融合算法进行调试
    %   参考：https://www.yuque.com/mikeyleee/gm2fhm/xgkxi9l5n6xrsct3
    %   整体使用我最熟悉的先验后验下标
    % TODO:尽量减少全局变量的使用，除了Output结构体，其他属性都应该是只读的
    % TODO:每次运算之前都做一次维数检测
    %       
    
    properties
        uwb_data                % 2023-07-20 11:15:37 放弃结构体的写法，改用结构体数组，这样索引和管理都会比较方便
        imu_data        
        vrpn            struct
        ground_truth    struct
        P               struct  % TODO:将State和P都写成结构体数组
        State           struct
        param           struct
        pos_anc
        gravity = 9.80665

        Output          struct
    end
    
    methods
        function obj = UWB_ECL_EKF(dg)
            %UWB_ECL_EKF 构造此类的实例 
            %   使用仿真数据运行ECL，其中dg是DataGenerator类
            %   dg会提供整个运行过程的真值、各个传感器的各种数据，分别装在对应的结构体数组当中
            % TODO:根据imu_data和uwb_data的新结构修改初始化函数的赋值部分，或者直接修改dg的输出
            [gt, imu_meas] = dg.generate();
            obj.ground_truth= gt;
            obj.imu_data= imu_meas;
            obj.uwb_data= uwb_meas;
            obj.State= [GetInitialState_()];
            obj.P.pri= [GetInitialCovariance()];
            obj.P.post=[];
            
            % 调用各种初始化函数
            SetParameter_();
        end

        function Initial_State = GetInitialState_(obj) %#ok<MANU> 
            %GetInitialState_ 从参考数据当中较准确地估计滤波器的初始状态值
            %   之后可能会改成使用高斯牛顿法
            %   当前就是凭感觉定的值
            init_quat=zeros(1,4);
            init_vel =zeros(1,4);
            init_pos =zeros(1,4);
            init_delAngb=0.01;
            init_delVelb=0.01;

            Initial_State = [init_quat;init_vel;init_pos;init_delAngb;init_delVelb];
        end

        function Initial_P =GetInitialCovariance_(obj) %#ok<MANU> 
            % 仿真初始猜测误差不会超过1e-2米，所以P0的对角线元素应为1e-4
            sigma_quat= 0.1 * [1;1;1;1]; % 反映的是param.alignment.quatErr
            velErrNE = 5.0; % Initial 1SD velocity error when aligning without GPS. (m/sec)
            velErrD = 1.0; % Initial 1SD vertical velocity error when aligning without GPS. (m/sec)
            sigma_vel=  [velErrNE;velErrNE;velErrD];
            posErrNE= 1;
            hgtErr  = 1;
            sigma_pos=  [posErrNE;posErrNE;hgtErr];
            sigma_dAngBias = 0.05*pi/180*dt*[1;1;1]; % param.alignment.delAngBiasErr
            sigma_dVelBias = 0.07*dt*[1;1;1]; % param.alignment.delVelBiasErr
            Initial_P= diag([sigma_quat;sigma_vel;sigma_pos;sigma_dAngBias;sigma_dVelBias].^2);
        end

        function RunFilter_(obj)
            %RunFilter_ 控制滤波器的运行
            %TODO 设计一个机制避免断层的情况
            %
            % 放弃了for time的写法，改用for index的写法
            indexStart= 1;
            indexStop=  length(obj.imu_data);
            
            for index=indexStart:indexStop

                dt= obj.imu_data(index).dt;

                % 1.状态预测
                % 其中对于增量，先简单地使用generator生成的测量值，之后再根据需要调整
%                 [correctedDelVel,correctedDelAng]= PredictStates_(obj.State,obj.imu_data.devAng(index),obj.imu_data.delVel(index));
                % 2023-07-19 12:22:23 给状态值添加了下标
                [obj.State.pri(end+1),correctedDelVel,correctedDelAng] = PredictStates_(obj.State.post(end),obj.imu_data(index).devAng,obj.imu_data(index).delVel);
        
                % 2.约束状态
                obj.State.pri(end) = ConstrainStates_(obj.State.pri(end),dt);

                % 3.预测协方差
                % ToDo：输入参数里的F是哪来的
                PredictCovariance_(correctedDelVel,correctedDelAng,dt);

                % 检查UWB量测是否到达
                % TODO:如何避免重复取样 ———— done
                latest_uwb_index = find(([obj.uwb_data.time] - obj.param.fusion.uwbTimeDelay) < obj.imu_data(index).time, 1, 'last' );
                % 为了避免声明新的变量，使用了这样的写法
                if index > 1 && lastest_uwb_index == find(([obj.uwb_data.time] - obj.param.fusion.uwbTimeDelay) < obj.imu_data(index-1).time, 1, 'last' )
                    latest_uwb_index = [];
                end
                if ~isempty(latest_uwb_index)
                    FuseUwbData_(latest_uwb_index);
                else
                    % TODO:没有量测到达要配置后验估计
                end

                % TODO:输出估计数据
            end        
        end
       
        function [states,correctedDelVel,correctedDelAng]= PredictStates_(obj,states,delAng,delVel)
            %PredictStates_ 
            %   使用IMU测量计算状态增量
            % ToDo：对输入的states做维度检测

            % 1.初始化
            % 虽然说尽量不要直接使用obj.，但这里可以保留，因为以后有可能会在别的地方动态调整dt的值
            dt= obj.imu_data.dt;

            % 2.移除增量中的偏差
            delAng= delAng - states(11:13);
            delVel= delVel - states(14:16);

            % 3.对速度增量进行 rotation and skulling 校正
            correctedDelVel= delVel;
            % 4.对角增量进行 coning errors and earth spin rate 校正
            correctedDelAng= delAng;
            % 5.保存当前测量

            % 6.用RotToQuat函数将旋转矢量转化为四元数（角增量就是旋转矢量）
            delQuat= RotToQuat(correctedDelAng);

            % 7.状态更新之四元数——四元数乘法+四元数归一化
            states(1:4)= QuatMult(states(1:4),delQuat);
            states(1:4)= NormQuat(states(1:4));

            % 8.使用更新后的四元数计算旋转矩阵
            Tbn= Quat2Tbn(states(1:4));

            % 9.使用旋转矩阵计算N系下的速度增量，并且更新
            delVelNframe = Tbn * correctedDelVel + [0;0;obj.gravity]*dt;
            states(5:7) = states(5:7) + delVelNframe(1:3);

            % 10.使用梯形积分法更新位置
            states(8:10) = states(8:10) + 0.5 * dt * (prevVel + states(5:7));
        end

        function PredictCovariance_(obj,delVel,delAng,dt)
            % 这一步对状态估计的协方差矩阵P做更新
           
            % 1.初始化
            P_= obj.P.post(end); 
            q=  obj.State.pri(1:4);
            dAngBiasSigma = dt*dt*obj.param.prediction.dAngBiasPnoise;  % ←根据量纲的相对关系可以得到这个式子
            dVelBiasSigma = dt*dt*obj.param.prediction.dVelBiasPnoise;
            % 2.计算雅可比矩阵
            F= StateTransitionJacobian_(q,delAng,delVel,dt);
            % TODO:配置delAngVar
            delAngVar = (dt*obj.param.prediction.angRateNoise)^2;       % ←同样通过量纲推出
            delVelVar = (dt*obj.param.prediction.accelNoise)^2;
            Q = CalQ_(q,delAngVar,delVelVar);
            % 3.更新IMU量测噪声contribute的部分
            % TODO:维数检测
            P_ = F*P_*F'+ Q;
            % 4.计算过程噪声
            processNoiseVariance = [zeros(1,10), dAngBiasSigma*[1 1 1], dVelBiasSigma*[1 1 1]].^2;
            for i=1:16
                P_(i,i)= P_(i,i)+ processNoiseVariance(i);
            end
            % 5.强制对称
            P_ = 0.5*(P_ + P_');
            % 6.保证半正定
            for i=1:24
                if P_(i,i) < 0
                    P_(i,i) = 0;
                end
            end
            obj.P.pri(end+1)= P_; %#ok<*MCVM> 
        end
        
        function FuseUwbData_(obj, uwb_index)  
            % 将uwb数据和imu进行融合，完成卡尔曼滤波的测量更新步骤
            % TODO: 设置一个有效测量检验机制
            % TODO: Apply an innovation consistency check
            % 1.初始化
            pos_Anc = obj.pos_anc; % 存放当前参与测距的锚点的三维坐标 m*3
%             pos_pri = obj.State(8:10); % 当前时刻对位置的先验估计     3*1
            state = obj.State.pri(end); % 当前时刻的先验估计     16*1
            ranges_meas= obj.uwb_data(uwb_index).ranges; % 准备融合的uwb数据 m*1
            [nAnc, ~]= size(ranges_meas);
            R = obj.uwb_data(uwb_index).noise*eyes(nAnc);  % 测距量测噪声 m*m
            P_= obj.P.pri(end); % 16*16
            H = zeros(nAnc,16); % m*16

            % 2.计算预测量测y_pri和新息uwb_innov
            y_pri =     vecnorm(state(8:10) - pos_Anc, 2, 2); % m*1
            uwb_innov = ranges_meas - y_pri; % m*1

            % 3.计算观测Jacobian矩阵
            H(1:nAnc,8:10) = (state(8:10).' - pos_Anc) ./ y_pri; % m*3
            
            % 4.计算卡尔曼增益 K=PH'(HPH'+R)^-1
            K= P_*H'/(H*P_*H'+R); % 16*m

            % 5.对位置状态和协方差进行correct
            % TODO:决定P的处置方法 ———— 拓展H矩阵
            state_post  = state + K * uwb_innov; % 16*1
            P_post      = (eyes(16) - K*H) * P_; % 16*16

            % 6.保证对称和半正定
            P_post = 0.5*(P_post + P_post');
            for i=1:16
                if P_post(i,i) < 0
                    P_post(i,i) = 0;
                end
            end
            % 上面的写法只能保证对角线的元素非负，下面这个元素是针对全体元素的，然而这种方法也不能够保证半正定性
%             index_z =find(P_post < 0);
%             P_post(index_z) = 0;

            % 7.保存计算结果
            obj.State.post(end + 1)= state_post;
            obj.P.post(end + 1)= P_post; %#ok<*MCVM> 
            
%             所涉及矩阵的维数
%             H =     zeros(nAnc, 3);
%             y_pri = zeros(nAnc,1);
%             uwb_innov=zeros(nAnc,1);
            
        end
    
        function SetParameter_(obj)
            obj.param.fusion.uwbTimeDelay= 0;
            % 噪声项:用一倍标准差（1SD）表示
            obj.param.prediction.dVelBiasPnoise = 0.03;  % 这一项应该能够反映加速度计的constant偏差的随机变化程度，对应到实际的参数就是加速度计偏差的变化率(m/sec^3)乘上一个dt
            obj.param.prediction.dAngBiasPnoise = 0.001; % 类似的，这项应该是陀螺仪偏差的变化率(rad/sec^2)
            obj.param.prediction.angRateNoise   = 0.015;  % IMU的陀螺仪的角速度测量噪声(rad/sec)
            obj.param.prediction.accelNoise     = 0.35;     % IMU的加速度计的测量噪声(m/sec^2) 
        end
    end


    methods(Static)
        function F= StateTransitionJacobian_(q,delAng,delVel,dt)
            %StateTransitionJacobian_ 构造状态转移方程
            
            F_qq= calculate_Fqq_(delAng);
            F_vq= calculate_Fvq_(q, delVel);
            F_vp= calculate_Fvp_(dt);
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
        end
        function Q= CalQ_(q,delAngVar,delVelVar)
        % Input:
        %   q是四元数，4*1
        %   delAngVar和delVelVar分别表示角增量和速度增量三轴的量测噪声方差，3*1
        % Output:
        %   Q:预测阶段中，IMU量测噪声contribute的过程噪声协方差阵GRG'，16*16
        %
            % 计算控制影响矩阵G
            Gqta= 0.5 * Xi_(q);
            Gvtv= quat2rotm(q);
            G= [Gqta,       zeros(4,3);
                zeros(3,3), Gvtv      ]; % 7*6
            
            M= [delAngVar*[1;1;1];delVelVar*[1;1;1]]; % 扩充成三维向量，然后拼接在一起
            R_imu= diag(M); % 构造对角阵，组成imu的协方差阵
            
            % 由此得到imu增量测量误差contribute的过程噪声协方差阵，并且将它拓展到和P匹配的16维
            Q_=  G*R_imu*G';
            Q= zeros(16,16);
            Q(1:size(Q_,1), 1:size(Q_,2)) = Q_;  
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
        function [states]  = ConstrainStates_(states,dt_imu_avg)
            % constrain gyro bias states
            limit = 5.0*pi/180*dt_imu_avg;
            for i=11:13
                if (states(i) > limit)
                    states(i) = limit;
                elseif (states(i) < -limit)
                    states(i) = -limit;
                end
            end
            
            % constrain accel bias states
            limit = 0.5*dt_imu_avg;
            for i=14:16
                if (states(i) > limit)
                    states(i) = limit;
                elseif (states(i) < -limit)
                    states(i) = -limit;
                end
            end
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