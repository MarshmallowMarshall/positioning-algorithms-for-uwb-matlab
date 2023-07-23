classdef DataGenerator
    properties
        dt = 0.005;  % Time step in seconds
        distance = 5;  % Total distance in meters
        min_speed = 3;  % Min speed in m/s
        max_speed = 4;  % Max speed in m/s
        max_acceleration = 0.1;  % Max acceleration in m/s^2
%         noise_density = 0.03;
        accelNoise = 0.35; % measurement noise of accelerator
        accelBias  = 0.03*[1 1 1]; % drift bias of accelerator
        
        gyroNoise  = 0.015; % measurement noise of accelerator
        gyroBias   = 0.01*[1 1 1]; % drift bias of gyroscope
        ortho_error= 0.05*[1 1 1]; % the orthogonal error of gyroscope (°)

        pos_anc = [10,-45,15; 10,5,15; -15,5,15; -15,-45,15]; % Coordinates of the UWB anchor points
        range_noise_variance = 1;  % Variance of range measurement noise
    end

    methods
        function obj = DataGenerator()
            % Constructor
            addpath ..\common\
        end
    
        function [data, imu, uwb] = generate(obj)
            % Initialize
            data.quaternions = []; 
            data.angle_velocities = [];
%             data.rotation = [];
            data.positions = [];
            data.speeds = [];
            data.accelerations = [];
            data.time = [];
        
            imu = struct('accelerometer_measurements',[],'accelerometer_measurement_times',[],'speed_increment',[], ...
                'gyroscope_measurements',[],'gyroscope_measurement_times',[],'angle_increment',[]);
            uwb = struct('range_measurements',[],'range_measurement_times',[],'range_truth',[]);

        
            current_position = [0, 0, 0];  % Current position in 3D space
            current_time = 0;
            index_imu = 1;
            current_speed = [0, 0, 0];  % Current speed in 3D space
            current_acceleration = [0, 0, 0];  % Current acceleration in 3D space
            yaw = 0;  % Yaw angle, 0 means the robot is moving towards the positive x direction
            current_angle_velocity = [0, 0, 1];  % Current angle velocity in 3D space
    
            % initial quaternion base on original yaw
            q0 = cosd(yaw / 2);
            q1 = 0;
            q2 = 0;
            q3 = sind(yaw / 2);
            current_quaternion = [q0, q1, q2, q3];   
        
            % Define setpoints for acceleration in first and third stages
            setpointx = [0, 1, 3, 4, 7, 8, 10, 11];
            setpointy = [0, 0.5, 1.5, 3, 0, -1, -1, 0];
        
            % Define query points for interpolation
            xq = 0:obj.dt:15;
        
            % Perform spline interpolation
            yq = interp1(setpointx, setpointy, xq, 'spline');
    
            % Generate sensor data
            for i = 1:length(xq)
                % Current time
                current_time = xq(i);

                % Update quaternion with angle velocity
                current_quaternion = obj.update_q_(current_quaternion, current_angle_velocity);
        
                % Current acceleration in the direction of movement
                acceleration_in_direction = yq(i);
        
                % Decompose the acceleration in the direction of movement into x, y and z components
                % Calculate the rotation matrix between B frame & N frame 
                % Normalize the quaternion
                current_quaternion = current_quaternion / norm(current_quaternion);
                current_rotation   = quat2rotm(current_quaternion);
                current_acceleration = (current_rotation * [acceleration_in_direction; 0; 0])';
        
                % Update current speed in x, y and z directions
                current_speed = current_speed + current_acceleration * obj.dt;
        
                % Calculate displacement in x, y and z directions
                displacement = current_speed * obj.dt;
        
                % Update current position
                current_position = current_position + displacement;
                   
                % Generate imu measurement at 100 Hz
                if mod(current_time, 0.01) < obj.dt
                    acc_meas = obj.generate_accelerometer_measurement(current_acceleration);
                    gyro_meas= obj.generate_gyroscope_measurement(current_angle_velocity);

                    speed_increment = obj.dt * acc_meas; % Calculate speed increment
                    angle_increment = obj.dt * gyro_meas;% Calculate angle increment

                    imu(index_imu).accelerometer_measurements = acc_meas;
                    imu(index_imu).accelerometer_measurement_times = current_time;
                    imu(index_imu).speed_increment = speed_increment; % Store speed increment
                    imu(index_imu).gyroscope_measurements = gyro_meas;
                    imu(index_imu).gyroscope_measurement_times = current_time;
                    imu(index_imu).angle_increment = angle_increment;

                    index_imu = index_imu + 1;
                end

                % Generate UWB range measurement at 50 Hz
                if mod(current_time, 0.02) < obj.dt
                    [r_meas, r_truth] = obj.generate_uwb_measurement(current_position);
                    uwb(i).range_measurements = r_meas;
                    uwb(i).measurement_times = current_time;
                    uwb(i).range_truth = r_truth;
                end
        
                % Update bias
                obj.accelBias = obj.accelBias + 0.03 * obj.dt;
                obj.gyroBias  = obj.gyroBias  + 0.001* obj.dt;
        
                % Store current position, time, speed, acceleration, quaternion, and angle velocity
                data.positions = [data.positions; current_position];
                data.speeds = [data.speeds; current_speed];
                data.accelerations = [data.accelerations; current_acceleration];
                data.time = [data.time, current_time];
                data.quaternions = [data.quaternions; current_quaternion];
                data.angle_velocities = [data.angle_velocities; current_angle_velocity];
            end
        end
    
        function accelerometer_measurement = generate_accelerometer_measurement(obj, true_acceleration)
            % Generate acceleration measurement
            noise = obj.accelNoise * (2*randn(3,1)-1);
            accelerometer_measurement = true_acceleration + obj.accelBias + noise';
        end
    
        function [range_measurements, true_ranges]= generate_uwb_measurement(obj, true_position)
            % Initialize
            range_measurements = zeros(1,4);
            true_ranges = zeros(1,4);
          
            % Calculate range measurements for each anchor point
            for i = 1:size(obj.pos_anc, 1)
                true_range = norm(true_position - obj.pos_anc(i,:));  % True range
                noise = sqrt(obj.range_noise_variance) * randn;  % Measurement noise
                range_measurement = true_range + noise;  % Range measurement
                range_measurements(i) = range_measurement; % 1*4
                true_ranges(i) = true_range;
            end
        end

        function gyroscope_measurement = generate_gyroscope_measurement(obj, true_angle_velocity)
            noise = obj.gyroNoise * (2*randn(3,1)-1);
%             delK  = diag(obj.delk) - skew_sym(obj.mount_error) + down_tri(obj.ortho_error);
            delK = down_tri(obj.ortho_error);
            gyroscope_measurement = ((eye(3)-delK) \ (true_angle_velocity + obj.gyroBias)' + noise)';
        end

        function rmse = ErrorAnalysis(obj, true_accelerations, measured_accelerations) %#ok<INUSD> 
                % Compute RMSE of acceleration measurements
                rmse = sqrt(mean((true_accelerations - measured_accelerations).^2));
                fprintf('The RMSE of the acceleration measurements is %.6f m/s^2\n', rmse);
            end
        
        function quaternion = update_q_(obj, quaternion, w)
                dq0 = -0.5 * (w(1)*quaternion(2) + w(2)*quaternion(3) + w(3)*quaternion(4));
                dq1 =  0.5 * (w(1)*quaternion(1) - w(2)*quaternion(4) + w(3)*quaternion(3));
                dq2 =  0.5 * (w(1)*quaternion(4) + w(2)*quaternion(1) - w(3)*quaternion(2));
%                 dq3 = -0.5 * (w(1)*quaternion(3) - w(2)*quaternion(2) + w(3)*quaternion(1));
                dq3 = -0.5 * (w(1)*quaternion(3) - w(2)*quaternion(2) - w(3)*quaternion(1));
                quaternion = quaternion + obj.dt * [dq0, dq1, dq2, dq3];
        end
    end
end
