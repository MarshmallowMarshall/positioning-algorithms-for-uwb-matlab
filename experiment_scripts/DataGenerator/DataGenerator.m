classdef DataGenerator
    properties
        dt = 0.005;  % Time step in seconds
        distance = 5;  % Total distance in meters
        min_speed = 3;  % Min speed in m/s
        max_speed = 4;  % Max speed in m/s
        max_acceleration = 0.1;  % Max acceleration in m/s^2
        noise_density = 0.01;
        bias =0;
    end

    methods
    function obj = DataGenerator()
        % Constructor
    end

    function data = generate(obj)
    % Initialize
    data.positions = [];
    data.speeds = [];
    data.accelerations = [];
    data.acceleration_measurements = [];
    data.acceleration_measurement_times = [];
    data.time = [];
    data.quaternions = [];
    current_position = [0, 0, 0];  % Current position in 3D space
    current_time = 0;
    current_speed = [0, 0, 0];  % Current speed in 3D space
    current_acceleration = [0, 0, 0];  % Current acceleration in 3D space
    yaw = 0;  % Yaw angle, 0 means the robot is moving towards the positive x direction

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

        % Current acceleration in the direction of movement
        acceleration_in_direction = yq(i);

        % Decompose the acceleration in the direction of movement into x, y and z components
        current_acceleration(1:2) = acceleration_in_direction * [cosd(yaw), sind(yaw)];

        % Update current speed in x, y and z directions
        current_speed = current_speed + current_acceleration * obj.dt;

        % Calculate displacement in x, y and z directions
        displacement = current_speed * obj.dt;

        % Update current position
        current_position = current_position + displacement;

        % Convert yaw to quaternion
        q0 = cosd(yaw / 2);
        q1 = 0;
        q2 = 0;
        q3 = sind(yaw / 2);
        quaternion = [q0, q1, q2, q3];

        % Generate acceleration measurement at 100 Hz
        if mod(current_time, 0.01) < obj.dt
            acceleration_measurement = obj.generate_acceleration_measurement(norm(current_acceleration));
            data.acceleration_measurements = [data.acceleration_measurements, acceleration_measurement];
            data.acceleration_measurement_times = [data.acceleration_measurement_times, current_time];
        end

        % Store current position, time, speed, acceleration and quaternion
        data.positions = [data.positions; current_position];
        data.speeds = [data.speeds; current_speed];
        data.accelerations = [data.accelerations; current_acceleration];
        data.time = [data.time, current_time];
        data.quaternions = [data.quaternions; quaternion];
    end
end

    function acceleration_measurement = generate_acceleration_measurement(obj, true_acceleration)
        % Generate acceleration measurement
        noise = obj.noise_density / sqrt(obj.dt) * randn();  
        acceleration_measurement = true_acceleration + obj.bias + noise;
    end

    function rmse = ErrorAnalysis(obj, true_accelerations, measured_accelerations)
            % Compute RMSE of acceleration measurements
            rmse = sqrt(mean((true_accelerations - measured_accelerations).^2));
            fprintf('The RMSE of the acceleration measurements is %.6f m/s^2\n', rmse);
        end
        
    end
end
