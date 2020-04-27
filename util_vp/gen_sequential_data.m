function VP = gen_sequential_data()
    % This function is used to generate sequential odometry and measurement
    % data from the original VP dataset.
    % VP.GPS: GPS trajectory data
    % VP.Landmarks: Landmarks diameter matrix
    % Loop through VP.Data(i):
    % For VP.Data(i).type == 'odometry':
    %     VP.Data(i).trans, VP.Data(i).steer
    % For VP.Data(i).type == 'measure':
    %     VP.Data(i).sensors(j).range
    %     VP.Data(i).sensors(j).bearing
    %     VP.Data(i).sensors(j).dm
    
    %% Load Original Data
    load aa3_dr.mat;   % speed, steering, time
    load aa3_gpsx.mat; % La_m, Lo_m, timeGps
    load aa3_lsr2.mat; % z, TLsr
    load car.mat; % car parameters
    global AAr; % used for detectTreesl16.m
    AAr = [0:360]*pi/360;
           
    %% Unit Transformation
    % z measurement of timestep 7249
    z.ranges    = double(LASER)/100; % [m]
    z.time      = double(TLsr)/1000; % [s]
    % odometry/control of time step 61945
    odo.v       = speed;             % [m/s]
    odo.w       = steering;          % [rad]
    odo.time    = time/1000;         % [s]
    % GPS Measurement of groundtruth trajectory of time step 4466
    GPS.x       = Lo_m;                  % [m]
    GPS.y       = La_m;                  % [m]
    GPS.time    = timeGps/1000;          % [s]

    VP.GPS      = GPS;  % Store ground truth location
    %% Sequentialize data of motion and measurement
    t_cur       = min(z.time(1), odo.time(1)); % current time
    idx_odo     = 1; % idx of last control sequence in original odometry data
    VP.Data     = []; % sequence data contents measurements and odometry controls
    VP.Landmarks= []; % Landmarks' diameter vector dictionary
    % Loop over the laser measurement 
    for i = 1:length(z.time)
        % if is odometry data: append odometry conmmand
        while odo.time(idx_odo) < z.time(i)
            VP.Data(end+1).type     = 'odometry';
            dT      = odo.time(idx_odo) - t_cur; % Control command last for dT
            t_cur   = odo.time(idx_odo); % Update current time instance
            odo.trans   = dT*odo.v(idx_odo)/...
                          (1-tan(odo.w(idx_odo))*car.H/car.L); % car translation [m]
            odo.steer   = odo.w(idx_odo); % car stearing angle [rad]
            VP.Data(end).odometry   = odo;
            idx_odo = idx_odo + 1;
        end
        % if is sensor data: append measurements
        VP.Data(end+1).type     = 'measure';
        t_cur   = z.time(i);

        measurements            = detectTreesI16(z.ranges(i,:));
        for j = 1:size(measurements,2)
            VP.Data(end).sensors(j).range       = measurements(1,j);
            VP.Data(end).sensors(j).bearing     = measurements(2,j) - pi/2;
            % Load new landmarks or assign id to exists landmarks
            idx     = find(VP.Landmarks == measurements(3,j));
            if  isempty(idx)
                % Append new landmark diameter
                VP.Landmarks    = [VP.Landmarks, measurements(3,j)];
                VP.Data(end).sensors(j).id  = length(VP.Landmarks);
            else
                VP.Data(end).sensors(j).id  = idx;
            end            
        end
    end
end

