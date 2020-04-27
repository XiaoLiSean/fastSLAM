function sensor = gen_measurements(range_lim, bearing_lim, landmarks)
    %% Load Map and Odometry to re-gain new measurements under new defined range and bearing limits
    % Read sensor readings, i.e. odometry and range-bearing sensor
    sensor      = read_data('./data/sensor_data.dat');
    
    %% Initialize measurement noise (which is identical to SLAM sys initialization in run.m)
    betas   = [0.1 2 deg2rad(5)];
    sys     = system_initialization(zeros(1,6), betas); % motion model is not used here initialized with zeros(1,6)
    initialStateMean    = [0 0 0]'; % default: do not change! (initial robot pose)
    
    timestep            = size(sensor.timestep, 2);
    numlandmarks        = size(landmarks,2);
    for t = 1:timestep
        % Get ground true pose
        if t == 1
            pose        = initialStateMean;
        else
            u           = [sensor.timestep(t).odometry.t;
                           sensor.timestep(t).odometry.r1;
                           sensor.timestep(t).odometry.r2];
            pose        = sys.gfun(pose, u); 
        end
        measurement     = [];
        % get measurements and append them to sensor data
        for l = 1:numlandmarks
            data            = sys.hfun(landmarks(l).x, landmarks(l).y, pose);
            if data(1,1) < range_lim && abs(data(2,1)) < bearing_lim
                result.id       = landmarks(l).id;
                % Take noise measurements
                result.range    = normrnd(data(1,1), sys.Qt(1,1));
                result.bearing  = normrnd(data(2,1), sys.Qt(2,2));
                measurement     = [measurement, result];
            end
        end
        % reset sensor data
        sensor.timestep(t).sensor   = measurement;
    end
end