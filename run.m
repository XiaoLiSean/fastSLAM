clear all; clc
close all;
%% Preparation for DATA, MAP and PATH
% Make tools available
addpath('util');
addpath('slamcore');
% Read world data, i.e. landmarks. The true landmark positions are not given to the robot
landmarks   = read_world('./data/world.dat');
% Read sensor readings, i.e. odometry and range-bearing sensor
sensor      = read_data('./data/sensor_data.dat');

%% Initialize SLAM systems
SLAM_name           = 'FastSLAM2';
% Number of landmarks in the map
numlandmarks        = size(landmarks,2);
% how many particles
numParticles        = 200;
% simulation timestep
timestep            = size(sensor.timestep, 2);
% Initializa trajectory register for plot {particle, time step}
trajectory          = cell(numParticles, timestep);
gt                  = zeros(3, timestep); % Groud truth trajectory
% Set initial mean and covariance (prior) for PF
initialStateMean    = [0 0 0]';
initialStateCov     = 0.001*eye(3);
% Motion noise
alphas  = [0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2; % variance of noise proportional to alphas
% Standard deviation of Gaussian sensor noise (independent of distance)
beta    = deg2rad(10);%deg2rad(5);
sys     = system_initialization(alphas, beta);
SLAM    = SLAM_initialization(sys, initialStateMean, initialStateCov,...
                              numlandmarks, numParticles, SLAM_name);
% initialStateMean = 
% initialStateCov
% SLAM    = SLAM_initialization(sys, initialStateMean, initialStateCov,...
%                               numlandmarks, numParticles, SLAM_name,...
%                               initialMeasureMean, initialMeasureCov);


% toogle the visualization type
showGui = true;
%% SLAM MAIN
% Perform filter update for each odometry-observation pair read from the data file.
for t = 1:timestep
    % Get ground true trajectory
    if t == 1
        gt(:,t)     = initialStateMean;
    else
        u           = [sensor.timestep(t).odometry.t;
                       sensor.timestep(t).odometry.r1;
                       sensor.timestep(t).odometry.r2];
        gt(:,t)     = sys.gfun(gt(:,t-1), u); 
    end
    % Store system particle trajectory for plot
    for i = 1:numParticles
        trajectory{i,t}     = SLAM.particle(i).pose;
    end
    
    switch SLAM_name
        case 'FastSLAM'
            % Perform the prediction step of the particle filter
            SLAM.prediction(sensor.timestep(t).odometry);
            % Perform the correction step of the particle filter
            SLAM.correction(sensor.timestep(t).sensor);
            
        case 'FastSLAM2'
            % FastSLAM2.0 take both u and z
            SLAM.update(sensor.timestep(t).odometry, sensor.timestep(t).sensor); 
    end
    
    % Generate visualization plots of the current state of the filter
    plot_state(SLAM, gt, trajectory, landmarks, t, sensor.timestep(t).sensor, showGui);
end
