clear all; clc
close all;
%% Preparation for DATA, MAP and PATH
% Make tools available
addpath('util');
addpath('slamcore');
% Read world data, i.e. landmarks. The true landmark positions are not given to the robot
landmarks   = read_world('./data/world.dat');
% Read sensor readings, i.e. odometry and range-bearing sensor
range_lim   = 5;
bearing_lim = pi;
sensor      = gen_measurements(range_lim, bearing_lim);

%% Initialize SLAM systems
SLAM_name           = 'FastSLAM_unknow1';
% Number of landmarks in the map
numlandmarks        = size(landmarks,2);
% how many particles
numParticles        = 100;
% simulation timestep
timestep            = size(sensor.timestep, 2);
% Initializa trajectory register for plot {particle, time step}
trajectory          = cell(numParticles, timestep);
gt                  = zeros(3, timestep); % Groud truth trajectory
% Set initial mean and covariance (prior) for PF
initialStateMean    = [0 0 0]'; % default: do not change!
initialStateCov     = 0.001*eye(3);
% Motion noise
alphas  = [0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2; % variance of noise proportional to alphas
% Standard deviation of Gaussian sensor noise (independent of distance)
betas   = [0.1 2 deg2rad(5)]; % which is identical to gen_measurements.m
sys     = system_initialization(alphas, betas);
SLAM    = SLAM_initialization(sys, initialStateMean, initialStateCov,...
                              numlandmarks, numParticles,...
                              range_lim, bearing_lim, SLAM_name);


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
    % Perform the prediction step of the particle filter
    SLAM.prediction(sensor.timestep(t).odometry);
    
    % Perform the correction step of the particle filter
    SLAM.correction(sensor.timestep(t).sensor);
    
    % Generate visualization plots of the current state of the filter
    plot_state(SLAM, gt, trajectory, landmarks, t, sensor.timestep(t).sensor, showGui);
end
