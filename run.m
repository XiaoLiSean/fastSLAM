clear all; clc
close all;
%% Preparation for DATA, MAP and PATH
% Make tools available
addpath('util');
addpath('slamcore');
% Read world data, i.e. landmarks. The true landmark positions are not given to the robot
% numlandmarks= 25;% Number of landmarks in the map
% map         = [-2, 11;-2, 11]; % x, y map limits
% landmarks   = gen_landmarks(map, numlandmarks);
load data/landmarks;
% Read sensor readings, i.e. odometry and range-bearing sensor
range_lim   = 5;
bearing_lim = pi;
sensor      = gen_measurements(range_lim, bearing_lim, landmarks);

%% Initialize SLAM systems
SLAM_name           = 'FastSLAM';
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
initialStateMean    = [0 0 0]';
initialStateCov     = 0.001*eye(3);
% Motion noise
alphas  = [0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2; % variance of noise proportional to alphas
% Standard deviation of Gaussian sensor noise (independent of distance)
beta    = [0.1 2 deg2rad(5)];
sys     = system_initialization(alphas, beta);
SLAM    = SLAM_initialization(sys, initialStateMean, initialStateCov,...
                              numlandmarks, numParticles, SLAM_name);


% toogle the visualization type
showGui = true;
global err;
err.showErr = true;
err.mean    = zeros(numlandmarks, timestep);
err.sig     = zeros(numlandmarks, timestep);
err.tr      = zeros(2, timestep);
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
    set(gcf,'color','w');
    plot_state(SLAM, gt, trajectory, landmarks, t, sensor.timestep(t).sensor, showGui);
end
