clear all; clc
close all;
%% Preparation for DATA, MAP and PATH
% Make tools available
addpath('util');
addpath('slamcore');
load('./data/VP.mat');
% Read sensor readings, i.e. odometry and range-bearing sensor
range_lim   = 30;
bearing_lim = pi;

%% Initialize SLAM systems
SLAM_name           = 'FastSLAM_unknow1';
% Number of landmarks in the map
numlandmarks        = 0;
% how many particles
numParticles        = 100;
% simulation timestep
tt = timeUt(1):1:timeUt(end);
timestep            = length(tt);
% Initializa trajectory register for plot {particle, time step}
trajectory          = cell(numParticles, timestep);
gt                  = zeros(3, timestep); % Groud truth trajectory
% Set initial mean and covariance (prior) for PF
initialStateMean    = x0; % default: do not change!
initialStateCov     = 0.001*eye(3);
% Motion noise
alphas  = 0.0*[0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2; % variance of noise proportional to alphas
% Standard deviation of Gaussian sensor noise (independent of distance)
betas   = [0.1 2 deg2rad(5)]; % which is identical to gen_measurements.m
sys     = system_initialization(alphas, betas);
SLAM    = SLAM_initialization(sys, initialStateMean, initialStateCov,...
                              numlandmarks, numParticles,...
                              range_lim, bearing_lim, SLAM_name);


% toogle the visualization type
showGui = true;
global err;
err.showErr = true;
err.obsv_ID = cell(1,timestep); % All observed landmarks' ID by now (true)
err.obsv_N  = zeros(1,timestep); % All observed landmarks' number by now (true)
err.store   = zeros(1,timestep); % All stored landmarks number in particle EKF
err.mean    = zeros(1,timestep);
err.tr      = zeros(2, timestep);
%% SLAM MAIN
% Perform filter update for each odometry-observation pair read from the data file.

for ts = 1:timestep
    % Store system particle trajectory for plot
    for i = 1:numParticles
        trajectory{i,ts}     = SLAM.particle(i).pose;
    end
    [u,z,gt] = sync_data(tt(ts),ut,zt,gps,timeUt,timeZt,timeGps);
    % Perform the prediction step of the particle filter
    SLAM.prediction(u);
    
    % Perform the correction step of the particle filter
    SLAM.correction(z);
    disp(ts)
    % Generate visualization plots of the current state of the filter
    zz   = [z.range; z.bearing];
    plot_state_vp(SLAM, gt, trajectory, ts, zz, showGui);
end
