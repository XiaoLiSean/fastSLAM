clear all; clc
close all;
%% Preparation for DATA, MAP and PATH
% Make tools available
addpath('util');
addpath('slamcore');
% Load data
load('./data/VictoriaPark2.mat');
range_lim   = 30;
bearing_lim = pi;

%% Initialize SLAM systems
SLAM_name           = 'FastSLAM_unknown2';
% Number of landmarks in the map
numlandmarks        = 0;
% how many particles
numParticles        = 100;
% simulation timestep
tt = timeUt(1):1:timeUt(end);
% tt = timeZt;
timestep            = length(tt);%size(timeUt, 2);
% Initializa trajectory register for plot {particle, time step}
trajectory          = cell(numParticles, timestep);
gt                  = zeros(3, timestep); % Groud truth trajectory
% Set initial mean and covariance (prior) for PF
initialStateMean    = [gps(1,1) gps(2,1) 35.5*pi/180]'; 
initialStateCov     = diag([0.001, 0.001, 0.001]);%
% Motion noise
alphas  = [0.0025 0.005 0.0025 1.2 0.0025 0.005].^2;% 1sec
% alphas  = [0.0025 0.0005 0.025 1.2 0.025 0.05].^2;
% alphas  = [0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2; % variance of noise proportional to alphas
% Standard deviation of Gaussian sensor noise (independent of distance)
betas   = [1 5 deg2rad(15)];
sys     = system_initialization(alphas, betas);
SLAM    = SLAM_initialization(sys, initialStateMean, initialStateCov,...
                              numlandmarks, numParticles,...
                              range_lim, bearing_lim, SLAM_name);


% toogle the visualization type
showGui = true;
%% SLAM MAIN
% Perform filter update for each odometry-observation pair read from the data file.

for ts = 1:timestep
    % Store system particle trajectory for plot
    for i = 1:numParticles
        trajectory{i,ts}     = SLAM.particle(i).pose;
    end
    % Update the particle filter
    [u,z,gt] = sync_data(tt(ts),ut,zt,gps,timeUt,timeZt,timeGps);
%     u = [0 0 0]';
%     tic;
    SLAM.update(u, z);%*0.214
%     toc;   
    % Generate visualization plots of the current state of the filter
    zz   = [z.range; z.bearing];
    plot_state_vp(SLAM, gt, trajectory, ts, zz, showGui);
end
