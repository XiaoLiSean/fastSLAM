clear all; clc
close all;
%% Preparation for DATA, MAP and PATH
% Make tools available
addpath('util');
addpath('slamcore');
% Load data
load('./data/VP_SEQ.mat');
range_lim   = 30;
bearing_lim = pi;

%% Initialize SLAM systems
SLAM_name           = 'FastSLAM_unknown2';
% Number of landmarks in the map
numlandmarks        = 0;
% how many particles
numParticles        = 1;
% simulation timestep
%tt = timeUt(1):1:timeUt(end);
timestep            = length(Data);
% Initializa trajectory register for plot {particle, time step}
trajectory          = cell(numParticles, timestep);
gt                  = gps; % Groud truth trajectory
% Set initial mean and covariance (prior) for PF
initialStateMean    = x0; 
initialStateCov     = diag([0.001, 0.001, 0.001]);%
% Motion noise
% alphas  = 0.0*[0.0025 0.005 0.0025 1.2 0.0025 0.005].^2;% 1sec
% alphas  = [0.0025 0.0005 0.025 1.2 0.025 0.05].^2;
alphas  = [0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2; % variance of noise proportional to alphas
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

for t = 1:timestep
    % Store system particle trajectory for plot
    for i = 1:numParticles
        trajectory{i,t}     = SLAM.particle(i).pose;
    end
    % Update the particle filter
%    [u,z,gt] = sync_data(tt(ts),ut,zt,gps,timeUt,timeZt,timeGps);
%     u = [0 0 0]';
%     tic;
    z = Data(t).sensors;
    SLAM.update(Data(t).odometry, Data(t).sensors);%*0.214
%     toc;   
    disp('Timestep'); disp(t);
    % Generate visualization plots of the current state of the filter
    zz   = [z.range; z.bearing];
    plot_state_vp(SLAM, gps, trajectory, t, zz, showGui);
end
