clear all; clc
close all;
%% Preparation for DATA, MAP and PATH
% Make tools available
addpath('util');
addpath('util_vp');
addpath('slamcore');
VP  = gen_sequential_data();

%% Initialize SLAM systems
SLAM_name           = 'FastSLAM';
% Number of landmarks in the map
numlandmarks        = length(VP.Landmarks);
% how many particles
numParticles        = 50;
% simulation timestep
timestep            = length(VP.Data);
% Initializa trajectory register for plot {particle, time step}
trajectory          = cell(numParticles, timestep);
gt                  = [VP.GPS.x'; VP.GPS.y']; % Groud truth trajectory
% Set initial mean and covariance (prior) for PF
initialStateMean    = [VP.GPS.x(2); VP.GPS.y(2); 36*pi/180];
initialStateCov     = 0*eye(3);
% Control noise: [translation and steering]
M   = 0*diag([0.02; 2*pi/180].^2);
% Measurement noise: [range and bearing]
Qt  = diag([0.05; pi/180].^2);
sys     = system_initialization(M, Qt);
SLAM    = SLAM_initialization(sys, initialStateMean, initialStateCov,...
                              numlandmarks, numParticles, SLAM_name);


% toogle the visualization type
showGui = true;
%% SLAM MAIN
% Perform filter update for each odometry-observation pair read from the data file.
for t = 1:length(VP.Data)
    % Store system particle trajectory for plot
    for i = 1:numParticles
        trajectory{i,t}     = SLAM.particle(i).pose;
    end
    if strcmp(VP.Data(t).type, 'odometry')
        % Perform the prediction step of the particle filter
        SLAM.prediction(VP.Data(t).odometry);
    elseif strcmp(VP.Data(t).type, 'measure')
        % Perform the correction step of the particle filter
        SLAM.correction(VP.Data(t).sensors);
        % Generate visualization plots of the current state of the filter
        z   = [VP.Data(t).sensors.range;VP.Data(t).sensors.bearing];
        plot_state_vp(SLAM, gt, trajectory, t, z, showGui);
    end
end
