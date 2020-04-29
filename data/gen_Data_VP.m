clear all; clc;
load('VictoriaPark2.mat');
load('pose.mat');
x0  = [gps(1,1) gps(2,1) 35.5*pi/180]';
pose    = [x0,pose];
for i = 1:length(ut)
    dx      = pose(1,i+1) - pose(1,i);
    dy      = pose(2,i+1) - pose(2,i);
    dtheta  = pose(3,i+1) - pose(3,i);
    ut(i).t     = sqrt(dx^2+dy^2);
    ut(i).r1    = atan2(dy,dx) - pose(3,i);
    ut(i).r2    = dtheta - ut(i).r1;
end
save VP gps timeGps timeUt timeZt ut zt x0
