function sys = system_initialization(alphas, beta)
%% Motion Predition Model
% motion model: 
% x = [x y theta]; 
% u = [trans r1 r2] = [v w gamma]
sys.gfun = @(x, u) [...
    x(1) + u(1)*cos(x(3)+u(2));
    x(2) + u(1)*sin(x(3)+u(2));
    wrapToPi(x(3) + u(2) + u(3))];
% motion noise: [trans r1 r2] = [v w gamma]
sys.M = @(u) [...
    alphas(1)*u(1)^2+alphas(2)*u(2)^2, 0, 0;
    0, alphas(3)*u(1)^2+alphas(4)*u(2)^2, 0;
    0, 0, alphas(5)*u(1)^2+alphas(6)*u(2)^2];
%sys.M   = diag([0.01, 0.005, 0.005]);
%% Sensor Measurement Model
% measurement model: [range bearing]
% negative for clockwise measurements
% positive for counter clockwise measurements
sys.hfun = @(landmark_x, landmark_y, x) [...
    sqrt((landmark_y - x(2))^2 + (landmark_x - x(1))^2);
    wrapToPi(atan2(landmark_y - x(2), landmark_x - x(1)) - x(3))];
% measurement noise: [range bearing]
sys.Qt  = 0.1*[...
           2^2,     0;
           0,       beta^2];
%sys.Qt   = 0.01*eye(2);
end