function SLAM = SLAM_initialization(varargin)
    % FastSLAM with known data association
    if nargin == 6
        sys                 = varargin{1,1};
        initialStateMean    = varargin{1,2};
        initialStateCov     = varargin{1,3};
        num_landmarks       = varargin{1,4};
        num_Particles       = varargin{1,5};
        SLAM_name           = varargin{1,6};
    % FastSLAM1.0 with unknown data association
    elseif nargin == 8
        sys                 = varargin{1,1};
        initialStateMean    = varargin{1,2};
        initialStateCov     = varargin{1,3};
        num_landmarks       = varargin{1,4};
        num_Particles       = varargin{1,5};
        range_lim           = varargin{1,6};
        bearing_lim         = varargin{1,7};
        SLAM_name           = varargin{1,8};
    end
    
    switch SLAM_name
        % FastSLAM with known data association
        case "FastSLAM"
            init.mu     = initialStateMean;
            init.Sigma  = initialStateCov;
            % Inverse of observation function obj.hfun: 
            % Given:  observation z_t and current estimated states x_t
            % Return: Estimated landmark position [x; y]
            init.h_inv  = @(z_t, x_t) [...
                            x_t(1) + cos(z_t(2)+x_t(3)) * z_t(1);
                            x_t(2) + sin(z_t(2)+x_t(3)) * z_t(1);];
            init.Hfun   = @(landmark_x, landmark_y, x, z_hat) [...
                           (landmark_x - x(1))/z_hat(1),       (landmark_y - x(2))/z_hat(1);
                           (x(2) - landmark_y)/(z_hat(1)^2),   (landmark_x - x(1))/(z_hat(1)^2)];
            init.n      = num_Particles;
            init.particle   = [];
            
            % The map contains 6 landmarks
            % And each particles contains 6 EKFs which tracking the 6 landmarks
            for i = 1:init.n
                init.particle(i).weight     = 1 / init.n;
                init.particle(i).pose       = mvnrnd(init.mu, init.Sigma)';
                for l = 1:num_landmarks
                    init.particle(i).landmark(l).EKF    = filter_initialization(sys, zeros(2,1), zeros(2,2), 'EKF_Landmark');
                    init.particle(i).landmark(l).isobserved     = false;
                end
            end
            SLAM        = FastSLAM(sys, init);
        
        % FastSLAM1.0 with unknown data association
        case "FastSLAM_unknow1"
            init.mu     = initialStateMean;
            init.Sigma  = initialStateCov;
            % Inverse of observation function obj.hfun: 
            % Given:  observation z_t and current estimated states x_t
            % Return: Estimated landmark position [x; y]
            init.h_inv  = @(z_t, x_t) [...
                            x_t(1) + cos(z_t(2)+x_t(3)) * z_t(1);
                            x_t(2) + sin(z_t(2)+x_t(3)) * z_t(1);];
            init.Hfun   = @(landmark_x, landmark_y, x, z_hat) [...
                           (landmark_x - x(1))/z_hat(1),       (landmark_y - x(2))/z_hat(1);
                           (x(2) - landmark_y)/(z_hat(1)^2),   (landmark_x - x(1))/(z_hat(1)^2)];
            init.n      = num_Particles;
            init.particle       = [];
            init.range_lim      = range_lim;
            init.bearing_lim    = bearing_lim;
            
            % unknow landmarks' numbers, therefore initialize zero landmark
            for i = 1:init.n
                init.particle(i).weight     = 1 / init.n;
                init.particle(i).pose       = mvnrnd(init.mu, init.Sigma)';
                init.particle(i).N          = 0; % number of landmarks exist on map
                init.particle(i).landmark   = [];
            end
            SLAM        = FastSLAM_unknown1(sys, init);
            
            % FastSLAM2.0 with unknown data association
        case "FastSLAM_unknown2"
            init.mu     = initialStateMean;
            init.Sigma  = initialStateCov;
            init.h_inv  = @(z_t, x_t) [...
                            x_t(1) + cos(z_t(2)+x_t(3)) * z_t(1);
                            x_t(2) + sin(z_t(2)+x_t(3)) * z_t(1);];
            init.Hmfun   = @(landmark_x, landmark_y, x, z_hat) [...
                            (landmark_x - x(1))/z_hat(1),       (landmark_y - x(2))/z_hat(1);
                            (x(2) - landmark_y)/(z_hat(1)^2),   (landmark_x - x(1))/(z_hat(1)^2)];
            init.Hxfun = @(landmark_x, landmark_y, x, z_hat) [...
                            -(landmark_x - x(1))/z_hat(1),     -(landmark_y - x(2))/z_hat(1),  0
                            (landmark_y - x(2))/(z_hat(1)^2), -(landmark_x - x(1))/(z_hat(1)^2), -1;];
            % Hmhun = dh/dm
            % Hxfun = dh/dx
            % Now we do not know how many land mark in whole map. Assume
            % initial landmark number m is zero
            init.n      = num_Particles;
            init.particle   = [];
            init.range_lim      = range_lim;
            init.bearing_lim    = bearing_lim;
            for i = 1:init.n
                init.particle(i).weight     = 1 / init.n;
                init.particle(i).pose       = mvnrnd(init.mu, init.Sigma)';
                init.particle(i).m          = 0;
            end
            SLAM        = FastSLAM_unknown2(sys, init);
    end
end