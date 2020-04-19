function SLAM = SLAM_initialization(sys, initialStateMean, initialStateCov, num_landmarks, num_Particles, SLAM_name, initialMeasureMean, initialMeasureCov)
    switch SLAM_name
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
            L = chol(init.Sigma,'lower');
            for i = 1:init.n
                init.particle(i).weight     = 1 / init.n;
                init.particle(i).pose       = mvnrnd(init.mu, init.Sigma)';
                for l = 1:num_landmarks
                    init.particle(i).landmark(l).EKF    = filter_initialization(sys, zeros(2,1), zeros(2,2), 'EKF_Landmark');
                    init.particle(i).landmark(l).isobserved     = false;
                end
            end
            SLAM        = FastSLAM(sys, init);
            
            
        case "FastSLAM2"
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
            for i = 1:init.n
                init.particle(i).weight     = 1 / init.n;
                init.particle(i).pose       = mvnrnd(init.mu, init.Sigma)';
                init.particle(i).m          = 0;
            end
            SLAM        = FastSLAM2(sys, init);
    end
end