function filter = filter_initialization(sys, initialStateMean,initialStateCov, filter_name)
    switch filter_name
        case "EKF_Landmark"
            init.mu     = initialStateMean;
            init.Sigma  = initialStateCov;
            init.Hfun   = @(landmark_x, landmark_y, x, z_hat) [...
                           (landmark_x - x(1))/z_hat(1),       (landmark_y - x(2))/z_hat(1);
                           (x(2) - landmark_y)/(z_hat(1)^2),   (landmark_x - x(1))/(z_hat(1)^2)];
            filter = EKF_Landmark(sys, init);
    end
end