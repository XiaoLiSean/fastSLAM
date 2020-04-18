classdef EKF_Landmark < handle
    properties
        mu;             % Pose Mean
        Sigma;          % Pose Covariance
        hfun;           % Measurement model equations
        Hfun;           % Measruement Model Jacobian
        Qt;             % Sensor Noise Covariance
        p;              % Possibility for getting z_t    
    end
    
    methods
        function obj = EKF_Landmark(sys, init)
            % measurement model
            obj.hfun    = sys.hfun;
            % Jocabian of measurement model
            obj.Hfun    = init.Hfun;
            % measurement noise covariance
            obj.Qt      = sys.Qt;
            % initial mean and covariance
            obj.mu      = init.mu;
            obj.Sigma   = init.Sigma;
            obj.p       = 0;
        end
        
        function EKF_update(obj, z_t, x_t)
            z_hat   = obj.hfun(obj.mu(1), obj.mu(2), x_t);
            H       = obj.Hfun(obj.mu(1), obj.mu(2),...
                               x_t, z_hat);
            Q       = H*obj.Sigma*H' + obj.Qt;
            K       = obj.Sigma*H' / Q;
            v       = z_t - z_hat; % innovation
            v(2,1)  = wrapToPi(v(2,1));
            obj.mu  = obj.mu + K*v;
            obj.Sigma   = (eye(2)-K*H)*obj.Sigma;
            obj.p   = mvnpdf(z_t, z_hat, Q);
        end
    end
end

