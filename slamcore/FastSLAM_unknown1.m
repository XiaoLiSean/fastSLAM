classdef FastSLAM_unknown1 < handle
    properties
        gfun;               % Motion model function
        hfun;               % Measurement Model Function
        Hfun;               % Measruement Model Jacobian
        h_inv               % Inverse of Measurement model
        Qt;                 % Sensor Noise
        M;                  % Motion Model Noise (dynamical and function of input)
        n;                  % Number of Particles
        particle;           % Pose of particle
        ekf_sys             % Used to initialized landmarks' EKFs
        range_lim           % range limit of perceptual field
        bearing_lim         % bearing limit of perceptual field
        mu;
        Sigma;
    end
    
    methods
        function obj = FastSLAM_unknown1(sys, init)
            % motion model
            obj.gfun    = sys.gfun;
            % measurement model
            obj.hfun    = sys.hfun;
            obj.Hfun    = init.Hfun;
            % Inverse of Measurement model
            obj.h_inv   = init.h_inv;
            % motion noise covariance
            obj.M   = sys.M;
            % measurement noise covariance
            obj.Qt  = sys.Qt;
            % PF parameters
            obj.Sigma   = init.Sigma;
            obj.n       = init.n;
            obj.particle    = init.particle;
            obj.ekf_sys     = sys;
            obj.range_lim   = init.range_lim;
            obj.bearing_lim = init.bearing_lim;
        end
        
        % Propagate Particles through Motion Model (Motion model)
        function prediction(obj, u)
            Cov     = obj.M([u.t; u.r1; u.r2]);
            for k = 1:obj.n
                u_noise     = [normrnd(u.t, Cov(1,1));...
                               normrnd(u.r1, Cov(2,2));...
                               normrnd(u.r2, Cov(3,3))];
                obj.particle(k).pose    = obj.gfun(obj.particle(k).pose, u_noise);
            end
        end
               
        function correction(obj, z)
            num_measurements    = size(z, 2);
            % Since we have multiple measurements, we need this
            % correspondence data for later counter update to avoid
            % situation: z(1) -> landmark(1) while z(2) -> landmark(2)
            % after each measurement loop, the counter for both will -1;
            cor     = zeros(num_measurements, obj.n);
            for k = 1:obj.n
                for nn = 1:num_measurements
                z_t     = [z(nn).range; wrapToPi(z(nn).bearing)];
                
                    % Update landmark numbers in map: obj.particle(k).N
                    N_prev  = obj.particle(k).N; % N_{t-1} number of landmarks in previous timestep
                    w       = zeros(1, N_prev); % correspondence vector
                    if N_prev > 0 % case initial step N_prev = 0
                        % Measurement likelihoods
                        for l = 1:N_prev
                            mu_l    = obj.particle(k).landmark(l).EKF.mu;
                            z_hat   = obj.hfun(mu_l(1), mu_l(2), obj.particle(k).pose);
                            H       = obj.Hfun(mu_l(1), mu_l(2),...
                                               obj.particle(k).pose, z_t);
                            Q_l     = H*obj.particle(k).landmark(l).EKF.Sigma*H' + obj.Qt;
                            w(l)    = mvnpdf(z_t, z_hat, Q_l); % likelihood of correspondence
                        end
                    end
                    p0              = 0.5; % importance factor, new feature, tuning parameter
                    w(N_prev+1)     = p0;
                    [w_k, c_hat]    = max(w);
                    cor(nn, k)      = c_hat; % store correspondence data
                    obj.particle(k).weight  = w_k * obj.particle(k).weight;
                    obj.particle(k).N       = max([N_prev, c_hat]);
                
                    % Update EKFs
                    for j = 1:obj.particle(k).N
                        % New features: initialize previously un-observed landmarks
                        if j == c_hat && c_hat == N_prev + 1
                            Mean_init   = obj.h_inv(z_t, obj.particle(k).pose);
                            H           = obj.Hfun(Mean_init(1), Mean_init(2),...
                                                   obj.particle(k).pose, z_t);
                            H_inv       = eye(2) / H;
                            Cov_init    = H_inv*obj.Qt*H_inv';
                            % Initialize new landmark
                            obj.particle(k).landmark(j).EKF = filter_initialization(obj.ekf_sys, Mean_init,...
                                                                                    Cov_init, 'EKF_Landmark');
                            obj.particle(k).landmark(j).i  = 1; % initialize counter
                        % EKF update k'th particles landmark(l) with highest correspondence (if it's not new feature) 
                        elseif j == c_hat && c_hat <= N_prev
                            obj.particle(k).landmark(j).EKF.EKF_update(z_t, obj.particle(k).pose);
                            obj.particle(k).landmark(j).i   = obj.particle(k).landmark(j).i + 1;
                        end
                    end
                    % Exclude dubious landmarks at last measurement data
                    % using correspondence data
                    idx_dubious  = [];
                    if nn == num_measurements
                        for j = 1:obj.particle(k).N
                            pose_l  = obj.particle(k).landmark(j).EKF.mu;
                            obsv    = obj.hfun(pose_l(1), pose_l(2), obj.particle(k).pose);
                            % Rule out feature j which is in perceptual
                            % field for robot but not have corresponding
                            % measurement in correspondence data
                            if obsv(1) < obj.range_lim && abs(obsv(2)) < obj.bearing_lim && ~ismember(j, cor(:,k))
                                obj.particle(k).landmark(j).i   = obj.particle(k).landmark(j).i - 1;                       
                                if obj.particle(k).landmark(j).i < 0
                                    idx_dubious  = [idx_dubious, j];
                                end
                            end
                        end
                    end
                    % Discard dubious features
                    
                    obj.particle(k).landmark(idx_dubious)     = []; % discard dubious features
                    obj.particle(k).N   = length(obj.particle(k).landmark);
                    
                end
            end
            
            % Update Weights
            weight_sum  = sum([obj.particle.weight]);
            for k = 1:obj.n
                obj.particle(k).weight  = obj.particle(k).weight / weight_sum;
            end

            Neff = 1 / sum([obj.particle.weight].^2);
            if Neff < obj.n / 2
                obj.resample();
            end
            % obj.meanAndVariance();
        end 
         
        function resample(obj)
            W = cumsum([obj.particle.weight]);
            r = rand / obj.n;
            j = 1;
            for i = 1:obj.n
                u = r + (i-1) / obj.n;
                while u > W(j)
                    j = j+1;
                end
                obj.particle(i) = obj.particle(j);
                obj.particle(i).weight  = 1 / obj.n;
            end
        end
        
%         function meanAndVariance(obj)
%             % Compute Mean and Variance for Robot
%             obj.mu = obj.particles.robot * obj.particle_weights'; 
%             % orientation is a bit more tricky.
%             sinSum = 0;
%             cosSum = 0;
%             for s = 1:obj.n
%                 cosSum  = cosSum + cos(obj.particles.robot(3,s));
%                 sinSum  = sinSum + sin(obj.particles.robot(3,s));
%             end
%             obj.mu(3) = atan2(sinSum, cosSum);     
%             % Compute covariance.
%             zeroMean = obj.particles.robot - repmat(obj.mu, 1, obj.n);
%             for s = 1:obj.n
%                 zeroMean(3,s) = wrapTo2Pi(zeroMean(3,s));
%             end
%             obj.Sigma = zeroMean * zeroMean' / obj.n;
%         end
    end
end