classdef FastSLAM < handle
    properties
        gfun;               % Motion model function
        hfun;               % Measurement Model Function
        Hfun;               % Measruement Model Jacobian
        h_inv               % Inverse of Measurement model
        Qt;                 % Sensor Noise
        M;                  % Motion Model Noise (dynamical and function of input)
        n;                  % Number of Particles
        particle;          % Pose of particle
        mu;
        Sigma;
    end
    
    methods
        function obj = FastSLAM(sys, init)
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
        end
        
        % Propagate Particles through Motion Model (Motion model)
        function prediction(obj, u)
            if  isfield(u,'r1')
                % Prediction of self generated dataset
                Cov     = obj.M([u.t; u.r1; u.r2]);
                for k = 1:obj.n
                    u_noise     = [normrnd(u.t, Cov(1,1));...
                                   normrnd(u.r1, Cov(2,2));...
                                   normrnd(u.r2, Cov(3,3))];
                    obj.particle(k).pose    = obj.gfun(obj.particle(k).pose, u_noise);
                end
            else
                % Prediction of VP dataset
                for k = 1:obj.n
                    u_noise     = [normrnd(u.trans, obj.M(1,1));...
                                   normrnd(u.steer, obj.M(2,2))];
                    obj.particle(k).pose    = obj.gfun(obj.particle(k).pose, u_noise);
                end
            end
        end
               
        function correction(obj, z)
            num_measurements    = size(z, 2);
            for k = 1:obj.n
                for j = 1:num_measurements
                    id  = z(j).id;
                    z_t = [z(j).range; wrapToPi(z(j).bearing)];
                    % Initialize previously un-observed landmarks
                    if ~obj.particle(k).landmark(id).isobserved
                        Mean_init   = obj.h_inv(z_t, obj.particle(k).pose);
                        H           = obj.Hfun(Mean_init(1), Mean_init(2),...
                                               obj.particle(k).pose, z_t);
                        H_inv       = eye(2) / H;
                        Cov_init    = H_inv*obj.Qt*H_inv';
                        % Initialize new landmark
                        obj.particle(k).landmark(id).EKF.mu     = Mean_init;
                        obj.particle(k).landmark(id).EKF.Sigma  = Cov_init;
                        obj.particle(k).landmark(id).isobserved = true;
                    % EKF update k'th particles landmark(id) if the landmark is previouly observed 
                    else
                        obj.particle(k).landmark(id).EKF.EKF_update(z_t, obj.particle(k).pose);
                        obj.particle(k).weight  = obj.particle(k).weight * obj.particle(k).landmark(id).EKF.p;
                    end
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