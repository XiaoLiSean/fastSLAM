classdef FastSLAM_unknown2 < handle
    properties
        gfun;               % Motion model function
        hfun;               % Measurement Model Function
        Hxfun;              % Measruement Model Jacobian dh/dx
        Hmfun;              % Measruement Model Jacobian dh/dm
        h_inv               % Inverse of Measurement model
        Qt;                 % Sensor Noise
        M;                  % Motion Model Noise (dynamical and function of input)
        n;                  % Number of Particles
        particle;           % Pose of particle (pose, weight, m, p, landmark)
        mu;
        Sigma;
        p0 = 0.001;%;0.01%          % likelihood threshold
        range_lim;          % sensor perceptual range
        bearing_lim;        % +-pi
    end
    
    methods
        function obj = FastSLAM_unknown2(sys, init)
            % motion model
            obj.gfun    = sys.gfun;
            % measurement model
            obj.hfun    = sys.hfun;
            obj.Hmfun    = init.Hmfun;
            obj.Hxfun    = init.Hxfun;
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
            % number of landmark
            % obj.particle.m  = init.m;
            % sensor limit
            obj.range_lim   = init.range_lim;
            obj.bearing_lim = init.bearing_lim;
        end
        

        function update(obj, u, z)
            % for fastSLAM2 the seperation between prediction and
            % correction is not clear
            EKF_Sigma = obj.M([u.t; u.r1; u.r2]);
            u_noise = [normrnd(u.t, EKF_Sigma(1,1));...
                           normrnd(u.r1, EKF_Sigma(2,2));...
                           normrnd(u.r2, EKF_Sigma(3,3))];
%             EKF_Sigma = obj.M(u);
%             u_noise = [normrnd(u(1), EKF_Sigma(1,1));...
%                            normrnd(u(2), EKF_Sigma(2,2));...
%                            normrnd(u(3), EKF_Sigma(3,3))];


            num_measurements = size(z, 2);
%             tic;
            for k = 1:obj.n
                pose_nn = nan(3,num_measurements);
                weight_nn = nan(1,num_measurements);
                cor_nn = nan(1,num_measurements);
                for nn = 1:num_measurements
                    z_t = [z(nn).range; wrapToPi(z(nn).bearing)];
%                     z_t = [z(1,nn); wrapToPi(z(2,nn))];
                    
                if obj.particle(k).m == 0
                    j = 0;
                else
                    obj.particle(k).p = [];
                    for j = 1:obj.particle(k).m
                        x_hat = obj.gfun(obj.particle(k).pose, u_noise);
                        EKF_mu = obj.particle(k).landmark(j).EKF.mu;
                        EKF_Sigma = obj.particle(k).landmark(j).EKF.Sigma;
                        z_bar = obj.hfun(EKF_mu(1),EKF_mu(2), x_hat);
                        Hx = obj.Hxfun(EKF_mu(1),EKF_mu(2), x_hat, z_bar);
                        Hm = obj.Hmfun(EKF_mu(1),EKF_mu(2), x_hat, z_bar);
                        Q = obj.Qt + Hm*EKF_Sigma*Hm';
                        
                        if sum(diag(obj.M(u_noise))) ~= 0
                            Sigma_x = inv( Hx'*inv(Q)*Hx + inv(obj.M(u_noise)) );
                        else
                            Sigma_x = inv( Hx'*inv(Q)*Hx + inv(diag([0.02; 0.02; 2*pi/180].^2)) );
                        end
                        mu_x = Sigma_x*Hx'*inv(Q)*(z_t-z_bar) + x_hat;
                        L_x = chol(Sigma_x,'lower');
                        x_j = mu_x + L_x*randn(3,1);
                        z_hat = obj.hfun(EKF_mu(1), EKF_mu(2), x_j);
                        obj.particle(k).p(j) = mvnpdf(z_t,z_hat,Q);
                    end
                end

                obj.particle(k).p(j+1) = obj.p0;
                [~,c_hat] = max(obj.particle(k).p);
                cor_nn(nn) = c_hat;
                isNew = (c_hat > obj.particle(k).m);
                obj.particle(k).m = max(obj.particle(k).m,c_hat);
                
                for j = 1:obj.particle(k).m
                    if j == c_hat && isNew % new landmark
                        pose_nn(:,nn) = obj.gfun(obj.particle(k).pose, u_noise);
                        EKF_mu = obj.h_inv(z_t, pose_nn(:,nn));
                        Hm = obj.Hmfun(EKF_mu(1), EKF_mu(2), pose_nn(:,nn), z_t);
                        H_inv = eye(2) / Hm;
                        EKF_Sigma = H_inv*obj.Qt*H_inv';
                        obj.particle(k).landmark(j).EKF.mu     = EKF_mu;
                        obj.particle(k).landmark(j).EKF.Sigma  = EKF_Sigma;
                        obj.particle(k).landmark(j).counter    = 1;
                        %obj.particle(k).landmark(j).isobserved = true;
                        weight_nn(nn)          = obj.p0;
                        
                    elseif j == c_hat && ~isNew                        
                        x_hat = obj.gfun(obj.particle(k).pose, u_noise);
                        EKF_mu = obj.particle(k).landmark(j).EKF.mu;
                        EKF_Sigma = obj.particle(k).landmark(j).EKF.Sigma;
                        z_bar = obj.hfun(EKF_mu(1),EKF_mu(2), x_hat);
                        Hx = obj.Hxfun(EKF_mu(1),EKF_mu(2), x_hat, z_bar);
                        Hm = obj.Hmfun(EKF_mu(1),EKF_mu(2), x_hat, z_bar);
                        Q = obj.Qt + Hm*EKF_Sigma*Hm';
                        if sum(diag(obj.M(u_noise))) ~= 0
                            Sigma_x = inv( Hx'*inv(Q)*Hx + inv(obj.M(u_noise)) );
                        else
                            Sigma_x = inv( Hx'*inv(Q)*Hx + inv(diag([0.02; 0.02; 2*pi/180].^2)) );
                        end
                        mu_x = Sigma_x*Hx'*inv(Q)*(z_t-z_bar) + x_hat;
                        L_x = chol(Sigma_x,'lower');
                        x_j = mu_x + L_x*randn(3,1);
                        z_hat = obj.hfun(EKF_mu(1), EKF_mu(2), x_j);
                        K = EKF_Sigma*Hm'*inv(Q);
                        L = Hx*obj.M(u_noise)*Hx' + Hm*EKF_Sigma*Hm' + obj.Qt;
                        EKF_mu = EKF_mu + K*(z_t-z_hat);
                        EKF_Sigma = (eye(2)-K*Hm)*EKF_Sigma;
                        pose_nn(:,nn) = x_j;
                        
                        obj.particle(k).landmark(j).EKF.mu     = EKF_mu;
                        obj.particle(k).landmark(j).EKF.Sigma  = EKF_Sigma;
                        obj.particle(k).landmark(j).counter    = obj.particle(k).landmark(j).counter+1;
                        %obj.particle(k).landmark(j).isobserved = true;
                        weight_nn(nn)          = mvnpdf(z_t,z_hat,L);
                    % else
                        % obj.particle(k).landmark(j).EKF.mu NOT change
                        % obj.particle(k).landmark(j).EKF.Sigma NOT change
                        % obj.particle(k).landmark(j).counter NOT change
                    end
                end 
                end
                
                [obj.particle(k).pose, weight_k] = obj.PoseAndWeight(pose_nn,weight_nn);
                obj.particle(k).weight = obj.particle(k).weight*weight_k;
                
                % Discard dubious feature
                is_dubious  = false(1, obj.particle(k).m);
                for j = 1:obj.particle(k).m
                    pose_l = obj.particle(k).landmark(j).EKF.mu;
                    obsv = obj.hfun(pose_l(1), pose_l(2), obj.particle(k).pose);
                    if obsv(1) < obj.range_lim && abs(obsv(2)) < obj.bearing_lim && ~ismember(j, cor_nn)
                        obj.particle(k).landmark(j).counter = obj.particle(k).landmark(j).counter - 1;
                        if obj.particle(k).landmark(j).counter < 0
                             is_dubious(1,j) = true;
                        end 
                    end
                end
                
                obj.particle(k).landmark(is_dubious) = []; 
                obj.particle(k).m = obj.particle(k).m - sum(is_dubious);

            end
%             toc;
            % Update Weights
            % try increase weight difference ======
%             temp = ([obj.particle.weight] - min([obj.particle.weight]));
%             if any(temp~=0)
%                 for k = 1:obj.n
%                     obj.particle(k).weight = temp(k);
%                 end
%             end
            %======================================
            weight_sum  = sum([obj.particle.weight]);
            for k = 1:obj.n
                obj.particle(k).weight  = obj.particle(k).weight / weight_sum;
            end

            Neff = 1 / sum([obj.particle.weight].^2);
            disp(Neff)
            if Neff < obj.n*0.85 %obj.n*0.9 obj.n / 2 *0.85
                obj.resample();
                disp('resample');
            end
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
        
        function [pose,weight] = PoseAndWeight(obj,pose_nn,weight_nn)
            num = length(weight_nn);
            pose = [sum(pose_nn(1,:))/num; sum(pose_nn(2,:))/num;...
                    atan2(sum(sin(pose_nn(3,:))),sum(cos(pose_nn(3,:))))];
            weight = 1;
            for i = 1:num
                weight = weight*weight_nn(i);
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