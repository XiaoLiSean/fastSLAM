function plot_state_vp(SLAM, gt, trajectory, timestep, z, window)
    % Visualizes the state of the FastSLAM algorithm.
    %
    % The resulting plot displays the following information:
    % - map ground truth (black +'s)
    % - currently best particle (red)
    % - particle set in green
    % - current landmark pose estimates (blue)
    % - visualization of the observations made at this time step (line between robot and landmark)
    clf;
    hold on
    grid("on")
    %graphics_toolkit gnuplot
%     L = struct2cell(landmarks);
%     alpha   = 0.5;
%     plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'o', 'color', [0,0,0] + alpha, 'markersize', 15, 'linewidth', 2);
%     text(cell2mat(L(2,:)), cell2mat(L(3,:)), string(cell2mat(L(1,:))), 'FontSize', 8);

    % draw the groud true trajectory
    plot(gt(1,:), gt(2,:), '.', 'color', 'cyan');%, 'linewidth', 2

    % Plot the particles
    ppos = [SLAM.particle.pose];
    plot(ppos(1,:), ppos(2,:), 'g.');

    % determine the currently best particle
    [~, bestParticleIdx] = max([SLAM.particle.weight]);

    % draw the landmark locations along with the ellipsoids
    % Plot for FastSLAM with known data association
    if isfield(SLAM.particle(bestParticleIdx).landmark, 'isobserved')
        for i = 1:length(SLAM.particle(bestParticleIdx).landmark)
            if SLAM.particle(bestParticleIdx).landmark(i).isobserved
                l = SLAM.particle(bestParticleIdx).landmark(i).EKF.mu;
                plot(l(1), l(2), 'bo', 'markersize', 3);
                drawprobellipse(l, SLAM.particle(bestParticleIdx).landmark(i).EKF.Sigma, 0.95, 'b');
            end
        end
    % Plot for FastSLAM with unknown data association
    else
        for i = 1:length(SLAM.particle(bestParticleIdx).landmark)
            l = SLAM.particle(bestParticleIdx).landmark(i).EKF.mu;
            plot(l(1), l(2), 'bo', 'markersize', 3);
            drawprobellipse(l, SLAM.particle(bestParticleIdx).landmark(i).EKF.Sigma, 0.95, 'b');
        end
    end
    

    % draw the observations
    for i = 1:size(z,2)
        pose    = SLAM.particle(bestParticleIdx).pose;
        l_x     = pose(1) + z(1,i)*cos(pose(3)+z(2,i));
        l_y     = pose(2) + z(1,i)*sin(pose(3)+z(2,i));
        line([pose(1), l_x], [pose(2), l_y],...
            'color', 'r', 'LineStyle','--', 'linewidth', 1);
    end
        
    
    % draw the trajectory as estimated by the currently best particle
    trajectory = [trajectory{bestParticleIdx,:}];
    line(trajectory(1,:), trajectory(2, :), 'color', 'k', 'LineStyle','-.', 'linewidth', 2);

    drawrobot(SLAM.particle(bestParticleIdx).pose, 'r', 3, 0.3, 0.3);
    xlim([-180, 60])
    ylim([-90, 180])
%     xlim([-260, 60])
%     ylim([-80, 180])

    hold off

    % dump to a file or show the window
    if window
        figure(1);
        drawnow;
%         pause(0.00001);
    else
        figure(1, "visible", "off");
        filename = sprintf('../plots/fastslam_%03d.png', timestep);
        print(filename, '-dpng');
    end
end
