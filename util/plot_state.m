function plot_state(SLAM, gt, trajectory, landmarks, timestep, z, window)
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
    L = struct2cell(landmarks);
    alpha   = 0.5;
    plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'o', 'color', [0,0,0] + alpha, 'markersize', 15, 'linewidth', 2);
    text(cell2mat(L(2,:)), cell2mat(L(3,:)), string(cell2mat(L(1,:))), 'FontSize', 8);

    % Plot the particles
    ppos = [SLAM.particle.pose];
    plot(ppos(1,:), ppos(2,:), 'g.');

    % determine the currently best particle
    [~, bestParticleIdx] = max([SLAM.particle.weight]);

    % draw the landmark locations along with the ellipsoids
    for i = 1:length(SLAM.particle(bestParticleIdx).landmark)
        if SLAM.particle(bestParticleIdx).landmark(i).isobserved
            l = SLAM.particle(bestParticleIdx).landmark(i).EKF.mu;
            plot(l(1), l(2), 'bo', 'markersize', 3);
            drawprobellipse(l, SLAM.particle(bestParticleIdx).landmark(i).EKF.Sigma, 0.95, 'b');
        end
    end

    % draw the observations
    for i = 1:size(z,2)
%         l = SLAM.particle(bestParticleIdx).landmark(z(i).id).EKF.mu;
%         l = SLAM.particle(bestParticleIdx).landmark(i).EKF.mu;
%         line([SLAM.particle(bestParticleIdx).pose(1), l(1)], [SLAM.particle(bestParticleIdx).pose(2), l(2)],...
%             'color', 'r', 'LineStyle','--', 'linewidth', 1);
        pose    = SLAM.particle(bestParticleIdx).pose;
        l_x     = pose(1) + z(i).range*cos(pose(3)+z(i).bearing);
        l_y     = pose(2) + z(i).range*sin(pose(3)+z(i).bearing);
        line([pose(1), l_x], [pose(2), l_y],...
            'color', 'r', 'LineStyle','--', 'linewidth', 1);
    end

    % draw the groud true trajectory
    line(gt(1,1:timestep), gt(2,1:timestep), 'color', 'cyan', 'linewidth', 2);

    % draw the trajectory as estimated by the currently best particle
    trajectory = [trajectory{bestParticleIdx,:}];
    line(trajectory(1,:), trajectory(2, :), 'color', 'k', 'LineStyle','-.', 'linewidth', 2);

    drawrobot(SLAM.particle(bestParticleIdx).pose, 'r', 3, 0.3, 0.3);
    xlim([-2, 12])
    ylim([-2, 12])

    hold off

    % dump to a file or show the window
    if window
        figure(1);
        drawnow;
        pause(0.1);
    else
        figure(1, "visible", "off");
        filename = sprintf('../plots/fastslam_%03d.png', timestep);
        print(filename, '-dpng');
    end
end
