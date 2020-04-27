function landmarks = gen_landmarks(map, numlandmarks)
    x_lim   = map(1,:);
    y_lim   = map(2,:);
    landmarks_x     = rand(numlandmarks,1)*(x_lim(2)-x_lim(1))+x_lim(1);
    landmarks_y     = rand(numlandmarks,1)*(y_lim(2)-y_lim(1))+y_lim(1);
    id              = [1:numlandmarks]';
    
    for i = 1:numlandmarks
        landmarks(i).id    = id(i);
        landmarks(i).x     = landmarks_x(i);
        landmarks(i).y     = landmarks_y(i);
    end
end