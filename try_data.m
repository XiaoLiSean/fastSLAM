figure;
for i = 1:length(timeGps)
    clf; hold on;
   plot(gps(1,:),gps(2,:),'b.');
   plot(gps(1,i),gps(2,i),'rx');
   axis([-260 60 -80 180]);
   drawnow;
%    pause(0.01);
   
end

%%
gfun = @(x, u) [...
    x(1) + u(1)*cos(x(3)+u(2));
    x(2) + u(1)*sin(x(3)+u(2));
    wrapToPi(x(3) + u(2) + u(3))];
x = [gps(1,1) gps(2,1) 35.5*pi/180]';
figure; hold on;
plot(gps(1,:),gps(2,:),'r.');
for i = 2:length(timeUt)
    x(:,i) = gfun(x(:,i-1),ut(:,i)*0.025);
%     plot(x(1),x(2),'b.');
%     axis([-260 60 -80 180]);
%     drawnow;
end

plot(x(1,:),x(2,:),'b.');
axis([-260 60 -80 180]);
%%
figure; hold on;
plot(gps(1,:),gps(2,:),'r.');

