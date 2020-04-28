function [u,z,gt] = sync_data(t,ut,zt,gps,timeUt,timeZt,timeGps)
% idx1 = find(timeUt<=t,1,'last');
% if isempty(idx1)
%     u = 0.0001*[1 1 1]';
% else
%     u = ut(:,idx1);
% end
% % u = ut(:,idx1);
% 
% idx2 = find(timeZt<=t,1,'last');
% z = cell2mat(zt(:,idx2));
% 
% % gt = gps(:,timeGps<=t);
% gt = gps;
% ========================


idx1 = find(timeUt<=t,1,'last');
if isempty(idx1)
    u.t = 0.0001;
    u.r1 = 0.0001;
    u.r2 = 0.0001;
else
    u = ut(idx1);
end


idx2 = find(timeZt<=t,1,'last');
z = zt(idx2).sensor;

% gt = gps(:,timeGps<=t);
gt = gps;


end