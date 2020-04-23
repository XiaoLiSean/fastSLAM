function [u,z,gt] = sync_data(t,ut,zt,gps,timeUt,timeZt,timeGps)
idx1 = find(timeUt<=t,1,'last');
u = ut(:,idx1);

idx2 = find(timeZt<=t,1,'last');
z = cell2mat(zt(:,idx2));

% gt = gps(:,timeGps<=t);
gt = gps;

end