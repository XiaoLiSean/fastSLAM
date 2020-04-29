clc;clear;

load('VictoriaPark.mat')
for i = 1:size(ut,2)
    U(i).t = ut(1,i);
    U(i).r1 = ut(2,i);
    U(i).r2 = ut(3,i);
end
for i = 1:size(zt,2)
    for j = 1:size(zt{i},2)
        z(j).range = zt{i}(1,j);
        z(j).bearing = zt{i}(2,j);
%         zz = [zz; z];
    end
    Z(i).sensor = z;
end

zt = Z;
ut = U;