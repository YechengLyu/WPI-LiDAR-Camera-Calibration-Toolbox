function [idx] = fun_pc_sel(pc)
idx = ones(1,size(pc,2));
x = pc(1,:);
y = pc(2,:);
z = pc(3,:);
r = vecnorm(pc(1:3,:));
end