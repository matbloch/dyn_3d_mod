function [pcloud, distance] = depthToCloud(depth)
% depthToCloud.m - Convert depth image into 3D point cloud
%
% Input: 
% depth - the depth image in [m]
%
% Output:
% pcloud - the point cloud, where each channel is the x, y, and z euclidean coordinates respectively. Missing values are NaN.
% distance - euclidean distance from the sensor to each point

depth= double(depth);
depth(depth == 0) = nan;

% RGB-D camera constants
center = [641/2 480/2];
[imh, imw] = size(depth);
ax = 640/2/tand(57/2);
ay = 480/2/tand(43/2);

% convert depth image to 3d point clouds
pcloud = zeros(imh,imw,3);
xgrid = ones(imh,1)*(1:imw) - center(1);
ygrid = (1:imh)'*ones(1,imw) - center(2);
pcloud(:,:,1) = xgrid.*depth/ax;
pcloud(:,:,2) = ygrid.*depth/ay;
pcloud(:,:,3) = depth;
distance = sqrt(sum(pcloud.^2,3));
end
