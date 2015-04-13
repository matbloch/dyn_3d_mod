function [DM] = Simulate_Kinect(pcloud,P,R,C)
% Function simulates the depthmeasurement of the kinect sensor
% pcloud is a pointcloud in the world frame.
% P is the projection matrix

disp('Taking kinect measurement..')

% Number of points
N = size(pcloud,2);
m = 640;
n = 480;

% Map points to image plane
pproj = P*[pcloud;ones(1,N)];
pproj = pproj./repmat(pproj(3,:),[3,1]);

% Get depth value for each point
pcam = R*(pcloud-repmat(C',[1,N]));
depth = pcam(3,:);

% Resample to pixel grid
[~,ind_x] = histc(pproj(1,:),0:m);
[~,ind_y] = histc(pproj(2,:),0:n);

DM = NaN*zeros(n,m);
for i = 1:m
    for j = 1:n
        DM(j,i) = min([depth(ind_x==i & ind_y==j),NaN]);
        
    end
end

% Interpolate empty pixels
[PX,PY] = meshgrid(1:m,1:n);
idx = find(~isnan(DM));
DM = griddata(PX(idx),PY(idx),DM(idx),PX,PY);

disp('   Measurement taken')
end