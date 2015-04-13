% Real kinect sample test
clc
clear
close all

%% Get input
% DM = imread('C:\Users\RGrandia\Dropbox\3dPhotography\recordings\depth images\some_depth_scene.png');
DM = load('C:\Users\RGrandia\Dropbox\3dPhotography\recordings\depth images\output.mat','-ASCII');
%% Camera internals
% Define camera postition and orientation w.r.t. world frame
C = [0 -1 0];
R = rotx(90)*roty(0)*rotz(0);

% Camera intrinsics
ax = 640/2/tand(57/2);
ay = 480/2/tand(43/2);
K = [ax, 0,  641/2;...
     0,   ay, 481/2;...
     0,     0,    1]; 

%% construct voxels
[Voxels,units] = fn_depth_to_voxels(DM,K,R,C);

%% Plot
% Plot Voxel grid for values |f(d)| < 0.1
s = find(abs(Voxels)<0.2);
[vx,vy,vz] = meshgrid(1:length(units));

figure(1)
plot3(-vx(s),vy(s),vz(s),'.')
grid on
xlabel('x');
ylabel('y');
zlabel('z');
axis equal

figure(2)
b = pcolor(flipud(DM));
set(b,'Edgecolor','none')
colorbar
axis equal
axis([0,640,0,480]);