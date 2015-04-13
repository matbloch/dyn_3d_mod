% Script testing the depth map -> filled in voxels pipeline

clc
clear 
close all

%% Set up the scene
[X,Y,Z,pcloud] = Create_scene();

% Get Camera calibration
[P,K,R,C] = Camera_calibration();

% Create image
DM = Simulate_Kinect(pcloud,P,R,C);

% Fill in voxels
[Voxels,units] = fn_depth_to_voxels(DM,K,R,C);


%% Plotting
% Plot Voxel grid for values |f(d)| < 0.1
s = find(abs(Voxels)<0.1);
[vx,vy,vz] = meshgrid(1:length(units));

figure(2)
plot3(vx(s),vy(s),vz(s),'.')
grid on
xlabel('x');
ylabel('y');
zlabel('z');

% Plot scene with Camera
figure(1);
subplot(1,2,1);
surf(X,Y,Z)
hold on
cam = plotCamera('Location',C,'Orientation',R,'Opacity',0);
grid on
xlabel('x');
ylabel('y');
zlabel('z');
axis(10*[-1, 1, -1, 1, -1, 1]);
axis square

% Plot depth image taken by kinect
subplot(1,2,2);
b = pcolor(DM);
set(b,'Edgecolor','none')
colorbar
axis equal
axis([0,640,0,480]);

