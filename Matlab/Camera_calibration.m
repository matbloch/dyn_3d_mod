function [P,K,R,C] = Camera_calibration()
% Define camera postition and orientation w.r.t. world frame
C = [0 -3 0];
R = rotx(90)*roty(0)*rotz(0);

% Camera intrinsics
ax = 640/2/tand(57/2);
ay = 480/2/tand(43/2);
K = [ax, 0,  641/2;...
     0,   ay, 481/2;...
     0,     0,    1]; 
 
% Projection matrix
P = K*[eye(3),zeros(3,1)]*[R,-(R*C'); 0 0 0 1];




end