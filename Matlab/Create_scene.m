function [X,Y,Z,pcloud] = Create_scene()

disp('Creating scene..')
x = linspace(-5,5,50);
[X,Y] = meshgrid(x);
Z = 0.5*peaks(X,Y);
X = 2*X;
Y = 2*Y;

pcloud = zeros(3,numel(Z));
pcloud(1,:) = X(:);
pcloud(2,:) = Y(:);
pcloud(3,:) = Z(:);
disp('   Scene created')




end