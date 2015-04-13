function pcloud_out = C2E(pcloud_in,R,C_center)
% Map points from camera to world frame
% R is rotation matrix
% T is translation vector

[m,n,~] = size(pcloud_in);
pcloud_out = zeros(size(pcloud_in));
for i = 1:m
    for j = 1:n
        pcloud_out(i,j,:) = R'*squeeze(pcloud_in(i,j,:))+C_center(:);
    end
end

end