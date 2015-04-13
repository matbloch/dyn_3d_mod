function [Voxels,units] = fn_depth_to_voxels(DM,K,R,C)

persistent octree_depth Vox_perdim Vox_size

if isempty(octree_depth)
    % Set parameters
    octree_depth = 7;
    Vox_perdim = 2^(octree_depth-1);
    Vox_size = 2/(Vox_perdim()-1); % voxels size in meters
end

disp('Filling in Voxels..');

tic

% Create empty voxel grid with 2^(depth-1) voxels along each dimension
Voxels = NaN*zeros(repmat(Vox_perdim,1,3));

% Get the mapping of the voxel index to real units
units = Vox_size*(((1:Vox_perdim)-(Vox_perdim+1)/2));

% Map all voxels to image
u = zeros(size(Voxels));
v = zeros(size(Voxels));
vox_depth = zeros(size(Voxels));

vox2image()

% Loop over all voxels
for l = 1:Vox_perdim
    for m = 1:Vox_perdim
        for n = 1:Vox_perdim
            % interpolate with nearest neigbor
                depth = int_image(u(l,m,n),v(l,m,n));
            % get values
                Voxels(l,m,n) = TSDF(depth-vox_depth(l,m,n));
        end
    end
end

t = toc;


% ----- Subfunctions ----- 


% Map voxels to image plane
    function [] = vox2image()
        for i = 1:Vox_perdim
            for j = 1:Vox_perdim
                for k = 1:Vox_perdim
                    X_cam = R*([units(i),units(j),units(k)]'-C');
                    vox_depth(i,j,k) = X_cam(3);
                    pix = K*X_cam;
                    u(i,j,k) = pix(1)/pix(3);
                    v(i,j,k) = pix(2)/pix(3);
                end
            end
        end
    end

% Interpolation of image
    function depth = int_image(px,py)
        % Take nearest neighbour
        if px>640 || px<1 || py>480 || py<1
            depth = NaN;
        else
            depth = DM(round(py),round(px));
        end
    end

% Truncated signed distance function
    function [f] = TSDF(d)
        if length(d)~= 1
            disp('flag 1')
        end
        slope = 1/0.2; 
        f = sign(d)*min([slope*abs(d),1]);
    end

disp(['   Voxels are filled in ',num2str(t),' seconds'])

end



