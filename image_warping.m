function [warped_I, mask] = image_warping( I, T, morphed_w, morphed_h, morphed_minx, morphed_miny)
%% IMAGE WARPING
% INPUT
% I - input image
% OUTPUT
% warped_I - morphed image shown in panorama view
% mask - mask of morphed image in panorama image
%
% Jiani Li, Nov. 2016


[h,w,nchannel] = size(I);
warped_I = zeros(morphed_h,morphed_w,nchannel);
mask = false(morphed_h,morphed_w);


%% find pixel correspondence
[xdest, ydest] = apply_homography(T, [1; 1; w; w], [1; h; 1; h]);
leftbound = floor(min(xdest)); rightbound = ceil(max(xdest));
upbound = floor(min(ydest)); downbound = ceil(max(ydest));

[Xdest, Ydest] = meshgrid(leftbound:rightbound,upbound:downbound); % X,Y in center image frame
Xdest = Xdest(:);
Ydest = Ydest(:);
[Xsrc_prime, Ysrc_prime] = apply_homography(inv(T), Xdest, Ydest);


nosrc_points = Xsrc_prime < 1 | Xsrc_prime > w | Ysrc_prime < 1 | Ysrc_prime > h;

Xdest(nosrc_points) = []; Ydest(nosrc_points) = [];
Xsrc_prime(nosrc_points) = []; Ysrc_prime(nosrc_points) = [];

Xdest = Xdest - (morphed_minx - 1);  % shift to world (panorama) frame 
Ydest = Ydest - (morphed_miny - 1);  % shift to world (panorama) frame 
Cdest = (Xdest-1) * morphed_h + Ydest;

mask(Cdest) = 1;

%% bilinear interpolation
[X,Y] = meshgrid(1:w,1:h);

for j = 1:nchannel
    morphed_c = zeros(morphed_h,morphed_w);
    Ic = im2double(I(:,:,j));
    % do bilinear interpolation here
    morphed_c(Cdest) = interp2(X,Y,Ic,Xsrc_prime,Ysrc_prime);
    
    warped_I(:,:,j) = morphed_c;
end

end

