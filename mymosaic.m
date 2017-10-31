function [img_mosaic] = mymosaic(img_input)
%% Mosiac
% INPUT
% img_input - a cell array of color images (HxWx3 uint8 values in the
% range [0,255]). In this function, we assume that these images are
% spatially sequential 
%
% OUTPUT
% img_mosaic - the output mosaic
%
% Reference:
% https://www.mathworks.com/help/vision/examples/feature-based-panoramic-image-stitching.html
% 
% Jiani Li, Nov. 2016

M = length(img_input); % number of images

fmax = 300;
vis_on = 0; % visualize intermediate results


%% feature detection
img_corners = cell(M,1); % keep corner coordinates of every image
img_desc = cell(M,1); % keep feature descritor of every image
img_h = zeros(M,1);
img_w = zeros(M,1);

for i = 1:M
    [img_h(i),img_w(i),nchannel] = size(img_input{i});
    if nchannel > 1
        grayimg = rgb2gray(img_input{i});
    else
        grayimg = img_input{i};
    end
    
    % harris corner detection
    cornermap = corner_detector(grayimg);
    % sparsify
    [cx, cy, ~] = anms(cornermap, fmax);
    img_corners{i} = [cx cy];
    

    if vis_on
        visual_feat(img_input{i},cornermap,cy,cx);
    end
    
    % feature extraction
    img_desc{i} = feat_desc(grayimg, cx, cy);   

end


%% feature match
img_match = cell(M,1);  % the last cell is null
H = cell(M,1);     % the last element is all 0
T = cell(M,1);
thresh = 4;
H{1} = eye(3);
T{1} = H{1};

for i = 2:M
    j = i-1;
    % match the feature between two consequtive images
    % notice that the H is the homogenious transformation matrix from the
    % SECOND image to the FIRST image: X_(i-1)=H_i*X_i
    
    img_match{i} = feat_match(img_desc{i}, img_desc{j});
     
    % find transformation H
    matched_i = img_corners{i}(img_match{i}~=-1,:);
    matched_j = img_corners{j}(img_match{i}(img_match{i}~=-1),:);
    
    [H{i}, inliers] = ransac_est_homography(matched_i(:,1), matched_i(:,2), ...
                        matched_j(:,1), matched_j(:,2), thresh);
     
    % transformation from n to 1
    T{i} = T{i-1}*H{i};
    
    if vis_on
        visual_match(img_input{i},img_input{j},matched_i(:,1), matched_i(:,2), ...
                        matched_j(:,1), matched_j(:,2), inliers); 
    end

end

%% image stitching

c = ceil(M/2);  %take center image as the base image
% change the transformations, so they are from (any) i to c
Tc_inv = inv(T{c});
for i = 1:M
    T{i} = Tc_inv*T{i};
end

% find the x,y range for each image after transformation

xMin = 1e+8; yMin = 1e+8;
xMax = -1e+8; yMax = -1e+8;
for i = 1:M
    [X_trans, Y_trans] = apply_homography(T{i},[1;1;img_w(i);img_w(i)],[1;img_h(i);1;img_h(i)]);
    xMin = min([X_trans;xMin]); yMin = min([Y_trans;yMin]);
    xMax = max([X_trans;xMax]); yMax = max([Y_trans;yMax]);
end
xMin = floor(xMin); yMin = floor(yMin);
xMax = ceil(xMax); yMax = ceil(yMax);

% initialize panorama
pwidth = xMax-xMin+1;
pheight = yMax-yMin+1;
img_mosaic = zeros(pheight,pwidth,nchannel);
mask_panorama = false(pheight,pwidth); % mask for panorama image

% stitch images
blending_method = 2; 

for i = 1:M   %image to be transformed to world-frame (c-frame)
    
    % warp current image to the panorama frame
    [warped_img,mask] = image_warping(img_input{i},T{i},pwidth,pheight,xMin,yMin);

    
    switch blending_method
        case 1
            % method1: average blending
            % calculate alpha
            blending_alpha = ones(pheight,pwidth);
            blending_alpha(mask) = 0;
            blending_alpha(mask_panorama & mask) = 0.5;
        case 2
            % method2: smooth blending
            % In this method we consider a distance weighted alpha
            bdist_panorama = bwdist(~mask_panorama,'cityblock');
            bdist_warped_img = bwdist(~mask,'cityblock');
            blending_alpha = bdist_panorama./(bdist_panorama+bdist_warped_img+1e-6);
        otherwise
            % default method: overlay warped_img on current panorama
            blending_alpha = ones(pheight,pwidth);
            blending_alpha(mask) = 0;            
    end
    
    % blending
    for j = 1:nchannel
        img_mosaic(:,:,j) = warped_img(:,:,j).*(1-blending_alpha) + ...
                            img_mosaic(:,:,j).*blending_alpha;
    end
    
    mask_panorama = mask_panorama | mask; % update mask_panorama
       
end

img_mosaic = im2uint8(img_mosaic);


close all

end


