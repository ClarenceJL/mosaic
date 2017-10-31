function [descs] = feat_desc(img, x, y)
%% generate feature descriptor 
% INPUT:
% img - double (height)x(width) array (grayscale image) with values in the
% range 0-255
% x - nx1 vector representing the column coordinates of corners
% y - nx1 vector representing the row coordinates of corners
% OUTPUT:
% descs - 64xn matrix of double values with column i being the 64 dimensional
%         descriptor computed at location (xi, yi) in im

n = length(x);
patch = zeros(40,40);
descs = zeros(64,n);

padded_img = padarray(img,[20 20],0,'both');
px = x + 20; py = y + 20;
lbound = px - 20; rbound = px + 20;
ubound = py - 20; dbound = py + 20;

for i = 1:n
    patch = padded_img(ubound(i):dbound(i),lbound(i):rbound(i));
    % blur the patch
    patch = imgaussfilt(patch); % adjust sigma of Gaussian here
    % sub-sampling
    feature = patch([3 8 13 18 23 28 33 38],:);
    feature = feature(:,[3 8 13 18 23 28 33 38]);
    % normalization
    feature = double(feature(:));
    feature = feature - mean(feature);
    feature = feature/std(feature);
    descs(:,i) = feature;
end




end