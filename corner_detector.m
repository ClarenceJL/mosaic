function [cimg] = corner_detector(img)
%% corner detectiton
% INPUT:
% img  - HxW grayscale image
% OUTPUT:
% cimg - HxW matrix representing corner metricmatrix ([0 1])


cimg = cornermetric(img,'Harris'); % default Harris corner detection 
% rescale the result map
cmax = max(cimg(:));
if cmax ~= 0
    cimg = cimg/cmax;
end

end