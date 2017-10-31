%% test script for image mosaic
% Jiani Li, Nov. 2016

clear
clc
close all

% load data
path = 'input imgs/';
files = dir([path '*.jpg']);
M = size(files,1);
images = cell(M,1);
for i = 1:M
    images{i} = imread([path files(i).name]);
end

%%
panorama = mymosaic(images);

figure
imshow(panorama);
imwrite(panorama,'panorama.png');

