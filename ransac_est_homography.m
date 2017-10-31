function [H, inlier_ind] = ransac_est_homography(x1, y1, x2, y2, thresh)
%% RANSAC homography estimation
% INPUT
% y1, x1, y2, x2 are the corresponding point coordinate vectors Nx1 such
% that (y1i, x1i) matches (x2i, y2i) after a preliminary matching
% thresh is the threshold on distance used to determine if transformed
% points agree
% OUTPUT
% H - the 3x3 matrix computed in the final step of RANSAC
% inlier_ind - the nx1 vector with indices of points in the arrays x1, y1,
% x2, y2 that were found to be inliers

N = length(x1);

% number of iterations
p = 0.99;
e = 0.20;    % adjust this value
s = 4;
nitr = log(1-p)/log(1-(1-e)^s); 

%T = (1-e)*N;

max_inlier_num = 0;


for i = 1:10*nitr
    r = randperm(N);
    sample_ind = r(1:s); % randomly pick 4 sample points
    
    H_i = est_homography(x2(sample_ind),y2(sample_ind),x1(sample_ind),y1(sample_ind));

    [x2_est,y2_est] = apply_homography(H_i,x1,y1);
    
    p2 = [x2 y2];
    p2_est = [x2_est y2_est];
    
    SSD_score = sum((p2_est-p2).^2,2); % N*1
    
    inlier_ind_i = SSD_score < thresh;
    if max_inlier_num < sum(inlier_ind_i)
        max_inlier_num = sum(inlier_ind_i);
        inlier_ind = inlier_ind_i;

    end
end

% re-calculate H using all inliers
H = est_homography(x2(inlier_ind),y2(inlier_ind),x1(inlier_ind),y1(inlier_ind));

end