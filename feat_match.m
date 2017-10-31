function [match] = feat_match(descs1, descs2)
%% feature(descriptor) matching
% INPUT
% descs1 - a 64x(n1) matrix of double values
% descs2 - a 64x(n2) matrix of double values
%
% OUTPUT 
% match - n1x1 vector of integers where match[i] points to the index of the
% descriptor in p2 that matches with the descriptor p1(:,i).
% If no match is found, match[i] = -1
%
% Jiani Li, Nov. 2016

threshold = 0.6;

DMAX = 1e+10;

n1 = size(descs1,2);
n2 = size(descs2,2);
match = zeros(n1,1)-1; % default -1

% brute-force search
% SSD_map = zeros(n1,n2); % distance map
% for j = 1:n2
%     SSD_map(:,j) = sum(bsxfun(@minus,descs1,descs2(:,j)).^2);
% end
% 
% SSD_map = SSD_map';     % n1xn2 -> n2xn1
% 
% [mindist,nearest] = min(SSD_map);   % nearest neighbor distance
% max_ind = n2*(0:n1-1) + nearest;
% SSD_map(max_ind) = DMAX;
% nextmindist = min(SSD_map);     % next nearest neighbor distance

% k-d tree search
tree = KDTreeSearcher(descs2');   
[nearests,mindists] = knnsearch(tree,descs1','K',2);

% decide whether is a match or not
score = mindists(:,1)./mindists(:,2);
matched_points = score < threshold;
match(matched_points) = nearests(matched_points,1);


end

