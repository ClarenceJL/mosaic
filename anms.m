function [x, y, rmax] = anms(cimg, max_pts)
%% Adaptive Non-Maximal Suppression
% INPUT:
% cimg - corner strength map
% max_pts - number of corners desired
% OUTPUT:
% x, y - coordinates of corners
% rmax - suppression radius used to get max_pts corners  
%
% Jiani Li, Oct. 2016

RINF = 1e+10;

[height,width] = size(cimg);
n = height * width;

%% Find maximum radius for each point
[cmap_sorted,cp_ind] = sort(cimg(:));
% neglect small corner responses
m = min([max_pts*10,height*width]);   % keep the largest m pixels
cmap_sorted = cmap_sorted(end-m+1:end);
cp_ind = cp_ind(end-m+1:end);


cp_x = ceil(cp_ind/height);      % mx1 x-coordinate of points
cp_y = cp_ind - (cp_x-1)*height; % mx1 y-coordinate of points

bigcpX = repmat(cp_x,1,m);      % m*m
bigcpY = repmat(cp_y,1,m);      % m*m
Dist = sqrt((bigcpX-bigcpX').^2 + (bigcpY-bigcpY').^2);
mask = triu(ones(m,m),0);
Dist = Dist.*(1-mask) + mask*RINF;
[rmap,~] = min(Dist);   % 1*m return the smallest distance between p and 
                        % any pixel larger thatn p (or the largest radius
                        % around p where all pixels inside the circle are 
                        % smaller than p)

%% sort rmap and find the top ones
[sorted_rmap,p_ind] = sort(rmap);

rmax = sorted_rmap(end-max_pts+1);
start_ind = m-max_pts+1;
% handle border situation
if sorted_rmap(end-max_pts) == rmax
    while start_ind <= m
        start_ind = start_ind + 1;
        if sorted_rmap(start_ind) > rmax
            rmax = sorted_rmap(start_ind);
            break;
        end
    end
end
ip_ind = p_ind(start_ind:end);   % largest pixel index (in cp_ind!)
x = cp_x(ip_ind);
y = cp_y(ip_ind);

end
