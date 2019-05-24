function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = [];
A = [];
for i = 1:4
    vid = video_pts(i,:);
    logo = logo_pts(i,:);
    ax = [-vid(1) -vid(2) -1 0 0 0 logo(1)*vid(1) logo(1)*vid(2) logo(1)];
    ay = [0 0 0 -vid(1) -vid(2) -1 logo(2)*vid(1) logo(2)*vid(2) logo(2)];
    A = [A; ax];
    A = [A; ay];
end

[U,S,V] = svd(A);
H = reshape(V(:,end),[3,3]);
H = H';
end
