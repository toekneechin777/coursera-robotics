function [ warped_pts ] = warp_pts( video_pts, logo_pts, sample_pts)
% warp_pts computes the homography that warps the points inside
% video_pts to those inside logo_pts. It then uses this
% homography to warp the points in sample_pts to points in the logo
% image
% Inputs:
%     video_pts: a 4x2 matrix of (x,y) coordinates of corners in the
%         video frame
%     logo_pts: a 4x2 matrix of (x,y) coordinates of corners in
%         the logo image
%     sample_pts: a nx2 matrix of (x,y) coordinates of points in the video
%         video that need to be warped to corresponding points in the
%         logo image
% Outputs:
%     warped_pts: a nx2 matrix of (x,y) coordinates of points obtained
%         after warping the sample_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% Complete est_homography first!
[ H ] = est_homography(video_pts, logo_pts);

% YOUR CODE HERE

warped_pts = [];

for i=1:size(sample_pts,1)
   d = sample_pts(i,:);
   new_dx = (H(1,1)*d(1) + H(1,2)*d(2) + H(1,3))/(H(3,1)*d(1) + H(3,2)*d(2) + H(3,3));
   new_dy = (H(2,1)*d(1) + H(2,2)*d(2) + H(2,3))/(H(3,1)*d(1) + H(3,2)*d(2) + H(3,3));
   new_d = [new_dx, new_dy];
   warped_pts = [warped_pts; new_d];
end


end

