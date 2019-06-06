% SPtrpy2tr - converts vector with translation and roll-pitch-yaw to rotation matrix 
%
%      R = SPtrpy2tr(x)
%	 
%      x - 6x1 vector with translations and roll-pitch-yaw angles 
%      x = [ Tx;Ty;Tz; roll;pitch;yaw ]
%      Tx;Ty;Tz - translation vector
%      roll;pitch;yaw - angles for roll-pitch-yaw transform
%      R - 4x4 homogeneous rotation matrix
%
%      See also rpy2tr, tr2rpy, translate, SPtr2trpy

function R = SPtrpy2tr(x)

R = transl(x(1:3)) * rpy2tr(x(4:6));

return
