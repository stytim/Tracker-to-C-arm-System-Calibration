% SPtr2trpy - converts rotation matrix to a vector ov translations and rotations
%
%      x = SPtr2trpy(R)
%	 
%      R - 4x4 homogeneous transformation matrix
%      x - 6x1 vector
%      x = [ Tx;Ty;Tz; roll;pitch;yaw ]
%      Tx;Ty;Tz - translation vector
%      roll;pitch;yaw - angles for roll-pitch-yaw transform
%
%      See also rpy2tr, tr2rpy, transl,  SPtrpy2tr

function x = SPtr2trpy(R)

x = [ transl(R) ; tr2rpy(R)' ];

return;
