% SPtrInv - inverse of homogeneous transformation
%
%      Rinv = SPtrInv(R)
%      
%      R - 4x4 transformation matrix
%      Rinv - 4x4 inverse transformation
%      
%           r11 r12 r13 tx
%           r21 r22 r23 ty
%      R =  r31 r32 r33 tz
%	         0   0   0   1
%
%      where 3x3 matrix r(i,j) is rotation matrix and 
%      tx,ty,tz define translation vector
%
%      See also: rotx, roty, rotz, SPtr2trpy, SPtrpy2tr


function Rinv = SPtrInv(R)

%Rinv = [ R(1:3,1:3)' , -(R(1:3,1:3)' * R(1:3,4)) ...
%       ; 0  0   0                 1 ];

Rinv = inv(R);

return
