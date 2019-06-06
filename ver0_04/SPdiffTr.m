function dR = SPdiffTr(x)
% SPdiffTr - retunrs derivative of a rotation matrix 
%      wrt translation and roll-pitch-yaw angles
%
%      dR = SPdiffTr(x)
%
%      x - 6x1 vector containing translation and rotation parameters
%      x = [ Tx Ty Tz roll pitch yaw ]'
%      
%      dR - 24x4 matrix, divided into 6 4x4 matrices, each of them is 
%	   derivative of rotation matrix wrt one element of x vector
%
%      Rotation matrix corresponding to vector x can be obtained by:
%	 R =  rotationMatrix( x )

cax = cos(x(6)); sax = sin(x(6));
cay = cos(x(5)); say = sin(x(5));
caz = cos(x(4)); saz = sin(x(4));

dR = [
% dR/dTx
0 0 0 1
0 0 0 0
0 0 0 0
0 0 0 0

% dR/dTy
0 0 0 0 
0 0 0 1
0 0 0 0
0 0 0 0

% dR/dTz
0 0 0 0
0 0 0 0 
0 0 0 1
0 0 0 0

% dR/droll
-cay*saz  -cax*caz-sax*say*saz  caz*sax-cax*say*saz  0
 cay*caz   caz*sax*say-cax*saz  cax*caz*say+sax*saz  0
 0                 0            0                    0
 0                 0            0                    0

% dR/dpitch
-caz*say        cay*caz*sax   cax*cay*caz   0
-say*saz        cay*sax*saz   cax*cay*saz   0
-cay           -sax*say      -cax*say       0
 0              0                   0       0

% dR/dyaw
0  cax*caz*say+sax*saz      -caz*sax*say+cax*saz   0
0 -caz*sax+cax*say*saz      -cax*caz-sax*say*saz   0
0  cax*cay                  -cay*sax               0
0  0                         0                     0

];


return
