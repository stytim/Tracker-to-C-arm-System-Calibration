% batch12 - creates M random poses of the gripper.
% 
%     [ bHg, gHb ] = batch16(M, D, R1, H, R2)
%
%    M - number of random poses of the gripper
%    D - Platform is positioned at distance D from base ...
%    R1- Platform is positioned within circle of diameter R1
%    H - Distance from base to target plane
%    R2- Target is positioned within circle of diameter R2
%    bHg-poses of gripper frame in base coord.sys. (4x4xM)
%    gHb-poses of base relative to gripper frame (4x4xM)
%    
%    See also: handEye
%
function [ bHg, gHb ] = batch12(M, D, R1, H, R2)

% Default arguments
if nargin < 1, M  =  1; end;
if nargin < 2, D  =350; end;
if nargin < 3, R1 =D/4; end;
if nargin < 4, H  =2*D; end;
if nargin < 5, R2 =2*R1; end;

bHg = zeros(4,4,M);           % Hbg = pose of the gripper frame wrt. base
gHb = zeros(4,4,M);           % Hbg = pose of the base wrt. gripper frame
for i=1:M,
   r = R1 * rand(1);          % Platform is positioned within a circle ..
   a = 2*pi*rand(1);          % .. of diameter R1
   bP= [ r*cos(a) ; r*sin(a) ; D ; 1]; % .. relative to the robot base
   
   r = R2 * rand(1);          % Target is positioned within a circle ..
   a = 2*pi*rand(1);          % .. of diameter R2
   bT= [ r*cos(a) ; r*sin(a) ; H ; 1]; % .. at height H from base
   
   d = bT(1:3)-bP(1:3);       % direction from platform towards base
   v = cross([0;0;1], d );    % rotation around vector v... 
   a = asin(norm(v)/norm(d)); % .. by angle a ..
   R = rotvec(v/norm(v), a);  % .. will face platform towards the target
   
   bHg(:,:,i) = transl(bP) * R;
   gHb(:,:,i) = inv(bHg(:,:,i));
   
end;

