% batch13 - This file is used for verifying algorithms for camera calibration
%           and hand-eye (gripper-camera) calibration
% 
% Notation: wHc - pose of camera frame (c) in the world (w) coordinate system
%                 .. If a point coordinates in camera frame (cP) are known
%                 ..     wP = wHc * cP
%                 .. we get the point coordinates (wP) in world coord.sys.
%                 .. Also refered to as transformation from camera to world

gHc = transl([0;0;50])*quat2rot(0.2*randn(3,1));
cHg = inv(gHc);    
                            
bHw = [                    % base to world transformation
      -1    0    0    0
       0    1    0    0
       0    0   -1  580
       0    0    0    1
];
wHb = inv(bHw);            % Pose of the base in the world coordinate system

% Create M random poses of the gripper:
% Platform is positioned at distance D=360 from base ...
M = 10;
[ bHg, gHb ] = batch12(M, 360, 50);


bHc = zeros(4,4,M);
wHc = zeros(4,4,M);
cHw = zeros(4,4,M);           % World system relative to the camera
                              % .. or transformation from camera to world
                              % .. => a point in world xw 
                              %    .. will be positioned at xc = Hcw*xw relative to the camera
                              % World system contains calibration block
for i=1:M,                    % For all gripper positions, calculate camera position
   bHc(:,:,i) = bHg(:,:,i)*gHc;
   wHc(:,:,i) = wHb * bHg(:,:,i) * gHc; % 
   cHw(:,:,i) = inv(wHc(:,:,i)); % Hcw = Hcg*Hgb*Hbw  
end;


gHc_calculated = handEye(bHg, wHc);

disp('error = ');
disp(gHc - gHc_calculated);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
