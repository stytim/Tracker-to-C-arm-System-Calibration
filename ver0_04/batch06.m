% batch6.m - sensitivity analisys
% plots images as seen by camera on the platform
%
%      [x,y,image] = batch6(dL, psib, psip, Azimuth)
%
%      dL - actuator step size
%      psib, psip - base and platform configuration angles in degrees
%      Azimuth - angle in radians along which trajectory is generated
%	   try azimuths for pi/6 + k*pi/3, error is narrower than for
%	   other angles
%      x - desired trajectory
%      y - obtained trajectory 
%      image - series of points as seen by a camera mounted on the platform
%
%      See also: SPModel, SPInvFA

function [x,y,image] = batch6(dL, psib, psip, Azimuth)

if nargin == 0, dL = 0.01; end; % all units in centimeters
if nargin < 3,   psib = 30; psip = -60;  end; % hex-base, triangle platform 
if nargin < 4, Azimuth = 0; end;


whitebg('white');

% Definition of fixed actuators model
% all dimensions in centimeters
H = 26;	   % height in neutral position
Rb = 7.6; Rp = 3; Thetab = psib/180*pi; Thetap = psip/180*pi;
model = SPModel( Rb, Rp, Thetab, Thetap, H );
 

[b,p,l] = SPmodel2bp(model);

%actuator step size
dL = 0.01; % cm


% Generate trajectory
R = 10; h = H + R;
da = pi/512;

% Trajectory along a meridian of a sphere
elevation = -pi/4:-da:-2*pi/4; 
azimuth = Azimuth*ones(size(elevation));
n = length(elevation);

% Desired trajectory
x = SPchaserPosition(R, h, azimuth, elevation);
% allocate space for actual trajectory
y = zeros(size(x));


%Generate testpoints
az = [ 0 0 2*pi/3 4*pi/3 0 2*pi/3 4*pi/3 ];
nt = length(az);
deltaFi = 15*pi/180;  % tracking point azimuth
el = -pi/2 + deltaFi*[ 0 ones(1,3)  2*ones(1,3) ];
r = 0.5*R;

trackPts = SPchaserPosition(r,0*h,az,el);
trackPts = [trackPts(1:3,1:nt); ones(1,nt)];
%Rtrack = SPtrpy2tr(trackPts(:,1));     % referent rotation 
%rotate back
%trackPts = SPtrInv(Rtrack)*[trackPts(1:3,1:nt); ones(1,nt)]; 

% Allocate space for points as viewed by camera
image = zeros(2,n*nt);
error = image;

phaseError = 10*dL*randn(size(SPInvFA(model, eye(4))));

% Desired image
image0 = SPperspective(transl([0,0,H+R])*trackPts ...
	   , 1.0, transl([0,0,H]));

for i = 1:n;   % for all trajectory points
   
   Rot = SPtrpy2tr(x(:,i));

   % Inverse kinematics
   heights = SPInvFA(model, Rot );

   % Error in actuator heights due to finite step size
   heights2 = round((heights + phaseError)/dL )*dL -phaseError;


   % Forward kinematics with new actuator heights
   Rot2= SPforFA( model, heights2, Rot, dL/1000);

   % Save the new point on the actual trajectory
   y(:,i) = SPtr2trpy(Rot2);
   
   trackedPts = SPtrpy2tr([zeros(3,1);x(4:6,i)])*trackPts ...
	      + [0;0;h;0]*ones(1,nt);

   % What will a camera on the platform see?
   t = (i-1)*nt+(1:nt);
   image(:,t) = SPperspective(trackedPts, 1.0, Rot2);
   error(:,t) = image(:,t) - image0;
   
end;

% Plot traces made by tracked points in the image plane
plot(reshape(image(1,:), nt,n)', reshape(image(2,:), nt,n)', 'b');

% Mark the desired positions of tracked points in the image plane
hold on;
a = 0.05; aMark = [ a a ; -a a ; -a -a ; a -a ;  a a ]; 
for i = 1:nt,
   plot(image0(1,i)+aMark(:,1), image0(2,i)+aMark(:,2)); 
end;

hold off;

axis([-0.6 0.4 -0.5 0.5]); axis('square');
xlabel('u [cm]'); ylabel('v [cm]');
title(sprintf('Tracking labels in image plane, trajectory azimuth = %2.0f deg'...
       , Azimuth/pi*180));


disp(sprintf('azimuth     = %f deg', Azimuth/pi*180 ));
disp(sprintf('max x error = %f mm', 10*max(error(1,:)) ));
disp(sprintf('max y error = %f mm', 10*max(error(2,:)) ));
ermax = 10*max(sqrt(sum(error.^2)));
disp(sprintf('max   error = %f mm', ermax ));
% and the rotational error in degrees: tan(errorAngle)=ermax/distanceFromCamera
disp(sprintf('max rot.error=%f deg', 180/pi*atan(ermax/10/5)));


return
