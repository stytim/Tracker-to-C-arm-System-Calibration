% batch4.m - sensitivity analysis for Stewart platform with fixed actuators
%      After initialization and trajectory generation
%      - inverse kinematics is calculated
%      - error is introduced into the actuator height values
%      - forward kinematics is calculated
%      - pose-error is obtained
%
%      batch4(dL, psib, psip, plotting)
%      dL - actuator positioning error
%      psib - angle between first two joints of the SP base in degrees 
%      psip - angle of SP platform
%      plotting - if equal to 'p' SP will be plotted
%      See also: SPModel
%

function image = batch4(dL, psib, psip, plotting)

if nargin < 4,   plotting = 'no plotting'; end;
if nargin < 3,   psib = 30; psip = 0;  end;

% Definition of fixed actuators model
% all dimensions in centimeters
H = 26;	   % height in neutral position
Rb = 7.6; Rp = 3; Thetab = psib/180*pi; Thetap = psip/180*pi;
model = SPModel( Rb, Rp, Thetab, Thetap, H );
SPPlot(model, transl([0 0 H]));  drawnow;

% Actuator step size:
[b,p,l] = SPmodel2bp(model);

if nargin == 0,
   dL = mean(l)/200;
end;

% Generate trajectory
R = 10; h = H + R;
da = 0.05;

% Trajectory along a meridian of a sphere
elevation = -pi/4:-da:-3*pi/4; 
azimuth =  pi/4*ones(size(elevation));

% Trajectory along a parallel of a sphere
%azimuth = 0:da:2*pi;
%elevation = -pi/4*ones(size(azimuth));

x = SPchaserPosition(R, h, azimuth, elevation);
% allocate space for actual trajectory
y = zeros(size(x));

% What will a camera on the platform see?
% Tracking the point on the surface of the sphere (heart)
% The point is viewed at pixel (0,0) by the ideal platform and camera
r = 5;      % heart radius;
image = zeros(2, size(x,2));   % camera image
points = SPchaserPosition(r, h, azimuth, elevation); % points to track
points = points(1:3,:);

% Initializing maximal/minimal errors, heights, velocities etc.
maxerrors = zeros(1,length(azimuth));
maxh = 0; minh = 10;	   % max and min actuator positions
maxv = 0; hold = SPInvFA(model, SPtrpy2tr(x(:,1)) ); % max speed
minang = 1.0;  % sine of minimal angle between a leg and platform


% Move along trajectory
for i = 1: length(azimuth),
   Rot = SPtrpy2tr(x(:,i));

   % Inverse kinematics
   heights = SPInvFA(model, Rot );
   maxh = max(maxh, max(heights));
   minh = min(minh, min(heights));
   maxv = max(maxv, max(abs(heights-hold)/da)); hold = heights;

   % Dot product of leg-vector and a vector normal to the platform
   %   is proportional to the sine of the angle between leg and platf.
   minang= min( minang...
       , min( (Rot*[0;0;1;0])' * (Rot*p-b-[0;0;1;0]*heights) ./ l ));


   % Error in actuator heights due to finite step size
   heights2 = round(heights/dL)*dL;

   % Forward kinematics with new actuator heights
   Rot2= SPforFA( model, heights2, Rot, dL/1000);

   % Save the new trajectory
   y(:,i) = SPtr2trpy(Rot2);

   % What will a camera on the platform see?
   image(:,i) = SPperspective(points(:,i), 1.0, Rot2);
   

if ( plotting(1) == 'p' ),
   figure(1);
   SPPlot(model, Rot); axis('square'); 
   view(0,0); axis([-10 10 -10 10 0 35]);
   title(sprintf('%d',i));
   figure(2);
   SPPlot(model, Rot2); axis('square'); 
   view(0,0); axis([-10 10 -10 10 0 35]);
   title(sprintf('%d',i));
   drawnow;
end;

%  I think this is a BIG BUG !!!
%   maxerrors(i) = max(abs(heights - heights2));

   maxerrors(i) = sqrt(sum( (x(1:3,i)-y(1:3,i)).^2 )); %positional error

%   disp( sqrt(sum( (heights - heights2).^2 )) );  
%   disp( sqrt(sum( (x(1:3,i)-y(1:3,i)).^2 )));   % position error
end;



disp('----------------------------------');
disp(sprintf('given actuator tolerance: %f cm' , dL));
disp(sprintf('actuator height span: %f cm' , maxh-minh));
disp(sprintf('actuator maximal speed: %f cm/rad' , maxv));
disp(sprintf('minimal leg-platform angle: %f deg' , 180/pi*asin(minang)));
disp(' ');
disp(sprintf('maximal positional error: %f mm' , 10*max(maxerrors) ));
disp(sprintf('max  orientational error: %f mm' , 10*max(sqrt(sum(image.^2))) ));

plot(image(1,:), image(2,:)); axis('square'); 
title('Trajectory of the point in the camera image');
xlabel('x [cm]'); ylabel('y [cm]');


% Result:
% maximal position error is always less than 0.5*dl !
