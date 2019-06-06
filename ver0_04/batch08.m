% batch8 - retruns positional and orientational error
%	   [Epos, Ero] = batch8(dL, psib, psip, rpy)
%
%	   dL - accuracy of the actuators, default 0.01
%	   psib, psip - angles determining base and platform shape (deg)
%	   rpy - rotation of the platform (roll-pitch-yaw)
%	   Epos - positional error
%	   Ero  - rotational error	   
%	   
%      The error is calculated along a sphere with a radius of 10cm, the
%      platform keeps a constant angle wrt. base as determined by 'rpy'.
%      The output is errors at a rectangular grid, as shown in the plot



function [Epos, Eor] = batch8(dL, psib, psip, rpy)

if nargin == 0, dL = 0.01; end; % all units in centimeters
if nargin < 3,   psib = 30; psip = -60;  end; % hex-base, triangle platform 
if nargin < 4, rpy = [0;0;0]; end;

% angle of the platform normal with respect to the z-axis
pAng = 180/pi*asin(norm( ...
       cross([0;0;1], [eye(3) zeros(3,1)]*SPtrpy2tr([0;0;0;rpy])*[0;0;1;0] ) ));

% Definition of fixed actuators model
% all dimensions in centimeters
H = 26;	   % height in neutral position
Rb = 7.6; Rp = 3; Thetab = psib/180*pi; Thetap = psip/180*pi;
model = SPModel( Rb, Rp, Thetab, Thetap, H );
[b,p,l] = SPmodel2bp(model);

% Make rectangular grid
L = floor(10/sqrt(2))+1;
[x,y] = meshgrid( -L:2:L, -L:2:L );
z = H * ones(size(x));
z = H + sqrt(10^2 - x.^2 - y.^2);
t = prod(size(x));     % otal number of points

%calculate errors
errors = SPFAerror( model, [ x(:)' ; y(:)' ; z(:)' ; rpy(:) * ones(1,t) ] );

ex = reshape(errors(1,:), size(x,1), size(x,2) );
ey = reshape(errors(2,:), size(x,1), size(x,2) );
ez = reshape(errors(3,:), size(x,1), size(x,2) );
er = reshape(errors(4,:), size(x,1), size(x,2) );
ep = reshape(errors(5,:), size(x,1), size(x,2) );
ey = reshape(errors(6,:), size(x,1), size(x,2) );

colormap(zeros(64,3));
Epos = dL*sqrt(ex.^2+ey.^2+ez.^2); % Error as the sqrt of sum of squares
subplot(121);
mesh(x,y,Epos);
title( sprintf(...
   'Positional error        platform inclination=%2.0f[deg]',pAng));
xlabel('x[cm]'); ylabel('y[cm]'); zlabel('error[cm]');
%v = axis; axis([-L L -L L 0 v(6)]);
axis([-L L -L L 0.01 0.018]);

Eor  = dL*sqrt(er.^2+ep.^2+ey.^2); % Error as the sqrt of sum of squares
subplot(122);
mesh(x,y,Eor/pi*180);
title( sprintf(...
   'Orientational error') );
xlabel('x[cm]'); ylabel('y[cm]'); zlabel('error[deg]');
%v = axis; axis([-L L -L L 0 v(6)]);
axis([-L L -L L 0.2 0.351]);



return

