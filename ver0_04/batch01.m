% batch1.m - displays animation of a Stewart Platform with fixed actuators,
% moving along inverted sphere meridian

% Definition of fixed actuators model
H = 26;
Rb = 7.6; Rp = 3; Thetab = 30/180*pi; Thetap = 0/180*pi;
model = SPModel( Rb, Rp, Thetab, Thetap );
l = SPInverse( model, eye(3), [0,0,H]');
model = [model , [ l ; zeros(2,6)] ];
 
% Plotting in neutral position
H0 = 5;
SPPlot( model , eye(3), [0;0;H+H0]);
axis('square');
axis([-20 20  -20 20   0 40]);
xlabel('x [cm]'); ylabel ('y [cm]'); zlabel ('z [cm]');
title('Stewart paltform with fixed actuators in neutral position');
 
% 

R = 10; h = H + 10;
elevation = -pi/4:-0.1:-3*pi/4; 
azimuth = pi/4*ones(size(elevation));
x = SPchaserPosition(R, h, azimuth, elevation);
for i = 1: length(azimuth),
	Rot = rotvec([sin(azimuth(i)) -cos(azimuth(i)) 0]', pi/2+elevation(i) );
	SPPlot( model, SPtrpy2tr(x(:,i)) );
%	SPPlot( model, transl(x(:,i)) * Rot );
	axis('square');
	axis([-20 20  -20 20  -10 40]);
	view(45+90,0); 
%	view(2);
%        view(3);
	xlabel('x [cm]'); ylabel ('y [cm]'); zlabel ('z [cm]');
	drawnow;
end;

