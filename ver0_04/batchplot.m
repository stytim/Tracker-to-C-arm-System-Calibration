% batchplot.m - plots Stewart Platforms (regular and with fixed actuators), in
% neutral position and at the end of the trajectory

figure(1);

% Definition of the model
H = 26;
Rb = 7.6; Rp = 3; Thetab = 30/180*pi; Thetap = 0/180*pi;
[base,platform] = SPModel( Rb, Rp, Thetab, Thetap );
model = [base, platform];


% Plotting in neutral position
SPPlot( model , eye(3), [0;0;H]);
axis('square'); 
axis([-15 15 -15 15 0 30]);
xlabel x; ylabel y; zlabel z;
title('Stewart paltform in neutral position');

figure(2);


% Plotting in end-of-trajectory position
Rot = rotationMatrix(0, -pi/4,0);
R = 10;	       %trajectory radius
Ax = R*cos(pi/4); Az = -R*sin(pi/4);
SPPlot( model , transl([Ax;0; H+R+Az])*Rot );
view(0,0);
axis([-35/2 35/2 -35/2 35/2 0 35]);
axis('square'); 
xlabel x; ylabel y; zlabel z;
title('Position at the end of the trajectory');


figure(3);

% Definition of fixed actuators model
H = 26;
Rb = 7.6; Rp = 3; Thetab = 30/180*pi; Thetap = 0/180*pi;
model = SPModel( Rb, Rp, Thetab, Thetap, H );

% Plotting in neutral position
H0 = 5;
SPPlot( model , eye(3), [0;0;H+H0]);
axis('square'); 
axis([-20 20  -20 20   0 40]);
xlabel('x [cm]'); ylabel ('y [cm]'); zlabel ('z [cm]');
title('Stewart paltform with fixed actuators in neutral position');

figure(4);


% Plotting in end-of-trajectory position
Rot = rotationMatrix(0, -pi/4,0);
R = 10;	       %trajectory radius
Ax = R*cos(pi/4); Az = -R*sin(pi/4);
SPPlot( model , transl([Ax;0; H+R+Az+H0])*Rot);
axis([-20 20  -20 20   0 40]);
axis('square'); 
xlabel('x [cm]'); ylabel ('y [cm]'); zlabel ('z [cm]');
title('Position at the end of the trajectory');

view(0,0);

