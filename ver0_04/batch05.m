% batch5.m - plots inverted stewart platform and a sphere


plotting = 'no plotting';
psib = 30; psip = 0; 

figure(1); whitebg('white');

% Definition of fixed actuators model
% all dimensions in centimeters
H = 26;	   % height in neutral position
Rb = 7.6; Rp = 3; Thetab = psib/180*pi; Thetap = psip/180*pi;
model = SPModel( Rb, Rp, Thetab, Thetap, H );
 

[b,p,l] = SPmodel2bp(model);


% Generate trajectory
R = 10; h = H + R;
da = 0.20;

% Trajectory along a meridian of a sphere
elevation = -pi/3; 
azimuth = 0;
x = SPchaserPosition(R, h, azimuth, elevation);
Rot = SPtrpy2tr(x);
SPPlot(model, Rot);  drawnow;



r = 5;      % heart radius;
[sx,sy,sz] = sphere(10);
hs = surface( r*sx+0, r*sy+0, r*sz+H+R);
cm = colormap; cm = ones(size(cm)); cm (1,1:3) = [1 1 1];
colormap(cm);
axis([-25 25 -25 25 0 50]);
view(+10, 20);


% Trajectory along a meridian of a sphere
elevation = -pi/4:-da:-3*pi/4; 
azimuth = 0*pi/4*ones(size(elevation));
x = SPchaserPosition(R, h, azimuth, elevation);
hold on; hp = plot3(x(1,:),x(2,:),x(3,:),'x'); hold off 
