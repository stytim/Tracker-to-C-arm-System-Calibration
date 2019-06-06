% SPchaserPosition - returns position on the sphere 
%
%	x = chaserPosition(R, H, azimuth, elevation)
%	R - radius of the sphere
%	H - height of the center of the sphere
%	azimuth, elevation - angles determining position of the sphere
%	x - 6xN matrix of the x,y,z,roll,pitch,yaw coordinates columnwise, 
%	   N is the length of azimuth/elevation vectors
%	
%	coordinates of the center of the sphere are [0,0,H]'
%
%      See also: SPtrpy2tr, SPtr2trpy

function x = SPchaserPosition(R, H, azimuth, elevation)


if size(azimuth,1) > size(azimuth,2),
	azimuth = azimuth';  elevation = elevation';
end;

N = size(azimuth,2);


x =     [0;0;H;0;0;0] * ones(1,N)     ...
 + R * [ cos(azimuth).*cos(elevation) ...
       ; sin(azimuth).*cos(elevation) ... 
       ; sin(elevation)               ...
       ; zeros(3,N)   ] ;

for i = 1:N,
   Rot = rotvec([sin(azimuth(i)) -cos(azimuth(i)) 0]', pi/2+elevation(i) );
   x(:,i) = x(:,i) + SPtr2trpy(Rot);
end;

return
