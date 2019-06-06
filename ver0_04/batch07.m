% batch7 - calculates angular error for a SPFA
%      errorAngle = batch7(xd,xa)    or   errorAngle = batch7
%
%
%      xd - 6xN matrix with desired trajectory (see SPchaserPosition)
%      xa - 6xN matrix with actual trajectory (see batch6)
%          if xd and xa are omitted, the function generates them by calling:
%	       [xd,xa] = batch6(0.01, 60, -60, pi/4)
%      errorAngle - 1xN matrix with values of angles between the vectors
%	   perpendicular to the platforms of desired and actual manipulator. 
%	   The vectors are positioned along the z-axes in the platform 
%	   coordinate system.
%
%

function errorAngle = batch7(x,y)

if nargin == 0,
   % Calculate desired trajectory 'x' and actual trajectory 'y'
   [x,y] = batch6(0.01, 60, -60, pi/4);
end;

n = size(x,2);
errorAngle = zeros(1,n);


for i = 1:n,

   Rotx = SPtrpy2tr(x(:,i));
   Roty = SPtrpy2tr(y(:,i));
   vx = Rotx * [ 0;0;1;0 ];    % rotate unit vector along z-axis
   vy = Roty * [ 0;0;1;0 ];    % rotate unit vector along z-axis
   
   errorAngle(i) = acos(vx' * vy); % cosine of the angle between two unit 
			       % vectors is equal to their dot product
end; %for

disp(sprintf('Maximal angular error=%f deg',180/pi*max (errorAngle)));
disp(sprintf('Average angular error=%f deg',180/pi*mean(errorAngle)));


return
