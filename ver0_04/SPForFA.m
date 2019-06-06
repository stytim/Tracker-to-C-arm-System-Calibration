% SPForFA - Forward kinematics for Stewart Platform with fixed actuators
%
%	   R = SPForFA( model, heights, Rinitial, tolerance )
%
%      model - 3x12 vector containing coordinates of the Stewart platform joints
%      heights - vector with 6 fixed actuator heights
%      Rinitial - initial estimate of position, 4x4 homogeneous coordinates
%      tolerance - accuracy of calculations, default: mean(lengths)/1000
%      R - 4x4 homogeneous rotation/translation matrix
%
%      see also: SPModel, SPInverse, SPInvFA, SPforward

function R = SPForFA( model, heights, R , tol)

if nargin < 4,
   [b,p,l] = SPmodel2bp(model);
   tol = mean(l)/100;
end;

[b,p,lengths] = SPmodel2bp(model);

if nargin < 3,													% if initial pose is not given
   H = sqrt(lengths(1)^2 ...
            - sum(b(:,1).^2 - p(:,1).^2));	      % calculate default height
   R = [eye(3) [0; 0; (H + mean(heights))]...		% use for initial pose a pose with mean  
       ; 0 0 0           1                 ];      % ... mean heights and no rotation
end;

Rinitial = R;

x = SPtr2trpy(R);
G = zeros(6,6);


while ( 1 ),

   % Inverse kinematics
   q = R*p - b;
   s = sqrt( lengths.^2 - q(1,:).^2 - q(2,:).^2);
   h = q(3,:) - s;
   if any(imag(s)~=0),
       disp('SPforFA: went into singular area. Reverting to original');
       R = Rinitial;
       return
   end;
   
   % difference between desired and actual heights
   dh = heights - h;


   if ( max(abs(dh)) < tol ) break; end;
 
   dR = SPdiffTr(x);
   dq = dR * p;

   % derivative of heights wrt. translation and rotation
   r = [ q(1:2,:) .* [ 1./s ; 1./s ] ; ones(2,6) ];

   G = zeros(6,6);
   for i = 1:6,
       G(i,:) =  r(:,i)' *  reshape(dq(:,i), 4,6);
   end;

%   Old version, appropriate for C programming
%   for i = 1:6,	   for j = 1:6,
%       G(i,j) =    1/s(i)*q(1,i)*dq((j-1)*4+1,i) ...
%		 + 1/s(i)*q(2,i)*dq((j-1)*4+2,i) ...
%	         +               dq((j-1)*4+3,i) ;
%
%   end;		   end;

   if rcond(G) < 1e-5,
       disp('Matrix is close to singular or badly scaled');
       keyboard;
   end

   % Newton-Raphson iteration
   dx = G \ dh';

   x = x + dx;
   R = SPtrpy2tr(x);
   
   
end % while

return
