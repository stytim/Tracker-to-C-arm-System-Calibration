% SPForward - returns direct kinematics for Stewart platform
%
%	   R = SPForward( model, lengths, Rinitial, tolerance )
%
%      model - 3x12 vector containing coordinates of the Stewart platform joints
%      lengths - vector with 6 leg lengths
%      Rinitial - initial estimate of position, 4x4 homogeneous coordinates
%      tolerance - accuracy of calculations, default: mean(lengths)/1000
%      R - 4x4 homogeneous rotation/translation matrix
%
%      see also: SPModel, SPInverse, SPInvFA, SPforFA
function R = SPForward( model, lengths, R , tol)

if nargin < 4,
   tol = mean(lengths)/1000;
end;

if size(model,2) == 18,  % SP with fixed actuators
   R = SPForFA( model, lengths, R , tol);
   return;
end;

% coordinates of base and platform joints
[b,p] = SPmodel2bp(model);

% calculate 
x  = SPtr2trpy(R);
G = zeros(6,6);

while ( 1 ),

   % calculate lengths from initial guess 
   l = R*p - b;
   absl = sqrt(sum(l.^2));
   dl = lengths - absl;
   
   if (max(abs(dl)) < tol), break; end;

   % calculate derivative of rotation matrix
   dR = SPdiffTr(x); 
   dq = dR*p;

   % calculate derivative of inverse kinematics
   l = l ./ [ absl; absl; absl; absl];
   for i = 1:6,
       G(i,:) = l(:,i)' * reshape( dq(:,i), 4,6); 
   end;

   % use Newton-Raphson method
   dx = G \ dl'; % dx = inv(G) * dl'
   x = x + dx;
   R = SPtrpy2tr(x);

end

% Inverse kinematics for SP is:
%      l = R*p - b;
%      absl = sqrt( lx^2 + ly^2 + lz^2 )
% For Newton-Raphson method we need derivative dx/dabsl
% We calculate dabsl/dx and then invert it:
%      G = d absl/dx = 1/absl * (lx*dlx/dx+ly*dly/dx+lz*dlz/dx)
%      G = 1/absl * ( l' * (dl/dx) )
%      dl/dx = dR/dx*p
% =>
%      G = 1/absl * ( l' * (dR/dx*p));
%      l = l0 + G*dx =>  dx = inv(G)*(l-l0) = inv(G)*dl
% Forward kinematics is obtained iteratively:
%      x = x + dx

return

