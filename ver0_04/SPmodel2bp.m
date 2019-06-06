% SPmodel2bp - returns coordinates of base and platform from model matrix
%
%      [b,p] = SPmodel2bp(model)
%      or
%      [b,p,lengths] = SPmodel2bp(model)
%
%      model - model of Stewart Platform, i.e:  
%	       model = SPModel( Rb,Rp, ab, ap)
%      b - 4x6 matrix of coordinates of base joints (x y z 1)
%      p - 4x6 matrix of coordinates of platform joints
%      lengths - 1x6 matrix of leg lengths for SP with fixed actuators
%
%      See also: SPModel, batchplot

function [b,p,lengths] = SPmodel2bp(model)

b = [ model(:,1:6 ); ones(1,6) ];
p = [ model(:,7:12); ones(1,6) ];

if (nargout == 3) & (size(model,2) >= 3*6),
   % Stewart Platform with Fixed Actuators
   lengths = model(1,13:18);
end;

return
