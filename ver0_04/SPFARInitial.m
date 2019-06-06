function Rinitial = SPFARInitial(model, heights)
% SPFARInitial
%              - returns initial default pose for a given SPFA model
%                (Stewart Platform with Fixed Actuators)
%
%	   R = SPFARInitial( model )
%	   R = SPFARInitial( model, heights ) 
%
%     model - 3x18 matrix containing model of the SPFA
%     heights- (optional) 1x6 vector of initial heights for forward kinemmatics
%     R - 4x4 matrix of the platform pose in homogeneous coordinates
%
%     See also: SPModel, SPForFA

if( nargin < 2 ), 
   heights = 0;
end;


[b,p,l] = SPmodel2bp(model);                    % calculate coordinates of base and platform

H = sqrt(l(1)^2 - sum( (b(:,1) - p(:,1)).^2 ));	% calculate default height
Rinitial = ...
    [eye(3) [0; 0; (H + mean(heights))]			% use for initial pose a pose with mean  
    ; 0 0 0           1               ];			% ... mean heights and no rotation
 
return;
 