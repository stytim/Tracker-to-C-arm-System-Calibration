function [base,plat] = SPModel( Rb, Rp, Thetab, Thetap, height )
% SPModel - returns model of a Stewart platform type manipulator 
%      joint coordinates, for the 6 joint base and platform
%      
%      model = SPModel( Rb, Rp, Thetab, Thetap )
%	       or
%      [base,platform] = SPModel( Rb, Rp, Thetab, Thetap )
%
%      Rb, Rp - radii of the base and platform 
%      Thetab, Thetap - angles between the first two joints on the base and
%	   platform
%      base, platform - 3x6 matrices with coordinates of 6 joints columnwise
%
%      For Stewart Platform with fixed actuators, usage is:
%      model = SPModel( Rb,Rp, Thetab,Thetap, height )
%
%      height - height of platform for zero actuator positions
%      
%      See also: batchplot, SPmodel2bp

%      REFERENCES: 
%      CC Nguyen, SS Antrazi, Z-L Zhou: Adaptive Control of a Stewart
%      Platform-Based Manipulator, Journal of Robotic Systems 10(5) 657-687 (1993)

%      Format of model
%      [ b1 b2 ... b6   p1 p2 ... p6  l1 l2 ... l6 ]
%      b1..b6 - 3x1 vectors of base joints coordinates
%      p1..p6 - 3x1 vectors of platform joints coordinates
%      l1..l6 - 3x1 vectors with leg lengths in the first row 
%	      , 2nd and 3rd row are zero. Lengths are present in FA model only.
%      


if nargin == 0,   Rb = 1; Thetab = pi/6; end;
if nargin <  2,   Rp = Rb; end;
if nargin <  4,   Thetap = 2*pi/3 - Thetab; end;



Lambdab = zeros(1,6); Lambdap = zeros(1,6);


baseAngles = 2*pi/3*[ -1 -1  0  0  +1 +1 ];
offset     =        [ -1 +1 -1 +1  -1 +1 ];
   
   
Lambdab  = baseAngles + Thetab/2*offset;
Lambdap  = baseAngles + Thetap/2*offset;

base = [ Rb * cos(Lambdab) ...
       ; Rb * sin(Lambdab) ...
       ; zeros(1,6) ];
 
plat = [ Rp * cos(Lambdap) ...
       ; Rp * sin(Lambdap) ...
       ; zeros(1,6) ];
 
if ( nargout == 1 ),
   if nargin == 5, % SP with Fixed Actuators
       l = sqrt(sum( (plat - base).^2 + (height*[zeros(2,6);ones(1,6)]).^2 ));
       base = [base plat [ l ; zeros(2,6)] ];
   else
       base = [base plat];
   end;
end;
return


