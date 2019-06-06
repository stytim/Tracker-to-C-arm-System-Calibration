function h = SPInvFA( model, R, T )
% SPInvFA - returns the lengths of a Stewart Platform fith Fixed Actuators
%      for a given position and orientation of the platform
%  
%      Heights = SPInvFA( model, Rotation, Translation )
%	   or
%      Heights = SPInvFA( model, Homogeneous )
%
%      model - 3x18 vector containing coordinates of the Stewart platform
%	   joints. The first six columns represent coordinates of the base 
%	   joints and the next six columns correspond to the platform, 
%	   z-values (third row) are added to the actuators lenghts.  
%	   The last six columns in the first row have lengths of legs.
%      Rotation    - 3x3 rotation matrix of the platform orientation
%      Translation - 3x1 translation vector of the platform position
%      Homogeneous = 3x4 homogeneous matrix, 
%	   Homogeneous = [ Rotation  Translation ; 0 0 0 1];
%      Lengths     - 1x6 vector of actuator lengths
%   
%      REFERENCES: 
%      CC Nguyen, SS Antrazi, Z-L Zhou: Adaptive Control of a Stewart
%	   Platform-Based Manipulator, 
%	   Journal of Robotic Systems 10(5) 657-687 (1993)
%      

if nargin == 3,
   R = [R T ; 0 0 0 1];
end;

[b,p,l] = SPmodel2bp(model);
q = R*p - b;

h = q(3,:) - sqrt(l.^2 - q(1,:).^2 - q(2,:).^2);

return
