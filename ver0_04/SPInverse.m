function l = SPInverse( model, R, T )
% SPInverse
% Stewart Platform Inverse - returns the lengths of a Stewart Platform actuators
%      for a given position and orientation of the platform
%  
%      Lengths = SPInverse( model, Rotation, Translation )
%	   or
%      Lengths = SPInverse( model, Homogeneous )
%
%      model - 3x12 vector containing coordinates of the Stewart platform
%	   joints. The first six columns represent coordinates of the base 
%	   joints and the next six columns correspond to the platform.  
%      Rotation    - 3x3 rotation matrix of the platform orientation
%      Translation - 3x1 translation vector of the platform position
%      Homogeneous = 4x4 homogeneous matrix, 
%	   Homogeneous = [ Rotation  Translation ; 0 0 0 1];
%      Lengths     - 1x6 vector of actuator lengths
%
%      For SPFA (Stewart Platform with Fixed Actuators), the model is 3x18 matrix
%
%      See also: SPModel
%   
%      REFERENCES: 
%      CC Nguyen, SS Antrazi, Z-L Zhou: Adaptive Control of a Stewart
%      Platform-Based Manipulator, Journal of Robotic Systems 10(5) 657-687 (1993)


if nargin == 3,
   R = [R T ; 0 0 0 1];
end;

if (size(model,2) >= 3*6),								% if the model is SPFA manipulator
	[b,p,l] = SPmodel2bp(model);
	q = R*p - b;

	h = q(3,:) - sqrt(l.^2 - q(1,:).^2 - q(2,:).^2);
else   														% if the model is SP manipulator
	[b,p] = SPmodel2bp(model);
	l = sqrt ( sum((R(1:3,1:4)*p - b).^2) );
end; 

return
