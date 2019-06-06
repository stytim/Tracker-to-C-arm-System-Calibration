% poseDiff - returns difference between two pose transforms
%		A pose is determined by translational and rotational
%		.. components. Rotational component can be represented
%		.. as a rotation by an angle around an axis.
%		The result is difference of the three components
%		.. (translation, axis and angle)
%
%		[dT, dAxisAngle, dRotAngle] = poseDiff(H1, H2)
%
%	H1, H2 - 4x4 homogeneous matrices designating the poses
%	dT     - 3x1 difference in translational component
%  dAxisAngle - angle between the two axis of rotation
%	dRotAngle - difference between the two rotation angles
%

function [dT, dAxisAngle, dRotAngle] = poseDiff(H1, H2)

	dT = transl(H2) - transl(H1);
   
   v1 = rot2quat(H1); 						% quaternion corresponding to rotation
   rotAngle1 = 2 * asin(norm(v1)); 		% rotation angle around the axis
   v1 = v1/norm(v1);							% the rotation axis is normalized quaternion

   v2 = rot2quat(H2); 						% quaternion corresponding to rotation
   rotAngle2 = 2 * asin(norm(v2)); 		% rotation angle around the axis
   v2 = v2/norm(v1);							% the rotation axis is normalized quaternion

	dAxisAngle = asin(norm(cross(v1,v2)));
	dRotAngle  = rotAngle2 - rotAngle1;



return