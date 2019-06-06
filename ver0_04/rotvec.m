%ROTVEC	Rotation about arbitrary axis
%
%	ROTVEC(V, THETA) returns a homogeneous transformation representing a 
%	rotation of THETA about the vector V.
%  
%  V - 3x1 unit vector, such that norm(v) == 1
%  theta - angle in radians
%
%	See also ROTX, ROTY, ROTZ.

% 	Copyright (C) Peter Corke 1990
function r = rotvec(v, t)
	ct = cos(t);
	st = sin(t);
	vt = 1-ct;
	v = v(:);
	r =    [ct	      -v(3)*st	   v(2)*st
           v(3)*st      ct       -v(1)*st
          -v(2)*st	 v(1)*st       ct	];
	r = [v*v'*vt+r zeros(3,1); 0 0 0 1];
