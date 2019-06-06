% SPperspective - returns coordinates of points as seen by camera
%
%      y = SPperspective( x, f, Rot )   or   y = SPperspective( x, f ) 
%
%      x   - 3xN or 4xN matrix of points coordinates (xyz columnwise)
%      f   - focal length
%      Rot - 4x4 homogeneous rotation/translation matrix of camera position
%	   x-y plane is image plane, camera is directed in positive z direction
%	   If not specified, camera is placed in coordinate origin
%      y   - 2xN matrix of image coordinates
%
%      
function y = SPperspective( x, f, Rot )

if nargin == 3,
   % Applying inverse camera pose transformation to points
   if size(x,1) == 3,
      x = SPtrInv(Rot) * [ x ; ones(1, size(x,2)) ];
   elseif size(x,1) == 4,
      x = SPtrInv(Rot) * x;
   else
       disp('SPperspective: matrix x should be 3xN or 4xN');
   end
end;

y = - f ./ [ x(3,:) ; x(3,:) ] .* x(1:2,:);

if any( x(3,:) < 0 ),
   disp('SPperspective: some points are behind the camera');
end;

end;
