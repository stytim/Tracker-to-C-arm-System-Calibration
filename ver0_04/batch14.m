% batch14 - Plots a Stewart Platform in a certain position and a set of points
% 3D rotation of the drawing is enabled
%    batch14(model, Hp, points)
%   
%    model - SP model (see SPModel)
%    Hp    - Platform position, 4x4 homogeneous transformation matrix
%    points- 3xN matrix, with N points [x;y;z]
%    
% See also: SPPlot


function batch14(model, Hp, points)
	SPPlot( model, Hp);
	axis normal; axis vis3d; axis equal;
	rotate3d on;
	hold on;
	plot3(points(1,:), points(2,:), points(3,:), 'o');
	hold off


return