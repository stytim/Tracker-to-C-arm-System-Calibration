function SPPlot( model, R, T )
% SPPlot - plots Stewart platform
%
%      SPPlot( model, Rotation, Translation)
%	   or
%      SPPlot( model, Homogeneous)
%
%      model - 3x12 vector containing coordinates of the Stewart platform
%	   joints. The first six columns represent coordinates of the base 
%	   joints and the next six columns correspond to the platform.  
%      Rotation    - 3x3 rotation matrix of the platform orientation
%      Translation - 3x1 translation vector of the platform position
%      Homogeneous = 4x4 homogeneous matrix, 
%	   Homogeneous = [ Rotation  Translation ; 0 0 0 1];


if nargin == 1,
   R = eye(4);
elseif nargin == 3,
   R = [R T ; 0 0 0 1];
elseif nargin ~= 2
   disp('SPPlot: Wrong number of parameters');
end;

b = model(:,1:6);
p = R* [ model(:,6+(1:6)) ; ones(1,6) ];

% bx = [b(1,:) b(1,1)]; by =  [b(2,:) b(2,1)]; bz =  [b(3,:) b(3,1)]; 
% px = [p(1,:) p(1,1)]; py =  [p(2,:) p(2,1)]; pz =  [p(3,:) p(3,1)];
bx = b(1,:); by = b(2,:); bz = b(3,:);
px = p(1,:); py = p(2,:); pz = p(3,:);


patch(px,py,pz,[0.3 0.7 0.8]);       % Plotting platform
hold on;
plot3(px(1:6),py(1:6),pz(1:6),'ko'); % Platform joints
if ( size(model,2) == 18 ),  % plotting SP with fixed actuators
   h  = SPInvFA( model, R);
   plot3(bx(1:6),by(1:6),bz(1:6)+h,'o');
   patch(bx,by,bz,[0.5 0.7 0.9]);
   for i = 1:6,
       l1=plot3( [bx(i);bx(i)], [by(i);by(i)], [bz(i);bz(i)+h(i)],'r' );
       l2=plot3( [bx(i);px(i)], [by(i);py(i)], [bz(i)+h(i);pz(i)],'k' );
       set(l1,'LineWidth',3);
       set(l2,'LineWidth',2);
   end;
else			   % plotting regular SP
   patch(bx,by,bz,'r');	   % Plotting the base
   plot3(bx,by,bz,'o'); 
   for i = 1:6,
       plot3( [bx(i);px(i)], [by(i);py(i)], [bz(i);pz(i)] );
   end;
end;

hold off;
axis equal;					% make lengths along all three axes the same
axis vis3d;					% freezes aspect ratio properties to enable rotation of 3-D objects
rotate3d on;				% enable 3D rotation using mouse

return
