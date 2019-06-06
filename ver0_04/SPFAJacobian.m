% SPFAJacobian - returns the Jacobian for the position of an SPFA
%
%	   J = SPFAJacobian ( model, trpy ) or J = SPFAJacobian ( model, Rot )
%  
%      model - parameters of the SPFA
%      trpy  - 6x1 vector describing the platform position
%	       (x,y,z,roll,pitch,yaw)
%      Rot   - 4x4 homogeneous transformation matrix describing 
%	       the platform position
%
%      The Jacobian is derivative of six pose parameters (as in trpy)
%	   wrt. six actuator heights
%
%  See also: SPModel, SPtr2trpy, SPtrpy2tr, SPforFA
%

function J = SPFAJacobian ( model, R )   

   if prod(size(R)) == 6,  % if the second argument is 'trpy'
       trpy = R;
       R = SPtrpy2tr(R);
   else,		   % if the second argument is 'Rot'
       trpy = SPtr2trpy(R);
   end;
   
   [b,p,lengths] = SPmodel2bp(model);


   % Inverse kinematics
   q = R*p - b;
   s = sqrt( lengths.^2 - q(1,:).^2 - q(2,:).^2);
   h = q(3,:) - s;

   dR = SPdiffTr(trpy);
   dq = dR * p;

   % derivative of heights wrt. translation and rotation
   r = [ q(1:2,:) .* [ 1./s ; 1./s ] ; ones(2,6) ];

   G = zeros(6,6);
   for i = 1:6,
       G(i,:) =  r(:,i)' *  reshape(dq(:,i), 4,6);
   end;

   J = inv(G);

end;
