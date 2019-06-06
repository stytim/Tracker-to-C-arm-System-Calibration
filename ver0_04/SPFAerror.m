% SPFAerror - calculates positional/rotational sensitivity of an SP
%
%	    error = SPFAerror( model, trpy )
%
%      model- parameters of the SPFA design
%      trpy - 6xN matrix, each column describing the position of the SP
%	       trpy(1:6,i) = [x;y;z; roll;pitch;yaw]
%      error- 6xN matrix with the error averaged for all six legs, 
%	       and normalized to unit leg length position.
%	       The each element of one column of the error is obtained as:
%	   e(i) = sqrt(sum( (1/dh(i) * J*[0 0 .. dh(i).. 0]).^2 ))
%	       where sum goes over i=1:6, dh(i) is the infinitesimal deviation
%	       of the i-th leg and J is the Jacobian for the given position

function  e = SPFAerror( model, trpy )

e = zeros(size(trpy));

for t = 1:size(trpy,2),
   % Error as a consequence of the i-th actuator deviation is obtained when 
   % the Jacobian is multiplied by the 6x1 vector with all elements zero
   % except for the i-th element which is one. i.e. [0 0 0 1 0 0]'
   % Six such vectors form an identity matrix of size 6 (eye(6))

   separateErrors = SPFAJacobian(model, trpy(:,t) ) * eye(6);
   e(:,t) = sqrt(0.5 * sum( separateErrors' .^2)');

end;

return;
