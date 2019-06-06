% handEye - performs hand/eye calibration
% 
%     gHc = handEye(bHg, wHc)
% 
%     bHg - pose of gripper relative to the robot base..
%           (Gripper center is at: g0 = Hbg * [0;0;0;1] )
%           Matrix dimensions are 4x4xM, where M is ..
%           .. number of camera positions. 
%           Algorithm gives a non-singular solution when ..
%           .. at least 3 positions are given
%           Hbg(:,:,i) is i-th homogeneous transformation matrix
%     wHc - pose of camera relative to the world ..      
%           (relative to the calibration block)
%           Dimension: size(Hwc) = size(Hbg)
%     gHc - 4x4 homogeneous transformation from gripper to camera      
%           , that is the camera position relative to the gripper.
%           Focal point of the camera is positioned, ..
%           .. relative to the gripper, at
%                 f = gHc*[0;0;0;1];
%           
% References: R.Tsai, R.K.Lenz "A new Technique for Fully Autonomous 
%           and Efficient 3D Robotics Hand/Eye calibration", IEEE 
%           trans. on robotics and Automaion, Vol.5, No.3, June 1989
%
% Notation: wHc - pose of camera frame (c) in the world (w) coordinate system
%                 .. If a point coordinates in camera frame (cP) are known
%                 ..     wP = wHc * cP
%                 .. we get the point coordinates (wP) in world coord.sys.
%                 .. Also refered to as transformation from camera to world
%
% Code from  http://lazax.com/www.cs.columbia.edu/~laza/html/Stewart/matlab/handEye.m

function [gHc, err_, ResR, MedianR, StdDevR, RMSR, ResT, MedianT,StdDevT,RMST] = handEye(bHg, wHc)

M = size(bHg,3);

K = (M*M-M)/2;               % Number of unique camera position pairs
A = zeros(3*K,3);            % will store: skew(Pgij+Pcij)
B = zeros(3*K,1);            % will store: Pcij - Pgij
k = 0;


% Now convert from wHc notation to Hc notation used in Tsai paper.
Hg = bHg;
% Hc = cHw = inv(wHc); We do it in a loop because wHc is given, not cHw
Hc = zeros(4,4,M); for i = 1:M, Hc(:,:,i) = inv(wHc(:,:,i)); end;

for i = 1:M,
   for j = i+1:M;
		Hgij = inv(Hg(:,:,j))*Hg(:,:,i);    % Transformation from i-th to j-th gripper pose
		Pgij = 2*rot2quat(Hgij);            % ... and the corresponding quaternion
      
		Hcij = Hc(:,:,j)*inv(Hc(:,:,i));    % Transformation from i-th to j-th camera pose
		Pcij = 2*rot2quat(Hcij);            % ... and the corresponding quaternion

      k = k+1;                            % Form linear system of equations
      A((3*k-3)+(1:3), 1:3) = skew(Pgij+Pcij); % left-hand side
      B((3*k-3)+(1:3))      = Pcij - Pgij;     % right-hand side
      
   end;
end;

% Rotation from camera to gripper is obtained from the set of equations:
%    skew(Pgij+Pcij) * Pcg_ = Pcij - Pgij
% Gripper with camera is first moved to M different poses, then the gripper
% .. and camera poses are obtained for all poses. The above equation uses
% .. invariances present between each pair of i-th and j-th pose.

Pcg_ = A \ B;                % Solve the equation A*Pcg_ = B

size(A);
size(B);
err = A*Pcg_ - B;
err;
residus_TSAI_rotation = sqrt(sum((err'*err))/K);
residus_2 = sum(sqrt(err'*err))/K;
% instead mean
mean_error = sum(abs(err)) / K;
std_dev = sqrt(sum((err - mean_error).^2) / (K-1));
std_dev_deg = rad2deg(std_dev);

% compute error based on x y z seperatley:
sz = ceil(K/3);
err_x = zeros(sz,1);
err_y = zeros(sz,1);
err_z = zeros(sz,1);
euler_rot = zeros(sz, 3);
quat_rot = zeros(sz, 3);
j = 1;
mean_frob_norm = 0;
for i =1:3:K
   err_x(j) = err(i); 
   err_y(j) = err(i+1);
   err_z(j) = err(i+2);
   local_quat = [err_x(j), err_y(j), err_z(j)]';
   norm_local_quat = local_quat / sqrt(1 + local_quat'*local_quat);
   quat_rot(j,1:3) = norm_local_quat;
   R_error = quat2rot(norm_local_quat);
   euler_rot(j,1:3) = rotm2eul(R_error(1:3,1:3));
   frobNorm = norm((R_error(1:3,1:3)-eye(3)),'fro');
   mean_frob_norm = mean_frob_norm + frobNorm;
   
   
   j = j+1;
end
mean_frob_norm = mean_frob_norm / j;
%final for paper:
for i = 1:3 
mean_euler_rotation(i) = sqrt(sum(euler_rot(:,i)' * euler_rot(:,i)) / sum(j));
mean_quat_rotation(i) = sqrt(sum(quat_rot(:,i)' * quat_rot(:,i)) / sum(j));
end 

c = quat2rot(mean_quat_rotation);
rad2deg(rotm2eul(c(1:3,1:3)));

x = sqrt(sum((err_x'*err_x))/j);
y = sqrt(sum((err_y'*err_y))/j);
z =sqrt(sum((err_z'*err_z))/j);
residualR = [x y z]';

med_X  = rad2deg(median(abs(err_x)));
med_Y  = rad2deg(median(abs(err_y)));
med_Z  = rad2deg(median(abs(err_z)));

MedianR = [med_X , med_Y, med_Z];

norm_residual = residualR / sqrt(1 + residualR'*residualR);

resRm = quat2rot(norm_residual);
eulResR = rad2deg(rotm2eul(resRm(1:3,1:3)));
mean_euler_rotation;
eulPaperDe = rad2deg(mean_euler_rotation);
ResR = eulPaperDe;

RMS_angle = sqrt(sum(eulPaperDe .^ 2)/3);

RMSR = RMS_angle;

std_devRx = rad2deg(sqrt(sum((abs(err) - mean_euler_rotation(1)).^2) / (3*K-1)));
std_devRy = rad2deg(sqrt(sum((abs(err) - mean_euler_rotation(2)).^2) / (3*K-1)));
std_devRz = rad2deg(sqrt(sum((abs(err) - mean_euler_rotation(3)).^2) / (3*K-1)));

StdDevR =[std_devRx, std_devRy, std_devRz];

% Obtained non-unit quaternin is scaled back to unit value that
% .. designates camera-gripper rotation
Pcg = 2 * Pcg_ / sqrt(1 + Pcg_'*Pcg_);

Rcg = quat2rot(Pcg/2);         % Rotation matrix


% Calculate translational component
k = 0;
for i = 1:M,
   for j = i+1:M;
		Hgij = inv(Hg(:,:,j))*Hg(:,:,i);    % Transformation from i-th to j-th gripper pose
		Hcij = Hc(:,:,j)*inv(Hc(:,:,i));    % Transformation from i-th to j-th camera pose

      k = k+1;                            % Form linear system of equations
      A((3*k-3)+(1:3), 1:3) = Hgij(1:3,1:3)-eye(3); % left-hand side
      B((3*k-3)+(1:3))      = Rcg(1:3,1:3)*Hcij(1:3,4) - Hgij(1:3,4);     % right-hand side
      
   end;
end;






Tcg = A \ B;

err_tr = A*Tcg-B;
size(err_tr);
residus_TSAI_translation = sqrt(sum((err_tr'*err_tr))/K);
%translation error:
mean_error = sum(abs(err_tr)) / (3*K);
std_dev = sqrt(sum((abs(err_tr) - mean_error).^2) / (3*K-1));


% Seperatly for xyz
% compute error based on x y z seperatley:
sz = ceil(K/3);
err_xT = zeros(sz,1);
err_yT = zeros(sz,1);
err_zT = zeros(sz,1);

j = 1;
for i =1:3:K
   err_xT(j) = err_tr(i); 
   err_yT(j) = err_tr(i+1);
   err_zT(j) = err_tr(i+2);
   j = j+1;
end
xT = sqrt(sum((err_xT .^ 2))/j);
yT = sqrt(sum((err_yT .^ 2))/j);
zT =sqrt(sum((err_zT .^ 2))/j);

residualT = [xT yT zT]';
ResT = residualT';
%final for paper:
rms = sqrt(sum(residualT .^ 2)/3);
RMST = rms;

medX  = median(abs(err_xT));
medY  = median(abs(err_yT));
medZ  = median(abs(err_zT));

MedianT = [medX, medY, medZ];

std_devTx = sqrt(sum((abs(err_tr) - xT).^2) / (3*K-1));
std_devTy = sqrt(sum((abs(err_tr) - yT).^2) / (3*K-1));
std_devTz = sqrt(sum((abs(err_tr) - zT).^2) / (3*K-1));

StdDevT = [std_devTx,std_devTy,std_devTz];
gHc = transl(Tcg) * Rcg;	% incorporate translation with rotation

err_ = [residus_TSAI_rotation,residus_TSAI_translation ];

%TODO: return all errors, not only residual...
return