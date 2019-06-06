% batch16 - investigates error in hand-eye calibration
%
% See also: batch15

batch15;	% load all values and perform hand-eye calibration
% Output: gHc - calculated camera-to-gripper transform
%         bHg - measured gripper-to-base transform
%         wHc - camera-to-world transform obtained from camera calibration 

M = size(bHg,3);            % number of calibration positions

% Verify that the calibration is successfull

% Take the average of all world-to-base transforms
bHw = zeros(4,4,M);
bHwsum = zeros(4,4);
for i = 1:M,    
        bHw(:,:,i) = bHg(:,:,i) * gHc * cHw(:,:,i);
        bHwsum = bHwsum + bHw(:,:,i); 
end;
bHwmean = bHwsum / M;

transError = zeros(M,1);
axisError  = zeros(M,1);
angleError = zeros(M,1);
for i = 1:M,    
        transError(i) = norm( transl(bHw(:,:,i)) - transl(bHwmean)); 
%       axisError (i) = 
end;


% Eliminate calibration positions number 4,6,7
% .. because they have the highest error
% .. relative to the mean

eliminate = [4,6,7];

bHg1 = bHg; bHg1(:,:,eliminate) = [];
wHc1 = wHc; wHc1(:,:,eliminate) = [];
gHc1 = handEye(bHg1, wHc1);

[dT, dAxisAngle, dRotAngle] = poseDiff(gHc, gHc1);

disp(sprintf('Translational  Error = %8.4f mm', norm(dT)));
disp(sprintf('Misalignment   Error = %8.4f degrees', dAxisAngle/pi*180) );
disp(sprintf('Rotation angle Error = %8.4f degrees', dRotAngle /pi*180) );


