% batch15 - performs hand-eye calibration for CABG manipulator and camera
%           .. using actual measurements of actuator heights and 
%           .. and actual camera calibration matrices
%
% See also: handEye, batch09


batch09;        % setup manipulator model 

% These actuator heights are measured manually by Hong Ray Cho
% Each row contains heights for one platform pose
% Manipulator dimensions are set in M-file 'batch09'
heights = [
      104    101    116   143   123   128 
      129    142    127   137   145   142.5
      118.5  110    133   145   132   146
      91     96.5   110   115   120   135
      122    119    133   123   146   141
      113    114.5  103   106   146   122 
      138    119.5  142   129   121   130 
      133.5  144    125   141   146   138.5
];

M = size(heights,1);            % number of calibration positions


bHg = zeros(4,4,M);
for i = 1:M,    
        h = heights(i,:);        % pick one row from heights
        bHg(:,:,i) = SPForFA(model, h);% calculate forward kinematics for that row
end;


% Position of the world relative to camera for each pose 
% The matrices that follow are obtained from images of a calibration block
% .. for each camera pose. 
cHw = zeros(4,4,M);

cHw(:,:,1) = [
 0.903979 -0.388514  0.178547       8.500774
-0.348914 -0.911644 -0.217173      25.940195
 0.247146  0.134023 -0.959665      151.226507
 0         0         0               1
];


cHw(:,:,2) = [
 0.991606 -0.127511 -0.021411      -3.821990
-0.129260 -0.973761 -0.187300      22.186054
 0.003034  0.188496 -0.982069      107.032666
 0         0         0               1
];


cHw(:,:,3) = [
 0.972268 -0.029103  0.232052       8.220355
-0.017725 -0.998543 -0.050968      21.858717
 0.233197  0.045442 -0.971367      119.783727
 0         0         0               1
];

cHw(:,:,4) = [
 0.993138  0.115747 -0.016730       7.951648
 0.116729 -0.989852  0.081044      21.838966
-0.007179 -0.082441 -0.996570      148.687517
 0         0         0               1
];

cHw(:,:,5) = [
 0.682954  0.724527  0.092919      -29.007865
 0.721425 -0.688973  0.069733       1.306336
 0.114542  0.019410 -0.993229      127.448856
 0         0         0               1
];

cHw(:,:,6) = [
 0.685306  0.728217  0.007459      -16.627221
 0.722766 -0.678850 -0.129510      -1.367368
-0.089248  0.094145 -0.991550      152.687429
 0         0         0               1
];

cHw(:,:,7) = [
 0.591395  0.776019  0.219195       5.079114
 0.756859 -0.627967  0.181169      -9.142064
 0.278237  0.058757 -0.958713      134.523654
 0         0         0               1
];

cHw(:,:,8) = [
 0.996567 -0.074977  0.035098      -18.846679
-0.063712 -0.965328 -0.253146      19.597162
 0.052861  0.250041 -0.966791      124.668887
 0         0         0               1
];


% Find camera-to-world transform (inverse from world-to-camera)
wHc = zeros(4,4,M);
for i = 1:M,    
        wHc(:,:,i) = inv(cHw(:,:,i)); 
end;



% Finally, perform hand-eye calibration

gHc = handEye(bHg, wHc);


