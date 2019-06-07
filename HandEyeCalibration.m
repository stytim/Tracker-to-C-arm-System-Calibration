%% Tracker to C-arm System Calibration:
% This is an implementation of TSAI:
% modifed from Stewart: http://lazax.com/www.cs.columbia.edu/~laza/html/Stewart/matlab/handEye.m
% Author: Tianyu Song
% tsong11@jhu.edu
% JOHNS HOPKINS UNIVERSITY, COMPUTER AIDED MEDICAL PROCEDURES
%% Initialize
clc
close all;
clear;
% Start from 3 pairs until 120 pairs
startn = 3;
step = 1;
endn = 120;
iter = 800;
AllErrT = [];
AllErrR = [];
AllErrRR = [];
AllstdT = [];
AllstdR =[];
AllstdRR =[];
%% Load Poses of Xray and Hololens:
load(strcat('./Tracker_Xray_Extrinsics/Extrinsics_',int2str(endn),'.mat'));
flip = [1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, -1, 0; %-1
        0, 0, 0, 1]; 
addpath(genpath('./ver0_04'));
%% Calibrate with Tsai's Hand-eye Calibration Method
for jj = startn:step:endn
     sumErrT = 0;
     sumErrR = 0;
     sumErrRR = 0;   
     for kk = 1:iter    
        idx=randperm(length(H2W),jj);
        idxx=randperm(length(H2W),120);
         fh = idx;
         sh = idxx;
        [c2h_stewart2, err_stewart2, ResR2, MedianR2, StdDevR2, RMSR2, ResT2, MedianT2,StdDevT2,RMST2] = handEye_stewart(H2W(:,:,fh), C2IR(:,:,fh));
        for i=1:size(fh,2)
            HRC(:,:,i) =  H2W(:,:,fh(i)) * c2h_stewart2 / C2IR(:,:,fh(i))  ;
        end
        %Average transformation              
        HRCAvg = averageTransformation(HRC);
        for ii = 1:size(sh,2)
            C2W1(:,:,ii) = HRCAvg * C2IR(:,:,sh(ii));
            C2W2(:,:,ii) = H2W(:,:,sh(ii)) * c2h_stewart2;      
        end
        % Test how well the dataset solves AX = XB:
        [angle, eulerdeg] = RotationDist(C2W1(1:3,1:3,:),C2W2(1:3,1:3,:));
        allangle(:,:,kk) = angle;
        alleuler(:,:,kk) = vecnorm(eulerdeg);
        sumErrR = sumErrR + angle;
        sumErrRR = sumErrRR + eulerdeg;
        t_err = reshape(mean(abs(C2W1(1:3,4,:) - C2W2(1:3,4,:)),3),1,3);
        alltrans(:,:,kk) = vecnorm(t_err);
        sumErrT = sumErrT + t_err;           
     end
    stdT = std(alltrans);
    stdR = std(allangle);
    stdRR = std(alleuler);
    ErrT = sumErrT / iter ;
    ErrR = sumErrR / iter ;
    ErrRR = sumErrRR /iter ;
    handEye(jj).ErrT = ErrT;
    handEye(jj).stdT = stdT;
    handEye(jj).stdR = stdR;
    handEye(jj).stdRR = stdRR;
    handEye(jj).ErrR = ErrR;
    handEye(jj).ErrRR = ErrRR;
    AllErrT = [AllErrT; handEye(jj).ErrT];
    AllErrR = [AllErrR; handEye(jj).ErrR];
    AllErrRR = [AllErrRR; handEye(jj).ErrRR];
    AllstdT = [AllstdT; handEye(jj).stdT];
    AllstdR = [AllstdR; handEye(jj).stdR];
    AllstdRR = [AllstdRR; handEye(jj).stdRR]; 

    c2h_stewart = c2h_stewart2;

    % Ready for Unity
    %
    % T_C2H - 4x4 homogeneous transformation from HoloLens to X-ray source ,
    % or X-ray source pose in HoloLens coordinate frame.
    %
    % 1. Change from right-hand to left-hand 
    c2h_stewart_flip = flip * c2h_stewart * flip;
    %
    % 2. Change from millimeter to meter
    T_C2H = c2h_stewart_flip;
    T_C2H(1:3,4) = T_C2H(1:3,4) * 0.001;
    
    handEye(jj).T = T_C2H;
    handEye;
end
save(strcat('./ErrorMat/Err_',int2str(startn),'_',int2str(step),'_',int2str(endn),'_',int2str(iter),'.mat'))

%% Plot Error

rad = rad2deg(AllErrR);
stdr = rad2deg(AllstdR);
t = vecnorm(AllErrT')';
stdt = AllstdT(2:118);
t = t(2:118);
stdr = stdr(2:118);
rad = rad(2:118);
jx = 4:1:120;
bx = ax;

data = [jx;rad'];     
stdr = stdr(2:end);
plot_areaerrorbar(data(:,2:end),jx(2:end),stdr',1,'Hand-Eye Error in Rotation','Number of Datasets','Error (deg)',[0 120 0.25 0.45],0);

data = [jx;t'];     
stdt = stdt(2:end);
plot_areaerrorbar(data(:,2:end),jx(2:end),stdt',2,'Hand-Eye Error in Translation','Number of Datasets','Error (mm)',[0 120 4.5 10],1);
