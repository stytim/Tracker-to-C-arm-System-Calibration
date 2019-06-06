% Batch file for forward kinematics

% Definition of the model
H = 26;
Rb = 7.6; Rp = 3; Thetab = 30/180*pi; Thetap = 0/180*pi;
[base,platform] = SPModel( Rb, Rp, Thetab, Thetap );
model = [base, platform];


R = 10;	       %trajectory radius
Ax = R*cos(pi/4); Az = -R*sin(pi/4);

dFi = 1/180*pi; % 1 degree

% one position
Rout = SPtrpy2tr([Ax;0; H+R+Az;    0; -pi/4; 0 ]);
% slightly rotated position
Rot2 = SPtrpy2tr([Ax+1;0+1; H+R+Az+1;    0+10*dFi; -pi/4-10*dFi; 0-10*dFi ]);


lengths = SPInverse(model, Rot2);

e = sum(abs( SPtr2trpy(Rout) - SPtr2trpy(Rot2)   ));
disp(e);
disp([ SPtr2trpy(Rot2)';  SPtr2trpy(Rout)'])
disp('---------------')

tol = mean(lengths)/10000;

   Rout = SPforward( model, lengths, Rout, tol);
   e = sum(abs( SPtr2trpy(Rout) - SPtr2trpy(Rot2)   ));
   disp(e);
   disp([ SPtr2trpy(Rot2)';  SPtr2trpy(Rout)'])
   disp('---------------')

