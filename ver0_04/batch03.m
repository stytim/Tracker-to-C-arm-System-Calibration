% Batch file for forward kinematics of Stewart Paltform (SP) with Fixed Actuators (FA)


% Definition of fixed actuators model
H = 26;
Rb = 7.6; Rp = 3; Thetab = 30/180*pi; Thetap = 0/180*pi;
model = SPModel( Rb, Rp, Thetab, Thetap, H );

x0 = [ 0;0;H;  0; 0; 0];
R0 = SPtrpy2tr (x0);
x1 = x0 + 0.3*randn(size(x0));
R1 = SPtrpy2tr (x1);

heights = SPInvFA( model, R0 );

%default tolerance
R = SPforFA( model, heights, R1); 
disp([ x0' ; SPtr2trpy(R)']  );

%high tolerance
R = SPforFA( model, heights, R1, 0.001); 
disp([ x0' ; SPtr2trpy(R)']  );



