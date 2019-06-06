% batch09
% Defines dimensions used for SPFA model of the CABG manipulator
%
% All dimensions are in millimeters.
% Actuators are named UVXYZT
%
%          Z        Y       
%                                    *  *
%       T              X    
%             Base                 Platform
%                                *          *
%                                  *      *
%            U    V            
%


% The radii of the base and platform, and height are as calculated in the thesis:

Rb =  76; 												% Radius of the base	
Rp =  30; 												% radius of the platform	
H  = 260; 												% height of the platform in neutral position 

% Actuators and joints are positioned along the circles of base and platform, respectively.
% The characteristic angles between joint pairs are calculated from
%  measured distances between the pairs. The distances are as follows:
% <Base>
%         VU : 15.67
%         TZ : 15.32
%         YX : 15.65
% 
% <Platform>
% 
%         UT : 12.45
%         ZY : 12.35
%         XV : 12.65

db = mean([15.67 15.32 15.65]);
dp = mean([12.45 12.35 12.65]);

% Tha angles are calculated:
Thetab = 2 * asin( db/2 / Rb );
Thetap = 2*pi/3  -  2 * asin( dp/2 / Rp );

% SPFA model:
model = SPModel( Rb, Rp, Thetab, Thetap, H );

clf;
SPPlot( model, eye(3), [0;0;H]);
axis normal; axis vis3d; axis equal;
rotate3d on;


