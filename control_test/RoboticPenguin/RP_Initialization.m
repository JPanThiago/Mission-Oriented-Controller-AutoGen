%---------------------------------------------%
%--- Initialize the penguin-inspired robot ---%
%---------------------------------------------%
global glideParas;
glideParas = struct;

global waterDensity graAcc; 
% unit: kg/m^3
waterDensity = 998.2;
graAcc = 9.8; % acceleartion of gravity

% unit vector of a frame
global iix jjy kkz IIX JJY KKZ;
iix = [1; 0; 0];
jjy = [0; 1; 0];
kkz = [0; 0; 1];
IIX = [0; 0; 0; 1; 0; 0];
JJY = [0; 0; 0; 0; 1; 0];
KKZ = [0; 0; 0; 0; 0; 1];

global left right U_old W_old;
left = 1;
right = 2;
U_old = 0;
W_old = 0;

%% body coefficient
% units m^2
glideParas.Sxx = 0.019839 + 2 * 0.002 + 2 * 0.000088;
glideParas.Syy = 0.077069 + 0.000772;
glideParas.Szz = 0.078168 + 2 * 0.007005 + 2 * 0.00539; 
glideParas.Ab = [glideParas.Sxx,              0,               0;
                              0, glideParas.Syy,               0;
                              0,              0, glideParas.Szz];            

glideParas.mall = 7.4;
glideParas.J = [ 0.018713,        0,        0;
                        0, 0.234470,        0;
                        0,        0, 0.238203];
                    
%% wing coefficient   
glideParas.leftWingPt = [0.066; -0.198; 0];
glideParas.rightWingPt = [0.066; 0.198; 0];
global Vp Ap kappa1 kappa2 varkappa1 varkappa2 beta;
Vp = 0.000065785; Ap = 0.007005;
kappa1 = 0.019154; kappa2 = 0.000010416784; varkappa1 = 0.102965; varkappa2 = 0.000116820987;
beta = 30 / 180 * pi;
