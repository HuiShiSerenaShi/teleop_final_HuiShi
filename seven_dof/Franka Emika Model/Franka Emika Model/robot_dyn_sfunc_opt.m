%SFUNCTION 

function [sys,x0,str,ts] = robot_dyn_sfunc_opt(t,x,u,flag,q0,dq0)
switch flag
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes(q0,dq0);
  case 1
    sys=mdlDerivatives(t,x,u); %u sarebbe tau
  case 3
    sys=mdlOutputs(t,x,u);
  case { 2, 4, 9 }
    sys = [];
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(q0,dq0)

%abbiamo 7 giunti quindi avremo 7 posizioni di giunto e 7 velocità di
%giunto
sizes = simsizes;
sizes.NumContStates  = 14; %7*2 ne vogliamo il doppio perchè posizione e velocità
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 14;
sizes.NumInputs      = 7; %una coppia per ogni giunto, in ingresso abbiamo le coppie in uscita pos e velocità
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1; %impostazioni di simulazione

sys = simsizes(sizes);

% x0 = [q0,dq0]; %stato iniziale
x0 = [q0,dq0]; %stato iniziale

str = [];
ts  = [0,0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function dx=mdlDerivatives(~,x,u)
    q = x(1:7);
    dq = x(8:14);
    g = get_GravityVector(q);            
    c = get_CoriolisVector(q,dq);         
    M = get_MassMatrix(q);                
    tauf = get_FrictionTorque(dq);
    ddq = inv(M)*(u - c - g - tauf);  %invertiamo perchè ci serve l'accellerazione
    
    dx = [dq;ddq]; %campo vettoriale che descrive la dinamica del manipolatore
    % dq sono gli ultimi 7 dello stato x
    %infatti abbiamo lo stato x che è = [q,dq] e derivando x otteniamo dx
    %che è = [dq, ddq] dq ce lo abbiamo già mentre ddq ce lo ricaviamo
    %invertendo l'equazione del moto che abbiamo. 
% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(~,x,~)

sys = x;

% end mdlOutputs