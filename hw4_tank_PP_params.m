% Sample parameters for four channel bilateral teleoperation
clear all;
close all;
clc

% Input function parameter (sin or step with low pass filter)
A = 1;

% Low pass frequency cuff off
Flp1 = 0.5;
% Sin frequency

Fc=1; % free no noise t50
% Fc = 0.5; % contact no noise   t50

% Human intention controller (PD)
Ph = 10*1; 
Dh = 20*0.8; 

% Human impedance parameters
Jh = 1;
Bh = 1; 
Kh = 0;   % Stiffness
% Inertia/Damping of robot dynamics
Mm = 0.5;
Ms = 2;
Dm = 0;
Ds = 0;

% Master controller
Bm = 40*0.8; % 40*0.8
Km = 10*1;

% Slave controller
Bs = 4*Bm; 
Ks = 4*Km; 

% Environment impedance parameters
Be = 100; 
Ke = 200; 
xe = 1.5;
xe = 0.5;
xe = 1.5;
s = tf('s');
Ts = 0.001;

% Transport delay
d = 10;

% Noise
noise = 0.001; %
% noise = 0.000001; %
% Kalman
x0 = [0 0];
Ak = [1 Ts ; 
     0 1 ]; 
B = [Ts^2/2; Ts];
C = [1 0];
q = 10000000; %
q = 1e10;
Q_m = q*(B*B');
Q_s = q*(B*B');
R = 1;
% Low-pass filter for the derivative in Zh
Flp = 1; %

tlcAlpha = 2;
beta = 0.1;
Hd = 1;
Hm_init = 0.5; % PP free no noise
Hs_init = 0.5; % PP
Hm_init = 0; % PP
Hs_init = 0; % PP
Hm_init = 5; % PP
Hs_init = 5; % PP
%  with noise works good , no change params
% no noise free works good, no change params. contact t30 fc 0.5, xe0.8