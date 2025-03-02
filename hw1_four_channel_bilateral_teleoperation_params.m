% Sample parameters for four channel bilateral teleoperation
clear all;
close all;
clc

% Input function parameter (sin or step with low pass filter)
A = 1;

% Low pass frequency cuff off
Flp = 0.5;
% Sin frequency
Fc = 1; 

% Human intention controller (PD)
Ph = 10*1; 
Dh = 20*0.8; 

% Human impedance parameters
Jh = 1;
Bh = 1; 

% Inertia/Damping of robot dynamics
Mm = 0.5;
Ms = 2;
Dm = 0;
Ds = 0;
% Dm = 5;
% Ds = 10;


% Master controller
Bm = 20*0.8;
Km = 10*1;

% Slave controller
Bs = 4*Bm; 
Ks = 4*Km; 

% Environment impedance parameters
Be = 100; 
Ke = 200; 
xe = 5;
xe = 0.7;

s = tf('s');
Ts = 0.001;
