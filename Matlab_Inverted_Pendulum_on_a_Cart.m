% Inverted Pendulum on a Cart

% More Information: lucascasaril.me/inverted-pendulum

% Author: Lucas Casaril

clear all; close all; clc

% System
m = 1;
M = 5;
l = 2;
g = -9.81;

% Linearized Matrices: x_dot = A*x + B*u

A = [ 0 1 0 0;
    0 0 m*g/M 0;
    0 0 0 1;
    0 0 (m+M)*g/(M*l) 0];

B = [0; 1/M; 0; 1/(M*l)];

% Cost Functions for the LQR Controller

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];

R = 0.001;

% LQR Controller

K = lqr(A,B,Q,R);

tspan = 0:.001:15;

%Initial Condition
x0 = [-2.4; 0; 0; 1];

%Solver
[t,x] = ode45(@(t,x)dynamical_system(x,m,M,l,g, -K*(x-[0;0;0;0])), tspan, x0);