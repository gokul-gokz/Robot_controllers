%% Dynamics of the system
% Notations: For a given variable, x, dx is its time derivative, ddx is
% 2nd-order derivative. 
clc
clear all;
close all;
% the following parameters for the arm
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
g=9.8;

% we compute the parameters in the dynamic model
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

symx= sym('symx',[4,1]); 

M = [a+2*b*cos(symx(2)), d+b*cos(symx(2));
    d+b*cos(symx(2)), d];
C = [-b*sin(symx(2))*symx(4), -b*sin(symx(2))*(symx(3)+symx(4)); b*sin(symx(2))*symx(3),0];
G = [m1*g*r1*cos(symx(1))+m2*g*(l1*cos(symx(1))+r2*cos(symx(1)+symx(2)));
    m2*g*r2*cos(symx(1)+symx(2))];


invM = inv(M);
invMC= inv(M)*C;

% the options for ode
% initial condition % feel free to change
x0= [-0.5,0.2,0.1,0.1];



%% PD+ GRAVITY COMPENSATION control for set point tracking.
xf = [0, 0, 0, 0];
tf=30;
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) PDControlGravity(t,x,[I1,I2,m1,r1,m2,r2,l1,l2]),[0 tf],x0, options);

figure('Name','Theta_1 under PD SetPoint Control');
plot(T, X(:,1),'r-');
hold on

figure('Name','Theta_2 under PD SetPoint Control');
plot(T, X(:,2),'r--');
hold on


%% Iterative Learning control set point tracking
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) ILcontrol(t,x),[0 tf],x0, options);
tf=5;

figure('Name','Theta_1 under iterative learning control');
plot(T, X(:,1),'r-');
hold on
figure('Name','Theta_2 under iterative learning control');
plot(T, X(:,2),'r--');
hold on


