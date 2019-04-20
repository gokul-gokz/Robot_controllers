%% TODO: GENERATE TRAJECTORY USING TwoLinkArmTraj matlab file.
%% You need to specify the initial state, final state (in the state variables of the state space form), the time span.
%% If quintic trajectories are to be generated, consider specify the desired accelerations for all joints.
%% Link1
t0=0;
tf=10;
q1_0=0;
q1_f=60;
v1_0=5;
v1_f=0;
t = linspace(t0,tf,100);
c = ones(size(t));
M = [ 1 t0 t0^2 t0^3;
0 1 2*t0 3*t0^2;
1 tf tf^2 tf^3;
0 1 2*tf 3*tf^2];
b = [q1_0; v1_0; q1_f; v1_f];
a1 = inv(M)*b;
%%
qd1 = a1(1).*c + a1(2).*t +a1(3).*t.^2 + a1(4).*t.^3;
vd1 = a1(2).*c +2*a1(3).*t +3*a1(4).*t.^2;
ad1 = 2*a1(3).*c + 6*a1(4).*t;


subplot(2,1,1);
plot(t,qd1);
title('Link1theta');
legend('desired');
hold on;

%% Inverse Dynamic control
%Initial estimate for phi
I=8;
mgd=5;
fv=2.5;

alpha=[mgd;fv;I]
% I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
% x=[q1_0+5;q2_0+2;v1_0;v2_0];

% %% Calculating dx without using ODE function
% % Instead of ODE function, I have approximated the values by multiplying dy
% % with dt
% z=[];
% for i=1:100
%     z=[z,x];
%     dx=ode2linkTracking(t(i),x,[a1,a2]',[I1,I2,m1,r1,m2,r2,l1,l2]);
%     x=x+dx*(tf-t0)/100;
% end
% 
% %% Plotting the result 
% % For Link 1
% subplot(2,1,1);
% plot(t,z(1,:));
% hold on;
% 
% % For Link 2
% subplot(2,1,2);
% plot(t,z(2,:));
% hold on;

%% Implement the inverse dynamic control  with ODE function
x0=[q1_0+5;q2_0+2;v1_0;v2_0];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) ode2linkTracking(t,x,[a1]'),[0 tf],alpha);

%% Plotting the result:
 figure('Name','Theta_1 under inverse dynamic control');
 plot(T, X(:,1),'r-');
 hold on
 plot(t,qd1);
 hold on;
 legend('actual', 'desired')
 figure('Name','Theta_2 under inverse dynamic control');
 plot(T, X(:,2),'r--');
 hold on
 plot(t,qd2);
 hold on;
 legend('actual', 'desired')