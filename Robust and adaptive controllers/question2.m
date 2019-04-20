%% Trajectory generation
%% Link1
t0=0;
tf=20;
q1_0=0;
q1_f=15;
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
%a1=[0 0 0 0]
%%
qd1 = a1(1).*c + a1(2).*t +a1(3).*t.^2 + a1(4).*t.^3;
vd1 = a1(2).*c +2*a1(3).*t +3*a1(4).*t.^2;
ad1 = 2*a1(3).*c + 6*a1(4).*t;


%Initial estimate for phi and system values
I=8;
mgd=5;
fv=2.5;
alpha=[0;0;mgd;fv;I];


%% Implement the adaptive control  with ODE function
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) ode1linkTracking_adaptive(t,x,a1'),[0 tf],alpha,options);

%% Plotting the result:
 figure('Name','Theta under Adaptive control');
 
 plot(T, X(:,1),'r-');
 xlabel('t')
 ylabel('theta')
 hold on
 plot(t,qd1);
 hold on;
 legend('actual', 'desired')
 
 figure('Name','ddTheta under Adaptive control');
 
 plot(T, X(:,2),'r-');
 xlabel('t')
 ylabel('ddtheta')
 hold on
 plot(t,vd1);
 hold on;
 legend('actual', 'desired')
 
 figure('Name','Error in Mgd over time for adaptive control');
 plot(T, (X(:,3)-1),'r-');
 xlabel('t')
 ylabel('e in Mgd')
 hold on
 
 
 figure('Name','Error in fv over time for adaptive control');
 plot(T, (X(:,4)-1),'r-');
 xlabel('t')
 ylabel('e in fv')
 hold on
 
 
 figure('Name','Error in I over time for adaptive control');
 plot(T, (X(:,5)-7.2),'r-');
  xlabel('t')
 ylabel('e in I')
 hold on
 
 
 %% Implement the robust control  with ODE function
 alpha=[3;0];
 options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4]);
 [T,X4] = ode45(@(t,x) ode1linkTracking_robust(t,x,a1'),[0 tf],alpha,options);
 
 %% Plotting the result:
 figure('Name','Theta under robust control');
 plot(T, X4(:,1),'r-');
 xlabel('t')
 ylabel('theta')
 hold on
 plot(t,qd1);
 hold on;
 legend('actual', 'desired')
 
 figure('Name','dTheta under robust control');
 plot(T, X4(:,2),'r-');
 xlabel('t')
 ylabel('ddtheta')
 hold on
 plot(t,vd1);
 hold on;
 legend('actual', 'desired')
 
%% Summary
% 1.From the plot, the adaptive controller trajectory tracking looks little
% jittery when compared to robust control trajectory.
% 2.But when the initial starting positions are different then the adaptive
% is not able to cope up with the actual trajectory and has a lot of
% oscillation whereas the robust is able to follow the trajectory better than adaptive.
% 3.The parameters in the adaptive control are to be tuned regularly whereas in the robust which are less.
% 4. In adaptive control, the velocity and the acccleration profile has
% more oscillations whereas in robust it is less.
 