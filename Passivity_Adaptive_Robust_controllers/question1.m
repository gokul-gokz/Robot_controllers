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
alpha=[4;1;mgd;fv;I];


%% Implement the passivity based adaptive control  with ODE function
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4,1e-4,1e-4]);
[T,X] = ode45(@(t,x) ode1linkTracking_passivity_adaptive(t,x,a1'),[0 tf],alpha,options);

%% Plotting the result:
 figure('Name','Theta under passivity based Adaptive control');
 
 plot(T, X(:,1),'r-');
 xlabel('t')
 ylabel('theta')
 hold on
 plot(t,qd1);
 hold on;
 legend('actual', 'desired')
 
 figure('Name','dTheta under Passivity based Adaptive control');
 
 plot(T, X(:,2),'r-');
 xlabel('t')
 ylabel('dtheta')
 hold on
 plot(t,vd1);
 hold on;
 legend('actual', 'desired')

 %% Implement the passivity based robust control  with ODE function
 
 options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4, 1e-4]);
 [T,X4] = ode45(@(t,x) ode1linkTracking_passitvity_robust(t,x,a1'),[0 tf],alpha,options);
 
 %% Plotting the result:
 figure('Name','Theta under Passivity based robust control');
 plot(T, X4(:,1),'r-');
 xlabel('t')
 ylabel('theta')
 hold on
 plot(t,qd1);
 hold on;
 legend('actual', 'desired')
 
 figure('Name','dTheta under passivity based robust control');
 plot(T, X4(:,2),'r-');
 xlabel('t')
 ylabel('ddtheta')
 hold on
 plot(t,vd1);
 hold on;
 legend('actual', 'desired')
 
%% Summary
% 1.From the plot, the passivity based adaptive controller tracking looks
% better than the passivity based robust controller. Eventhough the error in the starting
% state and dynamics are larger, still it converges quicker.
% 
% 2.Whereas the robust controller is not able to exactly follow the
% the desired trajectory. 
% 
% 3.Comparing the inverse dynamic adaptive and passivity based adaptive,
% the latter one is able to perform better eventhough the error is large.
%
% 4. Comparing the inverse dynamic robust and passivity based robust, the
% velocity profile of the passivity based is not smooth and also the it's
% very sensitive the parameters. So, tuning is much difficult in robust.
 