%% Initial conditions
theta_initial=8;
x_initial=0.5;
x0=[8;1];
x1=[5;1];
x2=[15;1];
xf=10;
tf=10;
%% ODE tracking with adaptive control
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4]);
[T0,X0] = ode45(@(t,x) odethetatracking(t,x),[0 tf],x0,options);

[T1,X1] = ode45(@(t,x) odethetatracking(t,x),[0 tf],x1,options);

[T2,X2] = ode45(@(t,x) odethetatracking(t,x),[0 tf],x2,options);


%% Plotting the results
figure('Name','Plotting theta vs t');
plot(T0, X0(:,1),T1, X1(:,1),T2, X2(:,1));
ylabel('theta');
xlabel('t');
legend({'theta=8','theta=5','theta=15'},'Location','southeast');



figure('Name','Plotting x vs t');
plot(T0, X0(:,2),T1, X1(:,2),T2, X2(:,2));
ylabel('x');
xlabel('t');
legend({'theta=8','theta=5','theta=15'},'Location','northeast');

