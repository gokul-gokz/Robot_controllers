function [ dx ] = ode2linkTracking( t,x,param,system_params )
%% Summary:
% The input for the function is the time frame, 
% initial conditions for the trajectory where we are inducing a small error to check whether the
% trajectory is converging or not.
% param - coefficients of the trajectory parameters
% system params - passing the sysyem parameters 
% Based on the coefficient of the trajectories and initial condition , an
% actual trajectory is generated.
% Then in the PD controller ,
% 1. KP is set to 10 and KD is set to 5
% 2. Calculated the error in theta and dtheta
% 3. Calculated the Aq value based on kp,kd,error and ddtheta.[Formula from
% lecture]
% 4. Calculated the torque value based on Aq and the dynamic equation.
% 5. Calculated the ddtheta values by inverse dynamics equation and stored
% in dx matrix.
% 6. Finally dx matrix is returned.
%   
% note x is in the form of q_1, q_2,dot q_1, dot q_2
%% Implementation
        % Extracting the system params
        I1=system_params(1);  I2 = system_params(2); m1=system_params(3); r1=system_params(4); m2=system_params(5); r2=system_params(6); l1=system_params(7); l2=system_params(8);
        g=9.8;
        
        %Extracting the coefficients of the trajectory
        a1=param(1,:);
        a2=param(2,:);
        
        %Create the actual trajectory
        vec_t = [1; t; t^2; t^3]; 
        theta_d= [a1*vec_t; a2*vec_t];
        
        % compute the velocity and acceleration in both theta 1 and theta2.
        a1_vel = [a1(2), 2*a1(3), 3*a1(4), 0];
        a1_acc = [2*a1(3), 6*a1(4),0,0 ];
        a2_vel = [a2(2), 2*a2(3), 3*a2(4), 0];
        a2_acc = [2*a2(3), 6*a2(4),0,0 ];
        
        % compute the desired trajectory (assuming 3rd order polynomials for trajectories)
        dtheta_d =[a1_vel*vec_t; a2_vel* vec_t];
        ddtheta_d =[a1_acc*vec_t; a2_acc* vec_t];
        theta= x(1:2,1);
        dtheta= x(3:4,1);
        
        a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
        b = m2*l1*r2;
        d = I2+ m2*r2^2;
        
        %% The actual dynamic model of the system:
       
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        Gmat =  [m1*g*r1*cos(x(1))+m2*g*(l1*cos(x(1))+r2*cos(x(1)+x(2)));
    m2*g*r2*cos(x(1)+x(2))];
        invM = inv(Mmat);
        invMC = invM*Cmat;
        
       
        %% PD controller
        
        KP=10;
        KD=15;
        K=[KP*eye(2), KD*eye(2)];
        AQ=-K*[theta-theta_d;dtheta-dtheta_d]+ddtheta_d;
        tor=Mmat*AQ+Cmat*theta_d+Gmat;
        
            
       %% Calculate dx based on the dynamics of the robot 
      
       dx=[dtheta;invM*tor-invMC*theta-invM*Gmat]
        

end

