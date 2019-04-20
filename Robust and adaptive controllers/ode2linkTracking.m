function [ dx ] = ode2linkTracking( t,x,param )
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
        
        % True dynamics of the system
        I=8;
        mgd=5;
        fv=2.5;
        
        %Extracting the coefficients of the trajectory
        a1=param(1,:);
        
        
        %Create the actual trajectory
        vec_t = [1; t; t^2; t^3]; 
        theta_d= [a1*vec_t];
        
        % compute the velocity and acceleration in both theta 1 and theta2.
        a1_vel = [a1(2), 2*a1(3), 3*a1(4), 0];
        a1_acc = [2*a1(3), 6*a1(4),0,0 ];
        
        
        % compute the desired trajectory (assuming 3rd order polynomials for trajectories)
        dtheta_d =[a1_vel*vec_t];
        ddtheta_d =[a1_acc*vec_t];
        theta= x(1:2,1);
        dtheta= x(3:4,1);
        
        
        
        %% The actual dynamic model of the system:
       
       
        
       
        %% PD controller
        persisitent X_previous
        
        KP=10;
        KD=15;
        gamma=[1 0 0;0 1 0;0 0 1]
        P=
        K=[0 1 0 0 0;-KP -KD 0 0 0;0 0 0 0 0;0 0 0 0 0;0 0 0 0 0];
        phi=(1/I)*[sin(theta_d);dtheta_d;ddtheta_d]
        alpha_actual=[mgd;fv;I]
        phi_cap=-inv(gamma)*trans(phi)*[0 1]*P*X_previous(2,:)
        X=-K*[theta-theta_d;dtheta-dtheta_d;0]+[0;1]*phi*(alpha-alpha_actual);
        
        
        
            
       %% Calculate dx based on the dynamics of the robot 
      
       dx=[dtheta;invM*tor-invMC*theta-invM*Gmat]
        

end

