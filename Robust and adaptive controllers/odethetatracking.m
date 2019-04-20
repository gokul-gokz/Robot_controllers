function [ dx ] = odethetatracking( t,X)
%% Implementation
        
        %Controller equation
        theta=10;
        theta_cap =X(1);
        x=X(2);
        theta_cap_dot=x*x;
        x_dot= theta*x -(theta_cap+1)*x;
        
       %% Calculate x_dot and estimate of the dynamics of the robot 
      
       dx=[theta_cap_dot;x_dot];
              
       
                  
       
end

