%% Function to perform the Iterative control
function [ dx ] = ILcontrol(t,x,system_params)

%% PD controller
% note x is in the form of q_1, q_2,dot q_1, dot q_2
% Condition to check whether the system has achieved the steady state 
if (x(3)==0 && x(4) == 0)
    % Variable to store the previous input value
    persistent current_u;
    if isempty(current_u)
        current_u=0;
    end
    
    % Initialize the gain values
    KP=200;
    K=[KP*eye(2)];
    
    %Calculate the input
    u=-K*[x(1);x(2)]+current_u;
    current_u=u;
    
    %Update the dx matrix and return
    dx=[x(3);x(4);u];
    
else
    % If the system has attained the steady state
    % Initialize the gain values
    KP=24;
    KD=21;
    K=[KP*eye(2), KD*eye(2)];
    
    % Calculate the input
    u=-K*[x];
    
    %Update the dx matrix
    dx=[x(3);x(4);u];
end
end

