function [ dx ] = ode2linkPD(t,x, param)
% note x is in the form of q_1, q_2,dot q_1, dot q_2
        
        a = I1+I2t+m1*r1^2+ m2*(l1^2+ r2^2);
        b = m2*l1*r2;
        d = I2+ m2*r2^2;
        % the actual dynamic model of the system:
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        Gmat =  [m1*g*r1*cos(x(1))+m2*g*(l1*cos(x(1))+r2*cos(x(1)+x(2)));
    m2*g*r2*cos(x(1)+x(2))];
        invM = inv(Mmat);
        invMC = invM*Cmat;
        % TODO: compute the control input for the system, which
        % should provide the torques
        
        % use the computed torque and state space model to compute
        % the increment in state vector.
        %TODO: compute dx = f(x,u) hint dx(1)=x(3); dx(2)= x(4); the rest
        %of which depends on the dynamic model of the robot.


end

