function Adaptive_control(initial_states, initial_param)
% The function is used to run simulation and generate plots. The input is
% the initial state of manipulator and the initial estimate of parameter.
%
%% ***************** Initial funcition handles and states *****************
% trajectory generator
trajhandle = @tra_generate;

% controller
controlhandle = @controller;

%% ****************** Define the parameter of lynx ************************
param = manipulatorParameter();

%% ****************** Define the real configuration ***********************
q = initial_states(1, :);
qd = initial_states(2, :);
global e edot Kd Kp theta_estimate;
theta_estimate = initial_param;

%% ****************** Define the matrix to restore data *******************
q_restore = zeros(2, 799);
qd_restore = zeros(2, 799);
qdes_restore = zeros(2, 799);
qddes_restore = zeros(2, 799);
theta_restore = zeros(2, 799);
V_restore = zeros(1, 799);
Vdot_restore = zeros(1, 799);
t_matrix = 0:0.05:39.95;

%% ****************** Prepare for simulation ******************************
fprintf('Initializing..../n')
max_iter  = 5000;      % max iteration
max_time = 40;
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
t      = starttime; % current time
x = zeros(1,6);        % initial state vector
x(1:2) = q; x(3:4) = qd;
%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....')
% Main loop
for i = 1:max_iter
    
    timeint = t:tstep:t+cstep;
    
    tic;
    % Iterate over each quad
    if i == 1
        % Initialize quad plot
        [q_des, qd_des, ~]  = trajhandle(0);
        Robot_plot(q_des, false);
        h_title = title(sprintf('iteration: %d, time: %4.2f', i, t));
        hold on;
        drawnow;
        % Run simulation
    else
        [q_des, qd_des, qdd_des]  = trajhandle(t);
        Robot_plot(q_des, false);
        h_title = title(sprintf('iteration: %d, time: %4.2f', i, t));
        hold on;
        qs.pos = q; qs.vel = qd;
        qs.pos_des = q_des; qs.vel_des = qd_des; qs.acc_des = qdd_des;
        theta_est = theta_estimate;
        [tsave,xsave] = ode45(@(t,s)controlhandle(t,s,qs,param,theta_est), timeint, x);
        q = xsave(6,1:2);
        qd = xsave(6,3:4);
        x = xsave(6,:);
         

        % compute V and Vdot
        [V, Vdot] = Vcompute(e, edot, Kp,Kd);

        % save the data
        q_restore(:, i-1) = q;
        qd_restore(:, i-1) = qd;
        qdes_restore(:, i-1) = q_des;
        qddes_restore(:, i-1) = qd_des;
        V_restore(:,i-1) = V;
        Vdot_restore(:,i-1) = Vdot;
        theta_restore(:,i-1) = theta_est;
    end
    % update real figure
    Robot_plot(q, true);
    hold off;
    drawnow;
   
    t = t + cstep; % Update simulation time
    
        
    % Check termination criteria
    if t > max_time
        break
    end
end
%% ************************* Post processing *************************
% Figure compare joint angle
figure;

% joint 1
subplot(1,2,1);
plot(t_matrix,q_restore(1,:));
set(gcf,'Color','w'); 
grid on; hold on;
plot(t_matrix,qdes_restore(1, :),'--')
xlabel('time [sec]'); ylabel('q(t)');
legend('q1','q1des');
title('Joint angle control performance'); 

% joint 2
subplot(1,2,2);
plot(t_matrix,q_restore(2,:));
set(gcf,'Color','w'); 
grid on; hold on;
plot(t_matrix,qdes_restore(2, :),'--')
xlabel('time [sec]'); ylabel('q(t)');
legend('q2','q2des');
title('Joint angle control performance'); 

% Figure compare joint angular velocity
figure;

% Joint 1
subplot(1,2,1);
plot(t_matrix,qd_restore(1,:));
set(gcf,'Color','w'); 
grid on; hold on;
plot(t_matrix,qddes_restore(1, :),'--')
xlabel('time [sec]'); ylabel('qd(t)');
legend('qd1','qd1des');
title('Joint velocity control performance'); 

% Joint 2
subplot(1,2,2);
plot(t_matrix,qd_restore(2,:));
set(gcf,'Color','w'); 
grid on; hold on;
plot(t_matrix,qddes_restore(2, :),'--')
xlabel('time [sec]'); ylabel('qd(t)');
legend('qd2','qd2des');
title('Joint velocity control performance'); 

% Figure for V and Vdot versus time
figure;

% V
subplot(1,2,1);
plot(t_matrix,V_restore(1,:));
set(gcf,'Color','w'); 
grid on; 
xlabel('time [sec]'); ylabel('V');
title('V versus time');

% Vdot
subplot(1,2,2);
plot(t_matrix,Vdot_restore(1,:));
set(gcf,'Color','w'); 
grid on; 
xlabel('time [sec]'); ylabel('Vdot');
title('Vdot versus time'); 

figure;
plot(t_matrix, theta_restore(1,:));
hold on;
plot(t_matrix, theta_restore(2,:));
set(gcf,'Color','w'); 
grid on; 
legend('m1','m2');
xlabel('time [sec]'); ylabel('mass [kg]');
title('Estimate mass versus time'); 
end


%% ***************** Function to initialize parameters ********************
function param = manipulatorParameter()
   param.a1 = 2;
   param.a2 = 1;
   param.grav = 9.8;
end

%% ***************** Function to generate trajectory ********************
function [pos, vel, acc] = tra_generate(t)
% TRAJECTORY GENERATOR
%   t      - the current time
%   pos    - the desired joint position
%   vel    - the desired joint velocity
%   acc    - the desired joint acceleration
pos = [sin(t), -cos(t)];
vel = [cos(t), sin(t)];
acc = [-sin(t), cos(t)];
end

%% ***************** Function to control dynamics ********************
function q_pvap = controller(t, x, qs, param, theta_est)
% CONTROLLER manipulator controller
%   qs     - the current states: qs.pos, qs.vel;
%            the desired states: qs.pos_des, qs.vel_des, qs.acc_des;
%   t      - time, unit is second;
%   param - output from manipulator() storing all robot parameters;
%   tau    - 4 x 1, controller output.
%   Compute the desired controls

global deltadot Kd Kp e edot theta_estimate

% initialize output
q_pvap = zeros(6,1);

% Turning gains 
Kd = 10*eye(2);
Kp = 15*eye(2);
Kd1 = Kd(1,1); Kd2 = Kd(2,2);
Kp1 = Kp(1,1); Kp2 = Kp(2,2);

m1 = theta_estimate(1);
m2 = theta_estimate(2);

a1 = param.a1;
a2 = param.a2;

g = param.grav;

q1 = x(1);
q2 = x(2);  

q1dot = x(3);
q2dot = x(4);

e = x(1:2) - qs.pos_des';
edot = x(3:4) - qs.vel_des';

% calculate the M,C,and N based on the calculation of dynamics
M = [m1*a1^2+m2*(a1^2+a2^2+2*a1*a2*cos(q2)), m2*(a2^2+a1*a2*cos(q2));
     m2*(a2^2+a1*a2*cos(q2)), m2*a2^2];
C = [-m2*a1*a2*sin(q2)*q2dot, -m2*a1*a2*sin(q2)*q1dot-m2*a1*a2*sin(q2)*q2dot;
     m2*a1*a2*sin(q2)*q1dot, 0];
N = [m1*g*a1*cos(q1)+m2*g*(a1*cos(q1)+a2*cos(q2+q1));
     m2*g*a2*cos(q1+q2)];
M_hat = M;
                                                                                                                             
feed_value = qs.acc_des' - Kd*edot - Kp*e;

% compute input value
tau = (M * (feed_value) + C*x(3:4) + N);

% Uncomment this if want to add torque limits
% if tau > 10
%     tau = 10;
% elseif tau < -10
%     tau = -10;
% end

% calculate the M,C,and N based on the real mass
m1 = 1; m2 = 1;
M = [m1*a1^2+m2*(a1^2+a2^2+2*a1*a2*cos(q2)), m2*(a2^2+a1*a2*cos(q2));
     m2*(a2^2+a1*a2*cos(q2)), m2*a2^2];
C = [-m2*a1*a2*sin(q2)*q2dot, -m2*a1*a2*sin(q2)*q1dot-m2*a1*a2*sin(q2)*q2dot;
     m2*a1*a2*sin(q2)*q1dot, 0];
N = [m1*g*a1*cos(q1)+m2*g*(a1*cos(q1)+a2*cos(q2+q1));
     m2*g*a2*cos(q1+q2)];
 
q_pvap(1:2) = x(3:4);
q_pvap(3:4) = (M\(tau - (C*x(3:4) + N)));

% compute adaptive control equation
q1ddot = q_pvap(3);
q2ddot = q_pvap(4);

W = [a1^2*q1ddot+g*a1*cos(q1), (a1^2+a2^2+2*a1*a2*cos(q2))*q1ddot+(a2^2+a1*a2*cos(q2))*q2ddot-a1*a2*sin(q2)*q2dot*q1dot-a1*a2*sin(q2)*q1dot*q2dot-a1*a2*sin(q2)*q2dot^2+g*(a1*cos(q1)+a2*cos(q2+q1))
     0, (a2^2+a1*a2*cos(q2))*q1ddot+a2^2*q2ddot+a1*a2*sin(q2)*q1dot^2+g*a2*cos(q1+q2)];

B = [zeros(2,2);eye(2)];

P = [(Kd1^2 + Kp1^2 + Kp1)/(2*Kd1*Kp1), 0, 1/(2*Kp1),0;
     0, (Kd2^2 + Kp2^2 + Kp2)/(2*Kd2*Kp2),  0, 1/(2*Kp2);
     1/(2*Kp1), 0, (Kp1 + 1)/(2*Kd1*Kp1),  0;
     0, 1/(2*Kp2), 0, (Kp2 + 1)/(2*Kd2*Kp2)];

% update parameter
deltadot = -2*W'*(inv(M_hat))'*B'*P*[e; edot];
q_pvap(5:6) = deltadot(1:2);

theta_estimate = theta_estimate + 0.01 * deltadot';

end

%% ***************** Function compute V *********************
function [V, Vdot] = Vcompute(e, edot, Kp, Kd)
    V = 0.5*Kp(1)*e(1)^2+0.5*Kp(1)*e(2)^2+0.5*edot(1)^2+0.5*edot(2)^2;
    Vdot = -Kd(1) * (edot(1)^2+edot(2)^2);
end
%% ***************** Function to plot robot motion ********************
function Robot_plot(q, real)
   if real == 1
      scatter(2*cos(q(1)), 2*sin(q(1)), 140, 'filled','r');
      hold on;
      scatter(2*cos(q(1))+ cos(q(1)+q(2)), 2*sin(q(1)) + sin(q(1)+q(2)), ...
                                                        140, 'filled','r');
      line([0;2*cos(q(1))], [0;2*sin(q(1))], 'Linewidth', 3, 'color', [1,0,0]);
      line([2*cos(q(1)) + cos(q(1)+q(2));2*cos(q(1))], ...
           [2*sin(q(1)) + sin(q(1)+q(2));2*sin(q(1))], 'Linewidth', 3, 'color', [1,0,0]);
      grid on;
      axis([-3 3 -3 3]);
      xlabel('x [m]'); ylabel('y [m]');
   else
      scatter(2*cos(q(1)), 2*sin(q(1)), 140, 'filled', 'b');
      hold on;
      scatter(2*cos(q(1))+ cos(q(1)+q(2)), 2*sin(q(1)) + sin(q(1)+q(2)), ...
                                                       140, 'filled', 'b');
      line([0;2*cos(q(1))], [0;2*sin(q(1))], 'Linewidth', 3, 'color', [0,0,1]);
      line([2*cos(q(1)) + cos(q(1)+q(2));2*cos(q(1))], ...
           [2*sin(q(1)) + sin(q(1)+q(2));2*sin(q(1))], 'Linewidth', 3, 'color', [0,0,1]);
   end
end
