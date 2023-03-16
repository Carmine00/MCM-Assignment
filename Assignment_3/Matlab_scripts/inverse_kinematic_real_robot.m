%% Modelling and Control of Manipulator assignment 3: Inverse Kinematic Control
addpath('include')
model = load("panda.mat");
% Simulation Parameters
ts = 0.005;
t_start = 0.0;
t_end = 30.0;
t = t_start:ts:t_end;
real_robot = false;
if real_robot == true
    hudpr = dsp.UDPReceiver('LocalIPPort',1503,'MaximumMessageLength',255);
    hudps = dsp.UDPSender('RemoteIPPort',1502);
    hudps.RemoteIPAddress = '127.0.0.1';
else
    hudps = dsp.UDPSender('RemoteIPPort',1505);% for work with the simulator in the lab
    hudps.RemoteIPAddress = '127.0.0.1';
end
% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Initiale transformation from <base> to <e-e>
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7'); %useful for save initial end-effector orientation w.r.t robot base

% Goal definition
bOg = [0.6; 0.0; 0.5];
bRg = [1 0 0; 0 -1 0; 0 0 -1];
bTg = [bRg bOg;0 0 0 1];

% Fixed transform from <ee> to <tool>
eRt = rotation(0,0,-0.7853087545);
eOt = [0,0,0.25]';
% Tool Definition
%eTt = ...;
%bTt = ...;

%Second goal Definition
bTg2 = [0.9986 -0.0412 -0.0335 0.6; 0.0329 -0.0163 0.9993 0.4; -0.0417 -0.9990 -0.0149 0.4; 0 0 0 1]; 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
x_dot = zeros(6,1);
error_linear = zeros(3,1);
error_angular = zeros(3,1); 
% Start the inverse kinematic control 
tool = true;
q = q_init;
for i = t
    % Read data from socket
    if real_robot == true
        data = step(hudpr);
        % wait for data (to decide)
         if i == 0
             while(isempty(data))
                 data = step(hudpr);
                 pause(deltat);
             end
         end

        q = typecast(data, 'double');
    end

    if tool == true
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        % Compute the cartesian error to reach the goal

    else
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        
        
    end
    %% Compute the reference velocities
    
    %% Compute desired joint velocities 
    if tool == true
        %q_dot = ...;
    else
        %q_dot = ...;
    end
    %% Move the robot - DO NOT EDIT
    if real_robot == true
        % Send directly the velocities to the real robot
        step(hudps,[t;q_dot]);
    else 
        % Integrate then send to simulator
        q = KinematicSimulation(q(1:7), q_dot,ts, qmin, qmax);
        step(hudps,[0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872,q']);
    end
    % Add debug prints here
    norm(x_dot)
    if(norm(x_dot) < 0.01)
        ang = error_angular;
        lin = error_linear;
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
    SlowdownToRealtime(ts);
end
