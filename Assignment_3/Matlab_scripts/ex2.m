%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
clear all
close all
clc
addpath('include')
model = load("panda.mat"); % don't worry about the warnings
% Simulation Parameters
ts = 0.5;
t_start = 0.0;
t_end = 30.0;
t = t_start:ts:t_end;

% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Initial transformation from <base> to <e-e>
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7'); %useful for save initial end-effector orientation w.r.t robot base
% END-EFFECTOR Goal definition 
bOge = [0.6; 0.4; 0.4];
theta = -pi/4;
eRge = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]; %elementary rotation of theta around the z-axis
bRge = bTe(1:3,1:3)*eRge;
bTge = [bRge bOge; 0 0 0 1];


% TOOL Goal definition
bOgt = [0.6; 0.4; 0.4];
eTt = [eye(3) [0,0,0.2]'; 0 0 0 1];
bTt = bTe*eTt;
tRgt = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]; %elementary rotation of theta around the z-axis
%bTgt = [bTt(1:3,1:3)*tRgt bOgt; 0 0 0 1];
r_skew =[0 -eTt(3,4) eTt(2,4) ; eTt(3,4) 0 -eTt(1,4) ; -eTt(2,4) eTt(1,4) 0 ]; 
tSe = [eye(3) zeros(3,3); r_skew eye(3)];

%new goal set 
bTgt = [0.9986 -0.0412 -0.0335 0.6; 0.0329 -0.0163 0.9993 0.4; -0.0417 -0.9990 -0.0149 0.4; 0 0 0 1]; 

% Control Proportional Gain 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
x_dot = zeros(6,1);
error_linear = zeros(3,1);
error_angular = zeros(3,1); 
% Start the inverse kinematic control 
tool = false; % change to true for using the tool 
q = q_init;
for i = t
    %% Compute the cartesian error to reach the goal
    if tool == true %compute the error between the tool frame and goal frame
        
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bTt = bTe*eTt;
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tSe*tmp(1:6,1:7); %DO NOT EDIT, this would be bJt
        tRgt = bTt(1:3,1:3)'*bTgt(1:3,1:3); 
        
        
    else % compute the error between the e-e frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        eRge = bTe(1:3,1:3)'*bRge;
    end

    %% Compute the reference velocities
    if tool == true
        [theta, h] = ComputeInverseAngleAxis(tRgt);
        error_linear = bTgt(1:3,4) - bTt(1:3,4);
    else
        [theta, h] = ComputeInverseAngleAxis(eRge);
        error_linear = bTge(1:3,4) - bTe(1:3,4);
    end
     error_angular = bTe(1:3,1:3)*h*theta;
     b_omega_e = angular_gain*error_angular;% desired angular velocity of the end-effector w.r.t the base
     b_v_e = linear_gain*error_linear;% desired linear velocity of the end-effector w.r.t the base
     x_dot = [b_omega_e; b_v_e];
     
   
    %% Compute desired joint velocities 
    q_dot = pinv(bJe)*x_dot;
 
    %% Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q(1:7), q_dot,ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    show(model.franka,[q',0,0],'visuals','on');%switch visuals to off for seeing only the frames
    hold on
    if tool == true
        plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
        plot3(bOgt(1),bOgt(2),bOgt(3),'ro','LineWidth',5);
    else
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOge(1),bOge(2),bOge(3),'ro','LineWidth',5);
    end
    drawnow
    if(norm(x_dot) < 0.01)
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
end


