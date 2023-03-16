%% Exercises Modelling Part 1
% Rotation matrices, Equivalent angle-axis representations, Quaternions
addpath('include') %%DO NOT CHANGE STUFF INSIDE THIS PATH

%% Exercise 1
if 1 == 0
% 1.2.
close all
clear all
clc

v = [1 0 0];
theta = pi/6;

aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.2:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.2:');disp(theta);
disp('v ex 1.2:');disp(v);


% 1.3.
v = [0 1 0];
theta = pi/4;

aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.3:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.3:');disp(theta);
disp('v ex 1.3:');disp(v);


% 1.4.
v = [0 0 1];
theta = pi/2;

aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.4:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.4:');disp(theta);
disp('v ex 1.4:');disp(v);

% 1.5.
v = [0.408 0.816 -0.408];
v_unit = v/norm(v);
theta = 0.2449;

aRb = ComputeAngleAxis(theta, v_unit);

disp('aRb ex 1.5:');disp(aRb);
plotRotation(theta,v_unit,aRb);
disp('theta ex 1.5:');disp(theta);
disp('v ex 1.5:');disp(v_unit);

% 1.6.
rho = [0 pi/2 0];
theta = norm(rho);
v = rho/theta;


aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.6:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.6:');disp(theta);
disp('v ex 1.6:');disp(v);

% 1.7.
rho = [0.4 -0.3 -0.3];
theta = norm(rho);
v = rho/theta;

aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.7:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.7:');disp(theta);
disp('v ex 1.7:');disp(v);

% 1.8.
rho = [-pi/4 -pi/3 pi/8];
theta = norm(rho);
v = rho/theta;

aRb = ComputeAngleAxis(theta, v);

disp('aRb ex 1.8:');disp(aRb);
plotRotation(theta,v,aRb);
disp('theta ex 1.8:');disp(theta);
disp('v ex 1.8:');disp(v);

end

%% Exercise 2

    % 2.1     
    wRa = eye(3); % Since w and a coincide, there's no rotation between them 
    % (theta = 0) and therefore wRa is equal to the identity
    v = [0 0 1];
    theta = pi/2;
    wRb = ComputeAngleAxis(theta, v);
    aRb = wRa'*wRb;

    % 2.2
    % Compute the inverse equivalent angle-axis repr. of aRb 
    [theta, v] = ComputeInverseAngleAxis(aRb);
    % Plot Results
    disp('aRb ex 2.1:');disp(aRb);
    plotRotation(theta,v,aRb);
    disp('theta ex 2.2:');disp(theta);
    disp('v ex 2.2:');disp(v); 

    % 2.3
    wRc = [0.835959, -0.283542, -0.469869; 
        0.271321, 0.957764, -0.0952472;
        0.47703, -0.0478627, 0.877583];

    cRb = wRc'*wRb;
    % Compute inverse equivalent angle-axis repr. of cRb
    [theta, v] = ComputeInverseAngleAxis(cRb);
    % Plot Results
    disp('cRb ex 2.3:');disp(cRb);
    plotRotation(theta,v,cRb);
    disp('theta ex 2.3:');disp(theta);
    disp('v ex 2.3:');disp(v); 


%% Exercise 3

% 3.1
    % a
        %rotation matrix from <w> to frame <b> by rotating around z-axes
        theta_z = pi/6;
        wRb_z = [cos(theta_z), -sin(theta_z), 0; 
            sin(theta_z), cos(theta_z), 0;
            0, 0, 1];
    % b
        %rotation matrix from <w> to frame <b> by rotating around y-axes
        theta_y = pi/4;
        wRb_y = [cos(theta_y), 0, sin(theta_y); 
            0, 1, 0;
            -sin(theta_y), 0, cos(theta_y)];
    % c
        %rotation matrix from <w> to frame <b> by rotating around x-axes
        theta_x = pi/12;
        wRb_x = [1, 0, 0;
            0, cos(theta_x), -sin(theta_x);
            0, sin(theta_x), cos(theta_x)];
    disp('es 3.1:');disp(wRb_z);disp(wRb_y);disp(wRb_x);
    
% 3.2
    % a
        [theta, v] = ComputeInverseAngleAxis(wRb_z);
        % Plot Results
        plotRotation(theta,v,wRb_z);
        disp('theta ex 3.2.a:');disp(theta);
        disp('v ex 3.2.a:');disp(v); 
    % b
        [theta, v] = ComputeInverseAngleAxis(wRb_y);
        % Plot Results
        plotRotation(theta,v,wRb_y);
        disp('theta ex 3.2.b:');disp(theta);
        disp('v ex 3.2.b:');disp(v); 
    % c
        [theta, v] = ComputeInverseAngleAxis(wRb_x);
        % Plot Results
        plotRotation(theta,v,wRb_x);
        disp('theta ex 3.2.c:');disp(theta);
        disp('v ex 3.2.c:');disp(v); 
% 3.3 
    % Compute the rotation matrix corresponding to the z-y-x representation;
    Rxyz = wRb_x*wRb_y*wRb_z;
    % Compute equivalent angle-axis repr.
    [theta, v] = ComputeInverseAngleAxis(Rxyz);
    % Plot Results
    plotRotation(theta,v,Rxyz);
    disp('theta ex 3.3:');disp(theta);
    disp('v ex 3.3:');disp(v);
% 3.4
    % Compute the rotation matrix corresponding to the z-x-z representation;
    Rzxz = wRb_z*wRb_x*wRb_z;
	% Compute equivalent angle-axis repr.
    [theta, v] = ComputeInverseAngleAxis(Rzxz);
    % Plot Results
    plotRotation(theta,v,Rzxz);
    disp('theta ex 3.4:');disp(theta);
    disp('v ex 3.4:');disp(v);


%% Exercise 4
    a = 0.8924; 
    b = 0.23912;
    c = 0.36964;
    d = 0.099046;
    % Compute the rotation matrix associated with the given quaternion with
    % a custom function
    rotMatrix = quatToRot(a,b,c,d);
    disp('rot matrix es 4.1');disp(rotMatrix)
    % Using matlab functions quaternion(), quat2rotm(),
    quat = quaternion(a,b,c,d);
    rotMatrix = quat2rotm(quat);
    % Evaluate angle-axis representation and display rotations
    [theta, v] = ComputeInverseAngleAxis(rotMatrix);
    % Plot Results
    plotRotation(theta,v,rotMatrix);
    disp('rot matrix es 4.2');disp(rotMatrix)

