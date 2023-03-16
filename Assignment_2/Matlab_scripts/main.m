%% Modelling and Control of Manipulator assignment 2: Manipulator Geometry and Direct kinematic
clc;
clear;
close all;
addpath('include');


%% 1.
% You will need to define all the model matrices, and fill the so called iTj matrices inside BuildTree() function 
% Be careful to define the z-axis coinciding with the joint rotation axis,
% and such that the positive rotation is the same as showed in the CAD model you received.
geom_model = BuildTree();

% Useful initizializations
numberOfLinks = 7;                  % number of manipulator's links.
linkType = [0 0 0 0 0 0 0];         % boolean that specifies two possible link types: Rotational (0), Prismatic(1).
bri= zeros(3,numberOfLinks);        % Basic vector of i-th link w.r.t. base
bTi = zeros(4,4,numberOfLinks);     % Trasformation matrix i-th link w.r.t. base

iTj = zeros(4,4,1);

% Initial joint configuration 
q = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];

% Q1.1 and Q1.2
biTei = GetDirectGeometry(q,geom_model,linkType);

%Q1.3
for i =1:numberOfLinks
    bTi(:,:,i)= GetTransformationWrtBase(biTei,i);
end

% two frames are chosen and the T matrix between them is computed
linkNumber_i = 2;
linkNumber_j = 6;
iTj = GetFrameWrtFrame(linkNumber_i,linkNumber_j,biTei);

for i = 1:numberOfLinks
    bri(:,i) = GetBasicVectorWrtBase(biTei,i);
end

% Q1.4
numberOfSteps =10;
%1
qi = q;
qf = [2, 2, 2, 2, 2, 2, 2];
q_discr = linspace(1.3,2,numberOfSteps);

for i = 1:numberOfSteps
%-------------------MOVE----------------------%
biTei = GetDirectGeometry(q_discr(i)*ones(numberOfLinks),geom_model,linkType);

    for j =1:numberOfLinks
        bri(:,j) = GetBasicVectorWrtBase(biTei,j);
    end
    
    figure(1)
    plot3(bri(1,:),bri(2,:),bri(3,:),'o')
    hold on
    line(bri(1,:),bri(2,:),bri(3,:));
    pause(0.5);
    
end

%2
qi= [1.3, 0, 1.3, 1.7, 1.3, 0.8, 1.3];
qf = [2, 0, 1.3, 1.7, 1.3, 0.8, 1.3];
q_discr = linspace(1.3,2,numberOfSteps);

for i = 1:10
%-------------------MOVE----------------------%
biTei = GetDirectGeometry([q_discr(i) qi(2:end)],geom_model,linkType);

    for j =1:numberOfLinks
        bri(:,j) = GetBasicVectorWrtBase(biTei,j);
    end
    
    figure(2)
    plot3(bri(1,:),bri(2,:),bri(3,:),'o')
    hold on
    line(bri(1,:),bri(2,:),bri(3,:));
    pause(0.5);
    
end

%3
qi = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];
qf = [2, 2, 2, 2, 2, 2, 2];
q_discr = zeros(numberOfSteps, numberOfLinks); % dim = 10x7

% compute the discrete q for each joint and each step
for i = 1:numberOfLinks
    q_discr(:,i) = linspace(qi(i),qf(1),numberOfSteps)';
end

for i = 1:numberOfSteps
%-------------------MOVE----------------------%
biTei = GetDirectGeometry(q_discr(i,:),geom_model,linkType);

    for j =1:numberOfLinks
        bri(:,j) = GetBasicVectorWrtBase(biTei,j);
    end
    
    figure(3)
    plot3(bri(1,:),bri(2,:),bri(3,:),'o')
    hold on
    line(bri(1,:),bri(2,:),bri(3,:));
    pause(0.5);
    
end

%Q1.5 - three configuration in which just one joint at time changes
% each row is a configuration and each value a change in a specific joint starting from the first
q_conf = [1.3 0 0 0 0 0 0; 2 0 0 0 0 0 0; 2 0.5 0.5 0.5 0.5 0.5 0.5]; 

for i=1:3 % loop for the number of configuration
    
    for n = 1:numberOfLinks-1 % compute and plot for the first six links
    biTei = GetDirectGeometry(q_conf(i,:),geom_model,linkType);

    for j = 1:numberOfLinks-1 % when joints are fixed, compute bri
        bri(:,j) = GetBasicVectorWrtBase(biTei,j);
    end

    figure(3+i)
    subplot(3,2,n)
    plot3(bri(1,:),bri(2,:),bri(3,:),'o')
    hold on
    line(bri(1,:),bri(2,:),bri(3,:));
    title("q = " + "["+strjoin(string(q_conf(i,:)))+"]");

    q_conf(i,:) = circshift(q_conf(i,:),1); % change the moving joint
    end


end