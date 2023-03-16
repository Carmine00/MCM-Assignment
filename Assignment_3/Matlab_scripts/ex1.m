%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix
clc;
clear;
close all;
addpath('include');

% The same model of assignment 2
geom_model = BuildTree();
numberOfLinks = size(geom_model,3); % number of manipulator's links.
linkType = zeros(numberOfLinks,1); % specify two possible link type: Rotational, Prismatic.
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base

% Initial joint configuration 
q1 = [1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3];
% Compute direct geometry
biTei = GetDirectGeometry(q1,geom_model,linkType);
% Compute the transformation w.r.t. the base
for i =1:numberOfLinks
    bTi(:,:,i)= GetTransformationWrtBase(biTei,i);
end
% computing end effector jacobian 
J1 = GetJacobian(biTei,bTi, linkType);


% Intermediate joint configuration - 1 
q2 = [1.3, 0.4, 0.1, 0, 0.5, 1.1, 0];
% Compute direct geometry
biTei = GetDirectGeometry(q2,geom_model,linkType);
% Compute the transformation w.r.t. the base
for i =1:numberOfLinks
    bTi(:,:,i)= GetTransformationWrtBase(biTei,i);
end
% computing end effector jacobian 
J2 = GetJacobian(biTei,bTi, linkType);


% Intermediate joint configuration - 2 
q3 = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];
% Compute direct geometry
biTei = GetDirectGeometry(q3,geom_model,linkType);
% Compute the transformation w.r.t. the base
for i =1:numberOfLinks
    bTi(:,:,i)= GetTransformationWrtBase(biTei,i);
end
% computing end effector jacobian 
J3 = GetJacobian(biTei,bTi, linkType);


% Final joint configuration 
q4 = [2, 2, 2, 2, 2, 2, 2];
% Compute direct geometry
biTei = GetDirectGeometry(q4,geom_model,linkType);
% Compute the transformation w.r.t. the base
for i =1:numberOfLinks
    bTi(:,:,i)= GetTransformationWrtBase(biTei,i);
end
% computing end effector jacobian 
J4 = GetJacobian(biTei,bTi, linkType);

