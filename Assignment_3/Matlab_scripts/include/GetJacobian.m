function J = GetJacobian(biTei, bTe, jointType)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - biTei: vector of matrices containing the transformation matrices from
% joint i-1 to joint i for the current configuration.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix

numberOfLinks = length(jointType);

J = zeros(6,numberOfLinks);

    for i=1:numberOfLinks
        %Angular Jacobian
        if jointType(i) == 0
            [theta, v] = ComputeInverseAngleAxis(bTe(1:3,1:3,i));
            J(1:3,i) = v; % rotational joint
        else
            J(1:3,i) = zeros(3,1); % prismatic joint
        end
        
        %Linear Jacobian
         if jointType(i) == 0
             matr_dist = bTe(1:3,4,7)-bTe(1:3,4,i);
            J(4:6,i) = cross(bTe(1:3,3,i), matr_dist); % rotational joint
        else
            J(4:6,i) = bTe(1:3,3,i); % prismatic joint
        end



    end










end