function [biTei] = DirectGeometry(qi, biTri, jointType)
% DirectGeometry Function 

% inputs: 
% q : current link position;
% biTri is the constant transformation between the base of the link <i>
% and its end-effector; 
% jointType :0 for revolute, 1 for prismatic

% output :
% biTei : transformation between the base of the joint <i> and its end-effector taking 
% into account the actual rotation/traslation of the joint

if jointType == 0 % rotational
    % with respect to the initial configuration of the transformation matrix 
    % we have a change in the rotation matrix only
    Rz = [cos(qi) -sin(qi) 0; sin(qi) cos(qi) 0; 0 0 1]; %rotation matrix for the axis given qi
    % rotation matrix of the resulting transformation matrix
    biRri = biTri(1:3,1:3)*Rz; % product of the rotation of the frames w.r.t one another 
    % in the inital configuration and the rotation joint axis
    biTei = [biRri biTri(1:3,4); 0 0 0 1];
elseif jointType == 1 % prismatic
    % with respect to the initial configuration of the transformation matrix 
    % we have a change in the distance vector between the joints because of a
    % displacement
    r = biTri(1:3,4) + biTri(1:3,3)*qi; 
    biTei = [biTri(1:3,1:3) r; 0 0 0 1];
end
end