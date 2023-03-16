function [rot_matrix] = quatToRot(q0,q1,q2,q3)
% quatToRot convert a quaternion into a full-three dimensional rotation matrix

    %Input
    %:param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    %Output
    %return: A 3x3 element matrix representing the full 3D rotation matrix. 
    epsilon = [q1 q2 q3];
    epsilon_skew =[0 -epsilon(3) epsilon(2) ; epsilon(3) 0 -epsilon(1) ; -epsilon(2) epsilon(1) 0 ];
    epsilon_skew_sq = 2*epsilon_skew*epsilon_skew;

    %Each row is explicitly computed in order to show where each component of the
    %quaternion appears the matrix

    %First row of the rotation matrix
    row_1 = [1++epsilon_skew_sq(1,1) 2*q0*epsilon_skew(1,2)+epsilon_skew_sq(1,2) 2*q0*epsilon_skew(1,3)+epsilon_skew_sq(1,3)];
    %Second row of the rotation matrix
    row_2 = [2*q0*epsilon_skew(2,1)+epsilon_skew_sq(2,1) 1++epsilon_skew_sq(2,2) 2*q0*epsilon_skew(2,3)+epsilon_skew_sq(2,3)];
    %Third row of the rotation matrix
    row_3 = [2*q0*epsilon_skew(3,1)+epsilon_skew_sq(3,1) 2*q0*epsilon_skew(3,2)+epsilon_skew_sq(3,2) epsilon_skew_sq(3,3)+1];
    %3x3 rotation matrix
    rot_matrix = [row_1; row_2; row_3];
end