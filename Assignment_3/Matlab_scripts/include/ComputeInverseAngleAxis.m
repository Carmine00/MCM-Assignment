function [theta,v] = ComputeInverseAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrix this function
%outputs the equivalent angle-axis representation values, 
% respectively 'theta' (angle), 'v' (axis) 
    
    tol = 1e-4; % Set a tolerance to make comparison between values
    [m,n] = size(R);
    if m ==3 && n ==3 % Check the dimension of the input matrix
        if abs(R*R' - eye(3)) < tol % Check if the input matrix is orthogonal --> R*R' = I --> R*R' -I = 0 but this
            % is compared to a tolerance
            if abs(det(R) -1) < tol % Check whether det(R) == 1
                theta = acos((trace(R)-1)/2);
                [V, D] = eig(R);
                % The position of the eigenvalue equal to 1 is the column
                % of the eigenvector matrix V corresponding to the vector
                % frame v (see report for more theoretical details)
                k = abs(diag(D) - 1) < tol;
                v = V(:,k);
            else % Display different error messages in case the condition above do not hold
                error('DETERMINANT OF THE INPUT MATRIX IS 0');
            end
        else
             error('NOT ORTHOGONAL INPUT MATRIX');
        end
    else
        error('WRONG SIZE OF THE INPUT MATRIX');
    end
end

