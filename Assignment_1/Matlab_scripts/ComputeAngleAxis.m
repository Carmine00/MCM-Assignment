function R = ComputeAngleAxis(theta,v)
% skew matrix written component by component
v_skew =[0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0 ]; 
I = eye(3);
% Computation of the Rodrigues's formula
R = I + v_skew*sin(theta) + (1-cos(theta))*v_skew*v_skew;
end