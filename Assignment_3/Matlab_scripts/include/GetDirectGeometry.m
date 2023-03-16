function [iTj_q] = GetDirectGeometry(q, iTj, linkType)
%%% GetDirectGeometryFunction

% Inputs: 
% q : links current position ; 
% iTj : vector of matrices containing the transformation matrices from link
% i to link j
% linkType: vector of size numberOfLinks identiying the joint type, 0 for revolute, 1 for
% prismatic.

% Outputs :
% iTj_q vector of matrices containing the transformation matrices from link i to link j for the input q. 
% The size of iTj is equal to (4,4,numberOfLinks)

numberOfLinks = length(linkType);



for i = 1:numberOfLinks
    iTj_q(:,:,i) = DirectGeometry(q(i),iTj(:,:,i), linkType(i));
end



end