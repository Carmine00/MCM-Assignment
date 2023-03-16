function [r]=GetBasicVectorWrtBase(biTei, linkNumber)
%%% GetBasicVectorWrtBase function 
% input :
% biTei vector of matrices containing the transformation matrices from link i to link i +1 for the current q.
% The size of biTei is equal to (4,4,numberOfLinks)
% output
% r : basic vector from i to j

bTi = GetTransformationWrtBase(biTei,linkNumber);

r = bTi(1:3,4);



end