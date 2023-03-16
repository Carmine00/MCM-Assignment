function [iTj]=GetFrameWrtFrame(linkNumber_i, linkNumber_j,biTei)
%%% GetFrameWrtFrame function 
% inputs : 
% linkNumber_i : number of ith link 
% linkNumber_j: number of jth link 
% biTei vector of matrices containing the transformation matrices from link i to link i +1 for the current q.
% The size of biTei is equal to (4,4,numberOfLinks)
% outputs:
% iTj : transformation Matrix in between link i and link j for the
% configuration described in biTei
iTj = eye(4,4);

for i = linkNumber_i:linkNumber_j
    iTj = iTj*biTei(:,:,i);
end



end