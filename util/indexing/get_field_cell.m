function outarray = get_field_cell(inarray, field, mat1cell0)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% GET FIELD FROM EACH ELEMENT OF A CELL ARRAY
%
% Brent Wallace  
%
% 2023-03-08
%
% This program enables setting of multi-dimensional array elements. Given a
% vector of indices, this program constructs the appropriate cell array to
% index an array along the entries of the vector. It then sets the input
% array at the desired index
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% inarray           (n-dim Array) Desired array to get field from.
% field             (String) Contains the field to index at each element of
%                   the array.
% mat1cell0         (Bool, OPTIONAL, Default = 1) Make output a matrix (=1) 
%                   or cell array (=0)        
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% outarray          (n-dim Array) Array with each element the value at the
%                   specified field of the respective element in the input 
%                   array
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Check input arguments
switch nargin
    case 2
        mat1cell0 = 1;
    case 3
        % Nothing
    otherwise
        error('ERROR: MUST HAVE 2-3 ARGUMENTS')
end

% Get dimensions of input array
n1 = size(inarray,1);
n2 = size(inarray,2);

% Initialize the output array
if mat1cell0
    outarray = zeros(n1,n2);
else
    outarray = cell(n1,n2);
end

% Set each element of the output array
for i = 1:n1
    for j = 1:n2
        currelt = inarray{i,j};
        currfield = currelt.(field);
        if mat1cell0
            outarray(i,j) = currfield;
        else
            outarray{i,j} = currfield;
        end
    end
end


