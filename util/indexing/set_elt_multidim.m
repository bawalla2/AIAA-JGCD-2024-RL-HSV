function outarray = set_elt_multidim(inarray, indvec, inelt, cell1mat0)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SET ELEMENT OF MULTI-DIMENSIONAL ARRAY
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
% inarray           (n-dim Array) Desired array to set.
% indvec            (n-dim Vector) Vector containing indices of array.
% inelt             (Variable Type) Element to set at the specified index
%                   of the input array
% cell1mat0         (Bool, OPTIONAL, Default = 1) Input array should be
%                   indexed as a cell array via '{}' (=1) or as a matrix
%                   with '()' (=0)
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% outarray          (n-dim Array) Array with element set
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Check input arguments
switch nargin
    case 3
        cell1mat0 = 1;
    case 4
        % Nothing
    otherwise
        error('MULTI-DIM INDEXING ERROR: MUST HAVE 3-4 ARGUMENTS')
end

% Make sure index vector is a column vector
indvec = indvec(:);

% Dimension of array
n = size(indvec,1);

% Initialize index cell
indcell = cell(n,1);

% Fill output cell
for i = 1:n
    indcell{i} = indvec(i);
end

% Initialize the output array
outarray = inarray;

% Set the new element of the output array
if cell1mat0
    outarray{indcell{:}} = inelt;
else
    outarray(indcell{:}) = inelt;
end


