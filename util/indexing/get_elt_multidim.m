function out = get_elt_multidim(inarray, indvec, cell1mat0)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% GET ELEMENT FROM MULTI-DIMENSIONAL ARRAY
%
% Brent Wallace  
%
% 2023-03-08
%
% This program enables indexing of multi-dimensional arrays. Given a vector
% of indices, this program constructs the appropriate cell array to index
% an array along the entries of the vector. It then indexes the array input
% to the program at these index values.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% inarray           (n-dim Array) Desired array to index.
% indvec            (n-dim Vector) Vector containing indices of array.
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
% out               (Variable Type) Value of array 'inarray' at the indices
%                   specified in 'indvec'.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Check input arguments
switch nargin
    case 2
        cell1mat0 = 1;
    case 3
        % Nothing
    otherwise
        error('MULTI-DIM INDEXING ERROR: MUST HAVE 2-3 ARGUMENTS')
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

% Index the array
if cell1mat0
    out = inarray{indcell{:}};
else
    out = inarray(indcell{:});
end


