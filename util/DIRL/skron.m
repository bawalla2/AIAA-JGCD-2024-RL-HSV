% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% MAKE KRONECKER TO BILINEAR FORM CONVERSION MATRIX\
%
% Brent Wallace  
%
% 2022-12-08
%
% This program makes the matrix M \in R^{n(n+1)/2 x n^2) such that
%
%       B(x, y) = M kron(x, y)
%
% and the right inverse M_{r}^{-1} of M such that
%
%       kron(x, x) = M_{r}^{-1} B(x, x)
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


function AsB= skron(A, B)

% Get dimensions
m = size(A,1);
n = size(A,2);

% Make W_m, W_n
Wm = make_W(m);
Wn = make_W(n);

% Calculate product
AsB = Wm * kron(A,B) * Wn';



