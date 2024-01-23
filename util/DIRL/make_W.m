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
%       B(x, y) = W kron(x, y)
%
% and the right inverse M_{r}^{-1} of M such that
%
%       kron(x, x) = M_{r}^{-1} B(x, x)
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


function [W, Wr]= make_W(n)

% Binomial coefficient
nb = n*(n+1)/2;

% Declare M, M_{r}^{-1} 
W = zeros(nb,n^2);

% Fill M, M_{r}^{-1} 
cnt = 1;
for i = 1:n
    for j = i:n
        indkron_12 = (i-1)*n + j;
        indkron_21 = (j-1)*n + i;
        % Fill M
        if indkron_12 == indkron_21
            W(cnt,indkron_12) = 1;
        else
            W(cnt,indkron_12) = sqrt(2)/2;
            W(cnt,indkron_21) = sqrt(2)/2;
        end
        cnt = cnt + 1;
    end
end

Wr = W';

