% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PROJECTION ONTO SYMMETRIC MATRICES
%
% Brent Wallace  
%
% 2022-12-08
%
% Given a matrix A \in R^{n \times n}, this program returns the orthogonal
% projection \pi(A) \in S^n of A onto the symmetric matrices S^n
% (orthognality is with respect to the Frobenius inner product on 
% R^{n \times n}).
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function piA = projsym(A)

piA = (A + A') / 2;

end






