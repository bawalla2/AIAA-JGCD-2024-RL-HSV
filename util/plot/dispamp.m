function dispamp(A)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PRINT MATRIX WITH AMPERSANDS BETWEEN ELEMENTS
%
% Brent Wallace  
%
% 2023-03-13
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Get number of rows
nrow = size(A,1);

% Get number of columns
ncol = size(A,2);

% Single tab
onetab = '  ';

% Display each row
for i = 1:nrow

    % Initialize empty string for this row
    currrowstr = [];

    % Construct the current row string
    for j = 1:ncol
        % Current string
        currstr = num2str(A(i,j));
        % Append to current row string
        if j < ncol
            currrowstr = [currrowstr currstr onetab '&' onetab];
        else
            currrowstr = [currrowstr currstr];
        end
    end

    % Display current row string
    disp(currrowstr)

end