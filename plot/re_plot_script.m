% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% RE-PLOT PREVIOUSLY GENERATED PRESET GROUP DATA
%
% Brent Wallace  
%
% 2022-12-12
%
% This program, given previously generated preset group settings and output
% data, re-calls the main plot function plot_main.m to generate new plots
% for the old data. See re_plot.m for further documentation.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZATION
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
% *************************************************************************
%
% PROGRAM CONFIG
% 
% *************************************************************************
% *************************************************************************


% ***********************
%
% FIGURE CONTROLS
%

close all

% Save figures control
% savefigs = 1; 
savefigs = 0; 

% Save data control
% savedata = 1;
savedata = 0;

% Relative file path for saved figures
relpath = '00 figures/'; 	

% Tag for group of plots to execute
% group_tag = 'ES_error_dirl_training_sweep_IC_CL';
% group_tag = 'ES_error_dirl_training_sweep_IC_CD';
% group_tag = 'ES_error_dirl_training_sweep_IC_CMa';
% group_tag = 'ES_error_dirl_training_sweep_nu_CL_CD';
% group_tag = 'ES_error_dirl_training_sweep_nu_CL_CMa';
% group_tag = 'ES_error_dirl_training_sweep_nu_CD_CMa';
% group_tag = 'ES_error_dirl_training_sweep_rand_nu';

% *************************************************************************
% *************************************************************************
%
% INITIALIZE PROGRAM BASED ON DESIRED PLOT DATA
% 
% *************************************************************************
% *************************************************************************

switch group_tag

    % ***********************
    %
    % SWEEP -- HSV IC -- C_L
    %    
    case 'ES_error_dirl_training_sweep_IC_CL'

        % Relative path to data
        relpath_data = '01 data/hsv/dirl_sweep/CL/main/';

    % ***********************
    %
    % SWEEP -- HSV IC -- C_D
    %    
    case 'ES_error_dirl_training_sweep_IC_CD'

        % Relative path to data
        relpath_data = '01 data/hsv/dirl_sweep/CD/main/';

    % ***********************
    %
    % SWEEP -- HSV IC -- C_Ma
    %    
    case 'ES_error_dirl_training_sweep_IC_CMa'

        % Relative path to data
        relpath_data = '01 data/hsv/dirl_sweep/CMa/main/';

    % ***********************
    %
    % SWEEP -- HSV \nu -- C_L / C_D
    %    
    case 'ES_error_dirl_training_sweep_nu_CL_CD'

        % Relative path to data
        relpath_data = '01 data/hsv/dirl_sweep/CL_CD/main/';

    % ***********************
    %
    % SWEEP -- HSV \nu -- C_L / C_Ma
    %    
    case 'ES_error_dirl_training_sweep_nu_CL_CMa'

        % Relative path to data
        relpath_data = '01 data/hsv/dirl_sweep/CL_CMa/main/';

    % ***********************
    %
    % SWEEP -- HSV \nu -- C_D / C_Ma
    %    
    case 'ES_error_dirl_training_sweep_nu_CD_CMa'

        % Relative path to data
        relpath_data = '01 data/hsv/dirl_sweep/CD_CMa/main/';        


    % ***********************
    %
    % SWEEP -- rand \nu -- TRAINING
    %    
    case 'ES_error_dirl_training_sweep_rand_nu'

        % Relative path to data
        relpath_data = '01 data/hsv/dirl_sweep/rand/main/';

end

% *************************************************************************
% *************************************************************************
%
% RE-PLOT THE DATA
% 
% *************************************************************************
% *************************************************************************

% Pack input arguments
plot_settings.savefigs = savefigs;    
plot_settings.savedata = savedata;
plot_settings.relpath = relpath; 	  
plot_settings.relpath_data = relpath_data;

% Call function
re_plot(plot_settings);

