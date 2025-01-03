function re_plot(plot_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% RE-PLOT PREVIOUSLY GENERATED PRESET GROUP DATA
%
% Brent Wallace  
%
% 2022-03-25
%
% This program, given previously generated preset group settings and output
% data, re-calls the main plot function plot_main.m to generate new plots
% for the old data.
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% re_plot(plot_settings)
%
% NOTE: Before running this program, make sure the MATLAB directory is at
% the folder housing 'main.m'
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% plot_settings         (Struct) Contains necessary plot settings to run
%                       the program. Has the following fields:
%
%   relpath_data        (String) Relative file path to algorithm data.  
%   savefigs            (Boolean) 1 = save figures to PDF. 0 = don't save.
%   relpath             (String) relative file path of folder to save plots
%                       to, if they are to be saved.
%   do_individual_plots (Boolean) 1 = plot each preset's individual plots.
%                       0 = don't plot.
%
% *** NOTE: following pre-existing data will need to be imported from the
% relative directory specified by 'relpath_data':
%
% alg_settings_cell     (Cell, each entry a Struct) Algorithm 
%                       settings/parameters for subsequent execution
%                       according to desired preset (fields vary by
%                       algorithm, see respective algorithm .m-file for
%                       details).
% out_data_cell         (Cell, each entry a Struct) Output data generated
%                       by the algorithm (fields vary by algorithm, see
%                       respective algorithm .m-file for details).
% group_settings        (Struct) contains preset group settings.
%                       Has the following fields:
%   savefigs            (Boolean) 1 = save figures to PDF. 0 = don't save.
%   relpath             (String) relative file path of folder to save plots
%                       to, if they are to be saved.
%   dolegend            (Boolean) 1 = include preset group legend on plots.
%                       0 = don't include legend.
%   sys_plot_settings   (Struct) contains system plot settings. See
%                       config_sys.m for details.
%   preset_group        (String) Tag of the current preset group being
%                       executed.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% 
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
%
% PROGRAM CONFIG
% 
% *************************************************************************


% ***********************
%
% INCLUDE UTILITY FOLDER
%
addpath('util');

% ***********************
%
% INCLUDE CONFIG FUNCTIONS FOLDER
%
addpath('config');

% ***********************
%
% INCLUDE EVALUATION FUNCTIONS FOLDER
%
addpath('eval_functs');

% ***********************
%
% FIGURE CONTROLS
%

% Save figures control
savefigs = plot_settings.savefigs;       

% Save data control
savedata = plot_settings.savedata;

% Relative file path for saved figures
relpath = plot_settings.relpath;  	

% % Do plots for each preset in the group
% do_individual_plots = plot_settings.do_individual_plots;    

% Relative path to data
relpath_data = plot_settings.relpath_data;


% *************************************************************************
%
% EXTRACT PRE-EXISTING DATA
% 
% *************************************************************************

% Load data -- alg_settings_cell struct
data = load([relpath_data 'alg_settings_cell_master.mat']);
alg_settings_cell_master = data.alg_settings_cell_master;

% Load data -- out_data_cell struct
data = load([relpath_data 'out_data_cell_master.mat']);
out_data_cell_master = data.out_data_cell_master;

% Load data -- group_settings struct
data = load([relpath_data 'group_settings_master.mat']);
group_settings_master = data.group_settings_master;

% Load data -- master_settings struct
data = load([relpath_data 'master_settings.mat']);
master_settings = data.master_settings;

% *************************************************************************
%
% MODIFY NECESSARY SETTINGS TO GENERATE/SAVE NEW PLOTS
% 
% *************************************************************************

% Is a re-plot
master_settings.isreplot = 1;

% Update the 'savefigs' entry of the master_settings struct
master_settings.savefigs = savefigs;

% Update the 'savedata' entry of the master_settings struct
master_settings.savedata = savedata;

% Update the 'relpath' entry of the master_settings struct
master_settings.relpath = relpath;

% Update plot formatting to latest
master_settings.psett_master = init_psett();


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% CALL plot_main.m
% 
% *************************************************************************

plot_main(alg_settings_cell_master, out_data_cell_master,...
    group_settings_master, master_settings)

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Display complete
disp('*******************************************************************')
disp('*******************************************************************')
disp('*')
disp(['* RE-PLOTTING DATA COMPLETE'])
disp('*')
disp('*******************************************************************')
disp('*******************************************************************')
