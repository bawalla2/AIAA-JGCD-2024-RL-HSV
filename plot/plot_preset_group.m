function [alg_settings_cell, out_data_cell, group_settings]  = ...
    plot_preset_group(...
    alg_settings_cell, out_data_cell, group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% MAIN PLOT FUNCTION -- PRESET GROUP
%
% Brent Wallace  
%
% 2021-11-06
%
% This program, given a specified preset group, generates all plots for the
% group and saves the plots.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZATION
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Unpack plot settings
savefigs = master_settings.savefigs;
savedata = master_settings.savedata;
relpath = group_settings.relpath;
dolegend = group_settings.dolegend;
do_individual_plots = group_settings.do_individual_plots;
sys_plot_settings = master_settings.sys_plot_settings;
relpath_dirl_sweep = master_settings.relpath_dirl_sweep;

% Plot frequency responses (=1) or not (=0)
isfreqresp = group_settings.isfreqresp;

% Is a re-plot (=1) or not (=0)
isreplot = master_settings.isreplot;

% Number of designs to plot for
numpresets = size(alg_settings_cell,1);


% Is a sweep (=1) or not (=0)
issweep = group_settings.issweep;
issweep_IC = group_settings.issweep_IC;
issweep_nu = group_settings.issweep_nu;
issweep_rand_nu = group_settings.issweep_rand_nu;
issweep_statdyn = group_settings.issweep_statdyn;

% Run algorithms (=1) or not (=0)
runpresets = group_settings.runpresets;

% Is cFVI comparison (=1) or not (=0)
iseval = group_settings.iseval;

% Initialize figure counter
figcount = group_settings.figcount;


% Create save directory if figures are to be saved
if savefigs
    mkdir(relpath);                   % Create directory for relative path
end

% Update relpath field of group_settings struct to include the time stamp
% (if it was added, otherwise this line is redundant)
group_settings.relpath = relpath;

% Tag of system being executed
systag = master_settings.systag;
% List of system names
sysnames = master_settings.sysnames;


% Check if is a training group
istraining = group_settings.istraining;



% ***********************
%
% MAKE LEGEND IF USER SPECIFIED
%  

if dolegend
   
    % Initialize legend
    lgnd = cell(numpresets,1);
    
    % Fill legend entries
    for i = 1:numpresets
       
        % Extract preset legend entry
        lgnd{i} = alg_settings_cell{i}.plot_settings.legend_entry;
        
    end
    
    % Store legend
    group_settings.lgnd = lgnd;
    
end



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


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STATE TRAJECTORY, CONTROL SIGNAL PLOTS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Store figure counter in plot settings to pass to function
group_settings.figcount = figcount;

if ~issweep && ~isfreqresp

    figcount = plot_x_u(alg_settings_cell,...
        out_data_cell, group_settings, master_settings);

end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALCULATE TRACKING METRICS, DO TRACKING PLOTS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Plot tracking responses (=1) or not (=0)
% doplottracking = 0;
% doplottracking = 1;
doplottracking = ~issweep_rand_nu;

% if 0
% if 1
if ~istraining && runpresets

    % Calculate tracking metrics
    if ~isreplot
    [alg_settings_cell, out_data_cell] = calc_tracking_metrics...
       (alg_settings_cell, out_data_cell, group_settings, master_settings);
    end
    
    % Store figure counter in plot settings to pass to function
    group_settings.figcount = figcount;
    
    % Do tracking plots
    if doplottracking && ~isreplot
    figcount = plot_tracking...
       (alg_settings_cell, out_data_cell, group_settings, master_settings); 
    end

    % If is a random \nu sweep, tabulate data
    if issweep_rand_nu
    % Store figure counter in plot settings to pass to function
    group_settings.figcount = figcount;       
    [out_data_cell, figcount] = tabulate_tracking_metrics_rand_nu...
       (alg_settings_cell, out_data_cell, group_settings, master_settings);
    end

end



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALCULATE DYNAMIC FUNCTIONS AND PERFORMANCE METRICS IF IS OF THE WANG,
% STENGEL (2000) FAMILY
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Check if is part of the Wang, Stengel (2000) HSV family
is_hsv_wang_stengel = strcmp(systag,sysnames.hsv);
% dohsvcalcs = is_hsv_wang_stengel && ~istraining ...
%     && ~issweep && ~isfreqresp && doplottracking;

dohsvcalcs = 0;

% Calculate HSV metrics if is HSV
if dohsvcalcs
    out_data_cell = hsv_wang_stengel_2000_calc...
    (alg_settings_cell, out_data_cell, group_settings); 
end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% ADDITIONAL PLOTS IF IS A SWEEP
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Store figure counter in plot settings to pass to function
group_settings.figcount = figcount;

if (issweep_IC || issweep_nu || issweep_rand_nu) && istraining
% if 0
    figcount = ...
        plot_sweep(alg_settings_cell, out_data_cell,...
        group_settings, master_settings);

end

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% ADDITIONAL PLOTS IF IS A STATIC/DYNAMIC SWEEP
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Store figure counter in plot settings to pass to function
group_settings.figcount = figcount;

if issweep_statdyn
% if 0
    figcount = ...
        plot_stat_dyn_sweep(alg_settings_cell, out_data_cell,...
        group_settings, master_settings);

end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOT -- CLOSED-LOOP FREQUNECY RESPONSES
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Update figure counter in master settings
master_settings.figcount = figcount;

% Call function
if isfreqresp
    if issweep
    [figcount, out_data_freq_resp] = ...
        plot_freq_resp_hsv_sweep(alg_settings_cell, ...
        out_data_cell,...
        group_settings, master_settings);
    out_data_cell{1}.freq_resp = out_data_freq_resp;
    end
end



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SAVE PRESET GROUP DATA TO DIRECTORY
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Store figure counter in plot settings
group_settings.figcount = figcount;

data_folder = 'data/';

if savedata
    
    % Make directory to save data to
    relpath_data = [relpath data_folder];
    group_settings.relpath_data = relpath_data;
    mkdir(relpath_data)
    
    % Save data -- alg_settings_cell struct
    varname = 'alg_settings_cell';
    save([relpath_data varname], varname);
    
    % Save data -- out_data_cell struct
    varname = 'out_data_cell';
    save([relpath_data varname], varname);
    
    % Save data -- group_settings struct
    varname = 'group_settings';
    save([relpath_data varname], varname);

end

