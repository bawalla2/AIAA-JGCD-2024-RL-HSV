function [master_settings, group_settings_master, preset_list_cell] = ...
    config_settings(master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PROGRAM SETTINGS
%
% Brent Wallace  
%
% 2023-03-29
%
% This program performs the master config of this entire MATLAB code suite.
% The initialization process includes:
%
%   * Configure relative paths to programs, data
%   * Configure algorithm settings
%   * Configure master plot formatting
%   * Configure frequency response plot settings
%   * System initialization
%       * Modeling error parameter values \nu
%       * Integral augmentation settings
%       * Configure misc system settings -- config_sys.m
%   * Configure relative paths to data for selected system
%   * Configure controllers for selected system
%   * Configure individual preset group settings 
%       -- config_preset_group_cell.m  
%       -- config_preset_group.m 
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
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
%
% UNPACK USER-SET SETTINGS
% 
% *************************************************************************
% *************************************************************************


% ***********************
%
% SETTINGS -- SYSTEM SELECTION
%

% System names
sysnames.hsv = 'hsv';
master_settings.sysnames = sysnames;        % Save list to master settings

% System tag
systag = sysnames.hsv;

% Preset group
preset_group = 'main';

% Preset group list
preset_group_list = master_settings.preset_group_list;

% Number of preset groups executed
numgroups = master_settings.numgroups;




%%
% *************************************************************************
% *************************************************************************
%
% INCLUDE FILE PATHS
% 
% *************************************************************************
% *************************************************************************


% ***********************
%
% INCLUDE UTILITY FOLDER
%
addpath('util');
addpath('util/plot');
addpath('util/CLTFM');
addpath('util/DIRL');
addpath('util/tracking metrics/');
addpath('util/indexing/');

% ***********************
%
% INCLUDE AIRCRAFT UTILITY FOLDERS
%
relpath_data_main = '01 data/';
addpath('02 aircraft/util');
addpath('02 aircraft/hsv_wang_stengel_2000');
addpath('02 aircraft/hsv_wang_stengel_2000/ndi');

% ***********************
%
% INCLUDE SYSTEM UTILITY FOLDERS
%
addpath([relpath_data_main 'hsv/']);

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
% INCLUDE ALGORITHM FOLDER
%
addpath('algs');

% ***********************
%
% INCLUDE PLOT FUNCTIONS FOLDER
%
addpath('plot');

% ***********************
%
% RELATIVE PATHS TO PROGRAM DATA
%

relpath_data_root = '01 data/';

%%
% *************************************************************************
% *************************************************************************
%
% ALGORITHM INIT
% 
% *************************************************************************
% *************************************************************************

% Algorithm names
algnames.mi_dirl = 'dEIRL';
algnames.lq_opt_nu = 'Opt';
algnames.lq_K0 = 'Nom';
algnames.fbl = 'FBL';
master_settings.algnames = algnames;


%%
% *************************************************************************
% *************************************************************************
%
% PLOT FORMATTING
% 
% *************************************************************************
% *************************************************************************


% Master plot settings
psett_master = init_psett();

% SET MASTER PLOT SETTINGS
master_settings.psett_master = psett_master;


% ***********************
%
% FREQUENCY RESPONSE PLOT SETTINGS
%  

% Frequency points for SV plots
wmin = -3;
wmax = 2;
nwpts = 500;
wvec = logspace(wmin,wmax,nwpts);
master_settings.wvec = wvec;

%%
% *************************************************************************
% *************************************************************************
%
% SYSTEM/CONTROLLER INIT
% 
% *************************************************************************
% *************************************************************************

% ***********************
%
% SETTINGS -- SYSTEM/CONTROLLER INIT
%
% Initialize (=1) or load from previous (=0)
%

% Model
% init1_load0_model = 1;
init1_load0_model = 0;

% LQ servo controllers
% init1_load0_lq = 1; 
% init1_load0_lq = 0;
init1_load0_lq = init1_load0_model;

% Feedback linearization controllers (HSV only)
% init1_load0_fbl = 1; 
init1_load0_fbl = 0; 
% init1_load0_fbl = init1_load0_model; 

% Model/controller init
master_settings.init1_load0_model = init1_load0_model;
master_settings.init1_load0_lq = init1_load0_lq;
master_settings.init1_load0_fbl = init1_load0_fbl;

% ***********************
%
% SETTINGS -- DIRL DATA LOCATION
%

% Pull DIRL data from sweep data location (=1) or temp data location (=0)
dirldatasweep1temp0 = 1;
% dirldatasweep1temp0 = 0;

% Pull DIRL data from sweep data location (=1) or temp data location (=0)
master_settings.dirldatasweep1temp0 = dirldatasweep1temp0;

% ***********************
%
% RELATIVE PATH TO CURRENT MODEL
%

% Root path
relpath_model = [relpath_data_root systag '/'];

% Path to model data
relpath_model_data = [relpath_model 'model/'];

% Filename of model data
filename_model = 'model.mat';

% ***********************
%
% SETTINGS -- MODELING ERROR PARAMETERS \nu USED FOR EVALUATIONS
%

% Tag of which model cell array to use
modelcelltag = 'main';

% ***********************
%       
% DETERMINE WHICH MODELING ERROR PARAMETER TO USE
%  

% Group name
groupnamei = preset_group_list{1};   

% Determine if is a rand \nu sweep
issweep_rand_nu = contains(groupnamei,'rand_nu');

% Determine modeling error type
if issweep_rand_nu
    nutag = 'rand';
else
    if contains(groupnamei,'CL_CD')
        nutag = 'CL_CD';
    elseif contains(groupnamei,'CL_CMa')
        nutag = 'CL_CMa';
    elseif contains(groupnamei,'CD_CMa')
        nutag = 'CD_CMa';            
    else
        if contains(groupnamei,'CL')
            nutag = 'CL';
        elseif contains(groupnamei,'CD')
            nutag = 'CD';
        else 
            nutag = 'CMa';
        end
    end 
end

% Perturbation params
switch modelcelltag   
    case 'main'        
        switch nutag
            case 'CL'
                nuvec = (1:-0.025:0.75)';
            case {'CD'; 'CMa'}
                nuvec = (1:0.025:1.25)';
            case {'CL_CD'; 'CL_CMa'}
                nu1vec = (1:-0.025:0.75)';
                nu2vec = (1:0.025:1.25)';
            case 'CD_CMa'
                nu1vec = (1:0.025:1.25)';
                nu2vec = (1:0.025:1.25)';
            case 'rand'
                numnu = 10000;
                nu1vec = ones(numnu,1);
                nu1vec(end) = 2;
                nu2vec = 1;
        end
    case {'test'; 'testtmp'}        
        switch nutag
            case 'CL'
                nuvec = [1; 0.9; 0.75];
            case {'CD'; 'CMa'}
                nuvec = [1; 1.1; 1.25];
            case {'CL_CD'; 'CL_CMa'}
                nu1vec = [1; 0.75];
                nu2vec = [1; 1.25];
            case 'CD_CMa'
                nu1vec = [1; 1.25];
                nu2vec = [1; 1.25];
%                         nu1vec = (0.9:0.05:1.1)';
%                         nu2vec = (0.9:0.05:1.1)';
            case 'rand'
                numnu = 100;
                nu1vec = ones(numnu,1);
                nu1vec(end) = 2;
                nu2vec = 1;
        end
end

switch nutag
    case {'CL'; 'CD'; 'CMa'}
        nu1vec = nuvec;
        nu2vec = [nan];
    otherwise
        nuvec = nu1vec;
end

% Which perturbations to plot data for
switch nutag
    case {'CL'; 'CL_CD'; 'CL_CMa'}
        nuvecplot = [1; 0.9; 0.75];
    otherwise
        nuvecplot = [1; 1.1; 1.25];
end


% Which single perturbation value to use for modeling error
% evaluation studies
if contains(groupnamei,'nom')
    nueval = 1;
else
    switch nutag
        case 'CL'
            if contains(groupnamei,'10pct')
                nueval = 0.9;
            else
                nueval = 0.75;
            end
        otherwise
            if contains(groupnamei,'10pct')
                nueval = 1.1;
            else
                nueval = 1.25;
            end
    end
end

% Tick spacing for \nu ablations
nu_tick_space = 0.05;

% Name of model initialization program
model_init_program = 'hsv_wang_stengel_2000_init';

% System name (for displaying on plots)
systag_disp = 'HSV';

% ***********************
%
% MODEL INDICES -- NOMINAL MODEL
%

% Index of nominal model
indnom = find(nuvec == 1);

% Indices of non-nominal models
indsnotnom = setdiff((1:size(nuvec,1)),indnom);

% ***********************
%
% MODEL INDICES -- MODELS TO PLOT DATA FOR
%        

indsmodelplot = [];

for mcnt = 1:size(nuvecplot,1)          
    ind = find(nuvec == nuvecplot(mcnt));
    if ~isempty(ind)
        indsmodelplot = [indsmodelplot; ind];
    end
end

nummodelsplot = size(indsmodelplot,1);

% Overwrite this settings if is a \nu sweep

switch nutag
    case {'CL_CD'; 'CL_CMa'; 'CD_CMa'}
        indnom1 = find(nu1vec == 1);
        indnom2 = find(nu2vec == 1);
        indnom = [indnom1; indnom2];
        indsmodelplot = 1;
        nummodelsplot = 1;
    case {'rand'}
        indnom = 1;
        indsmodelplot = 1;
        nummodelsplot = 1;
end


% Add extension to relative path
relpath_model_data = [relpath_model_data nutag '/' modelcelltag '/'];


% ***********************
%
% MODEL INDICES -- MODEL TO PERFORM MODELING ERROR EVALUATIONS FOR
%  

if length(nueval) == 1
    indnueval = find(nuvec == nueval);
    nueval_str = num2filename(nueval);
else
    indnu1eval = find(nu1vec == nueval(1));
    indnu2eval = find(nu2vec == nueval(2));
    indnueval = [indnu1eval; indnu2eval];
    nueval_str = ...
        num2filename([num2str(nueval(1)) '_' num2str(nueval(2))]);
end


% ***********************
%       
% CONTROLLER HAS INTEGRAL AUGMENTATION (=1) OR NOT (=0)
% 

hasintaug = 1; 

% Set
master_settings.hasintaug = hasintaug;

% ***********************
%
% SET TICK VALUES FOR \nu ABLATION 3D PLOTS
%  

nu_ticks_cell = cell(2,1);
nuvec_cell = cell(2,1);
nuvec_cell{1} = nu1vec;
nuvec_cell{2} = nu2vec;

for i = 1:2
    nuivec = nuvec_cell{i};
    if nuivec(1) < nuivec(end)
        nuiticks = (nuivec(1):nu_tick_space:nuivec(end))';        
    else
        nuiticks = (nuivec(end):nu_tick_space:nuivec(1))';        
    end
    % Clip greatest element for display
    nuiticks = nuiticks(nuiticks < max(nuivec)); 
    % Store
    nu_ticks_cell{i} = nuiticks;
end



    

% *************************************************************************
%
% INITIALIZE/LOAD MODEL
%
% *************************************************************************

% ***********************
%
% MISC SETTINGS
%

% Initialize model if desired
if init1_load0_model


    % Set tag
    setts.nutag = nutag;          
  
    % Set perturbation parameters
    switch nutag
        case {'CL'; 'CD'; 'CMa'; }
            nu1vec = nuvec;
            nu2vec = 1;
            setts.nu1vec = nuvec;
            setts.nu2vec = 1;
        otherwise  

    end

    % Set perturbation vectors
    setts.nu1vec = nu1vec;
    setts.nu2vec = nu2vec;      

    % Settings
    setts.indnom = indnom;
    setts.relpath_data = relpath_model_data;
    setts.filename = filename_model;   
    % Init
    eval([model_init_program '(setts)'])
    clear setts;

end

% Load model parameters
disp('***** LOADING MODEL DATA (PLEASE WAIT) *****')
models = load([relpath_model_data filename_model]);
models = models.model_struct;
model_cell = models.model_cell; 

% Get dimensions of system cell array
numnu1 = size(model_cell,1);
numnu2 = size(model_cell,2);
nummodels = numnu1*numnu2;

% Configure system settings
sys.tag = systag;
sys.tag_disp = systag_disp;
sys.relpath_data = relpath_model_data;
sys.nutag = nutag;

% Store model cell arrays, perturbation params
sys.model_cell = model_cell;   
sys.nummodels = size(model_cell,1);
sys.indnom = indnom;
sys.indsnotnom = indsnotnom;
sys.nuvec = nuvec;
sys.nu1vec = nu1vec;
sys.nu2vec = nu2vec;
sys.nuvecplot = nuvecplot;

% Initialize system
[sys, sys_plot_settings] = config_sys(sys);

% Store system parameters
master_settings.sys = sys;
master_settings.systag_disp = systag_disp;

% Store settings in system plot settings
sys_plot_settings.nu_ticks_cell = nu_ticks_cell;

% Store plot settings
master_settings.sys_plot_settings = sys_plot_settings;

% Save dimensions of system cell array
master_settings.numnu1 = numnu1;
master_settings.numnu2 = numnu2;
master_settings.nummodels = nummodels;

% Save nominal model 


%%
% *************************************************************************
% *************************************************************************
%
% RELATIVE PATHS TO DATA FOR THIS MODEL
% 
% *************************************************************************
% *************************************************************************


% ***********************
%
% RELATIVE PATHS TO DIRL DATA
%

% Root path
relpath_dirl = [relpath_model 'DIRL/'];

% Relative path to nominal model
relpath_dirl_nom = [relpath_dirl 'nu_1/'];

% Path to this value of \nu for evaluation
relpath_dirl_error = [relpath_dirl 'nu_' nueval_str '/'];

% File name of DIRL data
if dirldatasweep1temp0
    filename_data_dirl = 'out_data_cell_master';
else
    filename_data_dirl = 'dirl_data';
end

% ***********************
%
% RELATIVE PATHS TO DIRL DATA -- x_0 SWEEP TRAINING DATA
%

relpath_dirl_sweep = [relpath_model 'dirl_sweep/'];
relpath_dirl_sweep = [relpath_dirl_sweep nutag '/' modelcelltag '/'];
relpath_dirl_sweep_eval = [relpath_dirl_sweep 'eval/'];

% ***********************
%
% RELATIVE PATHS TO EVALUATION DATA
%

relpath_data_eval = [relpath_model 'eval_data/'];


% ***********************
%
% RELATIVE PATHS TO LQ SERVO, FBL DATA
%

relpath_lq = [relpath_model 'lq_servo/' nutag '/' modelcelltag '/'];
relpath_fbl = [relpath_model 'fbl/'];

% Store relative paths to LQ servo, FBL data in master settings
master_settings.relpath_lq = relpath_lq;
master_settings.relpath_fbl = relpath_fbl;


%%
% *************************************************************************
% *************************************************************************
%
% INITIALIZE/LOAD CONTROLLERS FOR THIS MODEL
% 
% *************************************************************************
% *************************************************************************

% Initalize/load controllers
master_settings = eval(['config_controllers_' systag '(master_settings)']);

%%
% *************************************************************************
% *************************************************************************
%
% SWEEP INIT
% 
% *************************************************************************
% *************************************************************************

% Array of sweep type names
sweeptypenames.IC = 'IC';
sweeptypenames.nu = 'nu';
sweeptypenames.rand_nu = 'rand_nu';
sweeptypenames.plot = 'plot';

%%
% *************************************************************************
% *************************************************************************
%
% SAVE SETTINGS
%
% *************************************************************************
% *************************************************************************

% System indices
master_settings.systag = systag;
master_settings.indsmodelplot = indsmodelplot;
master_settings.nummodelsplot = nummodelsplot;
master_settings.indnueval = indnueval;

% Store relative paths to DIRL data
master_settings.relpath_dirl = relpath_dirl;
master_settings.relpath_dirl_nom = relpath_dirl_nom;
master_settings.relpath_dirl_error = relpath_dirl_error;
master_settings.filename_data_dirl = filename_data_dirl;

% Store relative paths to DIRL x_0 sweep data in master settings
master_settings.relpath_dirl_sweep = relpath_dirl_sweep;
master_settings.relpath_dirl_sweep_eval = relpath_dirl_sweep_eval;

% Store relative path and filename to evaluation data in master settings
master_settings.relpath_data_eval = relpath_data_eval;

% Array of sweep type names
master_settings.sweeptypenames = sweeptypenames;

% Is a re-plot
master_settings.isreplot = 0;


%%
% *************************************************************************
% *************************************************************************
%
% METHOD/SYSTEM/DESIGN PRESET LIST AND SETTINGS
%
% This is a list of tags correspond to the presets to execute for the
% selected preset group. Each preset consists of settings which include the
% specific
%
%   Algorithm/methodology (e.g., dEIRL, FBL etc.)
%   System (e.g., HSV system)
%   Design (with numerical design parameters)
%
% Below is the selection of the preset list, along with initialization of
% any preset group settings which are shared among all the designs in the
% group (e.g., if all designs share the same system, or the same Q, R
% matrices, etc.)
%
% *************************************************************************
% *************************************************************************


% Preset group cell
group_settings_master = ...
    config_preset_group_cell(master_settings);

% Each entry contains the presets executed for the respective group
preset_list_cell = cell(numgroups, 1);

% Initialize 
for i = 1:numgroups

    preset_groupi.group_settings = group_settings_master{i};
    preset_groupi.tag = preset_group;

    [preset_list_cell{i}, group_settings_master{i}] = ...
                config_preset_group(preset_groupi, master_settings);

end

% Get total number of presets executed -- including sweeps
numpresets_tot = 0;
for i = 1:numgroups

    % Group settings
    group_settingsi = group_settings_master{i};

    % Number of presets in this group
    numpresetsi = group_settingsi.numpresets;

    % Extract sweep settings
    sweepsettsi = group_settingsi.sweepsetts;

    % Is a sweep preset group (=1) or not (=0)
    issweep = sweepsettsi.issweep;

    % Total number of sweep iterations
    if issweep
        numsweepits = prod(sweepsettsi.sweepsizevec);
    else
        numsweepits = 1;
    end

    % Increment total
    numpresets_tot = numpresets_tot + numsweepits * numpresetsi;

end

% Store
master_settings.numpresets_tot = numpresets_tot;
