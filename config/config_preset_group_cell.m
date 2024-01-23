function group_settings_master = config_preset_group_cell(master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PRESET GROUP
%
% Brent Wallace  
%
% 2022-02-16
%
% This program, given a list of desired preset groups to execute,
% initializes each of the preset groups. Most of the execution in this
% program is configuring automatic settings. However, to change which
% algorithms are executed for a given system, see the variable
% 'alg_list_default'.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% master_settings       (Struct) Master settings as initialized by main.m
%                       and config_settings.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% group_settings_master     ('numgroups' x 1 Cell) The i-th entry of this   
%                           cell array is itself a struct containing all of
%                           the settings for the corresponding preset
%                           group.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% UNPACK SETTINGS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Save figures
savefigs = master_settings.savefigs;

% Save data
savedata = master_settings.savedata;

% Relative path
relpath = master_settings.relpath;

% Number of preset groups executed
numgroups = master_settings.numgroups;

% Preset group list
preset_group_list = master_settings.preset_group_list; 

% Algorithm names
algnames = master_settings.algnames;

% System names
sysnames = master_settings.sysnames;

% Current system being executed
systag = master_settings.systag;

% Array of sweep type names
sweeptypenames = master_settings.sweeptypenames;

% Master plot settings
psett_master = master_settings.psett_master;

% Pull DIRL data from sweep data location (=1) or temp data location (=0)
dirldatasweep1temp0 = master_settings.dirldatasweep1temp0;

% % Which model to reference 
% modelcelltag = master_settings.modelcelltag;

% ***********************
%
% GET SYSTEM DATA
%   

% System
sys = master_settings.sys;
model_cell = sys.model_cell;
indnom = sys.indnom;
model = get_elt_multidim(model_cell,indnom);
nummodels = size(sys.model_cell,1);

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SETTINGS TAGS, DEFAULT SETTINGS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Algorithm list to execute -- default
switch systag
    case sysnames.hsv
        alg_list_default = {
                                algnames.lq_K0
                                algnames.mi_dirl
                                algnames.lq_opt_nu
                                algnames.fbl
                                        };
end


% Algorithm list to execute -- MI-DIRL only
alg_list_midirl_only = { algnames.mi_dirl };


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE PRESET GROUP CELL
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Number of preset groups executed
group_settings_master = cell(numgroups, 1);


% ***********************
%       
% PRESET GROUP NAME, ALGORITHM NAMES, SYSTEM NAMES
%     

for i = 1:numgroups

    % Group name
    groupnamei = preset_group_list{i};
    group_settings_master{i}.groupname = groupnamei;

    % Algorithm names
    group_settings_master{i}.algnames = algnames;

    % System tag
    group_settings_master{i}.systag = systag;

    % System names
    group_settings_master{i}.sysnames = sysnames;    

    % Master plot settings
    group_settings_master{i}.psett_master = psett_master;    

end

% ***********************
%       
% DETERMINE IF IS A SWEEP PRESET GROUP OR NOT, OTHER GROUP TRAITS BASED ON
% GROUP NAME
%     

for i = 1:numgroups

    % Group name
    groupnamei = group_settings_master{i}.groupname;

    % Check group name for '_sweep'
    strg = '_sweep';
    issweep = contains(groupnamei,strg);

    % Check group name for '_sweep_IC'
    strg = '_sweep_IC';
    issweep_IC = contains(groupnamei,strg);

    % Check group name for '_sweep_nu'
    strg = '_sweep_nu';
    issweep_nu = contains(groupnamei,strg);

    % Check group name for '_sweep_rand_nu'
    strg = '_sweep_rand_nu';
    issweep_rand_nu = contains(groupnamei,strg);    

    % Check group name for '_nom': Is on the nominal model (=1) or
    % perturbed model (=0)
    strg = '_nom';
    nom1error0 = contains(groupnamei,strg); 

    % Check group name for '_eval_val_pol'
    strg = '_eval_val_pol';
    iseval = contains(groupnamei,strg);

    % Check group name for '_eval_sweep_rand_nul'
    strg = '_eval_sweep_rand_nu';
    iseval_rand_nu = contains(groupnamei,strg);    

    % Check group name for 'freq_resp'
    strg = 'freq_resp';
    isfreqresp = contains(groupnamei,strg);

    % Check group name for 'stat_dyn'
    strg = 'stat_dyn';
    isstatdyn = contains(groupnamei,strg);

    % Handle sweep logic
    issweep_freq_resp = issweep && isfreqresp;
    issweep_statdyn = issweep && isstatdyn;

    % Handle plotting logic
    isplot = isfreqresp || isstatdyn;

    % Set sweep type
    if issweep
        if issweep_IC
            sweeptype = sweeptypenames.IC;
        elseif issweep_nu
            sweeptype = sweeptypenames.nu;
        elseif issweep_rand_nu
            sweeptype = sweeptypenames.rand_nu;        
        else
            sweeptype = sweeptypenames.plot;
        end
        group_settings_master{i}.sweeptype = sweeptype;
    end

    % Set flags
    group_settings_master{i}.issweep_IC = issweep_IC;
    group_settings_master{i}.issweep_nu = issweep_nu;
    group_settings_master{i}.issweep_rand_nu = issweep_rand_nu;
    group_settings_master{i}.issweep_freq_resp = issweep_freq_resp;
    group_settings_master{i}.issweep_statdyn = issweep_statdyn;
    group_settings_master{i}.issweep = issweep;
    group_settings_master{i}.nom1error0 = nom1error0;
    group_settings_master{i}.iseval = iseval;
    group_settings_master{i}.iseval_rand_nu = iseval_rand_nu;
    group_settings_master{i}.isfreqresp = isfreqresp;
    group_settings_master{i}.isstatdyn = isstatdyn;

    group_settings_master{i}.isplot = isplot;

    % Sweep state variable indices
    group_settings_master{i}.inds_x_sweep = model.inds_xr;

end

% ***********************
%       
% DETERMINE IF IS A TRAINING PRESET GROUP OR NOT
%
% DETERMINE IF IS A DIRL TRAINING PRESET GROUP OR NOT
%  

for i = 1:numgroups

    % Group name
    groupnamei = group_settings_master{i}.groupname; 

    % Is training (=1) or not (=0)
    group_settings_master{i}.istraining = contains(groupnamei,'training');

end

% ***********************
%       
% DETERMINE WHETHER OR NOT TO SAVE TRAINING DATA IF IS A TRAINING GROUP
%  

for i = 1:numgroups

    % Is training (=1) or not (=0)
    if group_settings_master{i}.istraining
        group_settings_master{i}.savetrainingdata = ...
            ~group_settings_master{i}.issweep;
    end
    
end

% ***********************
%       
% DETERMINE WHETHER OR NOT TO RUN ALGORITHMS
% 

for i = 1:numgroups

    group_settings_master{i}.runpresets = ...
 ~(group_settings_master{i}.iseval || group_settings_master{i}.isplot);
    
end




% ***********************
%       
% ALGORITHMS EXECUTED, NUMBER OF ALGORITHMS EXECUTED
%     

for i = 1:numgroups

    % If is a training preset, only execute DIRL. Else, execute all
    % algorithms
    if group_settings_master{i}.istraining
            switch preset_group_list{i}

            % MI-DIRL training groups
            case[   
                    {'ES_nom_dirl_training'};...
                    {'ES_error_dirl_training'};...
                    {'ES_error_dirl_training_sweep_IC_CL'};...
                    {'ES_error_dirl_training_sweep_IC_CD'};...
                    {'ES_error_dirl_training_sweep_IC_CMa'};...
                    {'ES_error_dirl_training_sweep_nu_CL_CD'};...
                    {'ES_error_dirl_training_sweep_nu_CL_CMa'};...
                    {'ES_error_dirl_training_sweep_nu_CD_CMa'};...
                    {'ES_error_dirl_training_sweep_rand_nu'};...
                    ]

            group_settings_master{i}.alg_list = alg_list_midirl_only;

            end
    else
        if ~group_settings_master{i}.isplot
            group_settings_master{i}.alg_list = alg_list_default;
        else
            group_settings_master{i}.alg_list = alg_list_midirl_only;
        end
    end

    % Number of presets executed
    group_settings_master{i}.numpresets =...
    size(group_settings_master{i}.alg_list,1);

end

% ***********************
%       
% DETERMINE WHETHER TO SAVE PRESET GROUP DATA OR NOT
%    

for i = 1:numgroups
    
    % Store system parameters
    group_settings_master{i}.savedata = savedata;    

end


% ***********************
%       
% CONFIGURE NOMINAL, SIMULATION MODEL, SET SYSTEM
%    

for i = 1:numgroups

%     % Store system parameters
%     group_settings_master{i}.sys = master_settings.sys;
    
    % Store plot settings
    group_settings_master{i}.sys_plot_settings =...
        master_settings.sys_plot_settings;

    % Check if to use the nominal model (=1) or perturbed model (=0)
    if group_settings_master{i}.nom1error0

        % ***********************
        %       
        % NOMINAL MODEL
        %     

            % Nominal model
            model_nom_tag = 'default';
%             model_nom_tag = 'perturbed';        
    
            % Simulation model
            model_sim_tag = 'default';
%             model_sim_tag = 'perturbed';
             

    else

        % ***********************
        %       
        % PERTURBED MODEL
        %             

            % Nominal model
            model_nom_tag = 'default';
%             model_nom_tag = 'perturbed';        
    
            % Simulation model
%             model_sim_tag = 'default';
            model_sim_tag = 'perturbed';
    
    end

    % Set value in the struct
    group_settings_master{i}.model_nom_tag = model_nom_tag;
    group_settings_master{i}.model_sim_tag = model_sim_tag;

end


% ***********************
%       
% REFERENCE COMMAND SETTINGS
% 

for i = 1:numgroups

    % ***********************
    %       
    % DIRL TRAINING PRESET GROUPS
    %     
    if group_settings_master{i}.istraining ...
            || group_settings_master{i}.isplot ...


        % Reference command tag
        group_settings_master{i}.refcmd = 'training';

        % Reference command type
        group_settings_master{i}.refcmdtype = 'training';
    
    % ***********************
    %       
    % NON-DIRL TRAINING PRESET GROUPS
    %             
    else

        % Group name
        groupnamei = group_settings_master{i}.groupname;


        % ***********************
        %       
        % STEP V PRESET GROUPS
        %          
        if contains(groupnamei,'step_V')

            % Reference command tag
            group_settings_master{i}.refcmd = 'step_V';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'step';            

        % ***********************
        %       
        % STEP \gamma PRESET GROUPS
        %     
        elseif contains(groupnamei,'step_g')

            % Reference command tag
            group_settings_master{i}.refcmd = 'step_g';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'step';            

    end       

    end
                
end

% ***********************
%       
% DO PREFILTER OR NOT
% 

for i = 1:numgroups

    switch group_settings_master{i}.refcmdtype

        % ***********************
        %       
        % TRAINING PRESET GROUPS
        %     
        case 'training'
        
            group_settings_master{i}.pf1nopf0 = 0;
%             group_settings_master{i}.pf1nopf0 = 1;
        

        % ***********************
        %       
        % STEP PRESET GROUPS
        %          
        case 'step'

            switch systag
                case sysnames.hsv
                    group_settings_master{i}.pf1nopf0 = 1; 
            end

    end

end


% ***********************
%       
% DIRL LEARNING SETTINGS -- WHICH LOOPS TO EXECUTE
% 

for i = 1:numgroups

    % Number of presets
    numpresets = group_settings_master{i}.numpresets;

    % Holds which loops to execute for each preset in the group
    irl_setts_cell = cell(numpresets,1);

    for j = 1:numpresets

        % Determine number of loops to execute
        numloops = 2;
        % Set number of loops
        irl_setts.numloops = numloops;

        % Which loops to execute
        irl_setts.doloopvec = ones(numloops,1);

        % Store IRL settings for this preset
        irl_setts_cell{j} = irl_setts;

    end

    % Store IRL settings for this group
    group_settings_master{i}.irl_setts_cell = irl_setts_cell;

end

% ***********************
%       
% DIRL LEARNING SETTINGS
% 

for i = 1:numgroups

    % ***********************
    %       
    % TRAINING PRESET GROUPS
    %     
    if group_settings_master{i}.istraining

        % Do learning (=1) or not (=0)
        group_settings_master{i}.dolearning = 1;
        
        % Do plots for each individual preset in the group
        group_settings_master{i}.do_individual_plots =...
            ~ group_settings_master{i}.issweep;
    
        % Do post-learning sim (=1) or not (=0)
        group_settings_master{i}.dopostlearning = 0;            

    
    % ***********************
    %       
    % NON TRAINING PRESET GROUPS
    %             
    else

        % Do learning (=1) or not (=0)
        group_settings_master{i}.dolearning = 0;

        % Do plots for each individual preset in the group
        group_settings_master{i}.do_individual_plots = 0;

        % Do post-learning sim (=1) or not (=0)
        group_settings_master{i}.dopostlearning = 1;


    end

end

      
% ***********************
%       
% DIRL DATA RELATIVE PATH AND FILE NAME
%     

% Relative path
% relpath_dirl = master_settings.relpath_dirl;
relpath_dirl_nom = master_settings.relpath_dirl_nom;
relpath_dirl_error = master_settings.relpath_dirl_error;
filename_data_dirl = master_settings.filename_data_dirl;


for i = 1:numgroups

    % File name
    group_settings_master{i}.filename_data_dirl = filename_data_dirl;

    % Relative path to DIRL data
    if group_settings_master{i}.istraining || ~dirldatasweep1temp0

        switch group_settings_master{i}.model_sim_tag
        
                case 'default'
                
                    group_settings_master{i}.relpath_data_dirl = ...
                          relpath_dirl_nom;
        
                case 'perturbed'
        
                    group_settings_master{i}.relpath_data_dirl = ...
                        relpath_dirl_error;
        end

    else

        group_settings_master{i}.relpath_data_dirl = ...
                    master_settings.relpath_dirl_sweep;
    
    
    end

end


% ***********************
%       
% PLOT SETTINGS
%    

for i = 1:numgroups

    % Save figures
    group_settings_master{i}.savefigs = savefigs;

    % Relative path
    group_settings_master{i}.relpath = relpath;
    
%     % Override relative path settings in plot_main.m
%     group_settings_master{i}.ismultigroup = 1;

    % Include legend in plots
    group_settings_master{i}.dolegend = 0;  

end



% ***********************
%       
% TEXTABLE SETTINGS
%   

% Row padding
colpadlr = '|-|';
colpadr = '-|';
hhline = '\hhline{';
lbrac = '}';

for i = 1:numgroups

    % Row padding -- before adding \hhline{}
    numpresetsi = group_settings_master{i}.numpresets;
    rowpadtmp = [colpadlr repmat(colpadr, 1, numpresetsi-1)];
    rowpad1 = [colpadlr rowpadtmp];
    rowpad2 = [rowpad1 rowpadtmp];

    % Row padding -- adding \hhline{}
    rowpad1 = [hhline rowpad1 lbrac];
    rowpad2 = [hhline rowpad2 lbrac];

    group_settings_master{i}.rowpad_cell1 = {rowpad1};
    group_settings_master{i}.rowpad_cell2 = {rowpad2};

    % File names to save .txt files to
    group_settings_master{i}.filename_textable1 = 'textable.txt';
    group_settings_master{i}.filename_textable2 = 'textable_combined.txt';

end






