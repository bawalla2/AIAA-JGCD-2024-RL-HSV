function alg_settings = config_preset(preset, group_settings, ...
    master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SELECT ALGORITHM, SYSTEM, DESIGN PARAMETERS BASED ON PRESET
%
% Brent Wallace  
%
% 2021-11-06
%
% This program, given a desired preset, handles all algorithm
% initialization/configuration. This is the main program for configuring
% specific algorithm hyperparameters (e.g., sample rates, excitations,
% etc.)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% preset            (String) Algorithm/system preset for desired example.
% group_settings    (Struct) Contains system/design parameters to
%                   be shared across all presets in the desired group.
%                   E.g., if for this preset group all designs share the
%                   same Q, R matrices, those fields may be included in
%                   this struct. This is initialized in
%                   config_preset_group.m.
% master_settings   (Struct) Master settings as initialized by main.m
%                   and config_settings.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% alg_settings  (Struct) Algorithm settings/parameters for subsequent
%               execution according to desired preset (see respective
%               algorithm .m-file for details). 
%               
%               Regardless of the preset, alg_settings will have the
%               following fields:
%
%   plot_settings       (Struct) Contains plot settings for this preset.
%                       Has the following fields:
%       legend_entry    (String) A label for this specific preset to
%                       distinguish it from the other designs executed in
%                       the preset group. E.g., if a group executes
%                       different algorithms, legend_entry for one preset
%                       might be 'IRL'.
%       plotfolder      (String) Name of the folder to save plots to for
%                       this preset. This could be the preset tag, or any
%                       other convenient identifier.
%
% NOTE: These are initialized automatically by settings in
% config_preset_group, but they nevertheless may be manually overriden.
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

% *************************************************************************
%
% UNPACK SETTINGS
% 
% *************************************************************************

% System
sys = master_settings.sys;
m = sys.m;

% Degree/radian conversions
D2R =   pi/180;
R2D =   180/pi;


% Tag of system being executed
systag = master_settings.systag;
% List of system names
sysnames = master_settings.sysnames;

% Extract algorithm names
algnames = master_settings.algnames;

% Is a sweep (=1) or not (=0)
issweep = group_settings.issweep;
issweep_IC = group_settings.issweep_IC;
issweep_nu = group_settings.issweep_nu;
issweep_rand_nu = group_settings.issweep_rand_nu;

% ICs
ICs = group_settings.ICs;
tx10vec = ICs.tx10vec;
tx20vec = ICs.tx20vec;

% ***********************
%
% GET SWEEP ITERATION VECTOR, PARAMETERS
%    

% Sweep iteration vector
sweepindvec = group_settings.sweepindvec;

% IC indices 
if issweep_IC
    indICs = sweepindvec(1:2);
    tx10 = tx10vec(indICs(1));
    tx20 = tx20vec(indICs(2));
    disp(['IC = ' num2str(tx10) '   ' num2str(tx20)])
else
    indICs = [1;1];
end

% Model index
if issweep_IC
    indmodel = sweepindvec(3);
elseif issweep_nu
    indmodel = sweepindvec(1:2);
elseif issweep_rand_nu
    indmodel = sweepindvec(1);
else
    indmodel = master_settings.indnueval;
end        

% ***********************
%
% GET ICs FOR THIS SWEEP ITERATION
%    

% Initial condition
x0 = ICs.x0mat(indICs(1),indICs(2),:);  
x0 = x0(:);

% ***********************
%
% GET NOMINAL, PERTURBED MODELS FOR THIS ITERATION
%    

% Nominal, perturbed models
model_cell = sys.model_cell;
indnom = sys.indnom;
model = get_elt_multidim(model_cell, indnom);
model_nu = get_elt_multidim(model_cell, indmodel);

% Indices of output variables
inds_xr = model.inds_xr;

% Nominal model
model_nom_tag = group_settings.model_nom_tag;
switch model_nom_tag
    case 'default'
        model_nom_ind = indnom;
    case 'perturbed'
        model_nom_ind = indmodel;
end 

% Simulation model
model_sim_tag = group_settings.model_sim_tag;
switch model_sim_tag
    case 'default'
        model_sim_ind = indnom;
    case 'perturbed'
        model_sim_ind = indmodel;
end      


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PRESET
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

switch preset

    %%    
    % *********************************************************************
    % *********************************************************************
    %
    % MI-DIRL
    %      
    % *********************************************************************
    % *********************************************************************

    
    case 'dirl_nonlin'

        % ***********************
        %
        % ALGORITHM
        %        

%         alg = strrep(preset,['_' systag],'');
        alg = preset;
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       
        
%         % Controller/model initialization settings
%         relpath_ctrl = group_settings.relpath_ctrl;

        % Get preset count
        presetcount = group_settings.presetcount;

        % Get algorithm names
        algnames = master_settings.algnames;

        % Get list of algorithms executed
        alg_list = group_settings.alg_list;

        % Get name of current algorithm
        curralg = alg_list{presetcount};

        % Commanded airspeed, altitude
        r_sett = group_settings.r_sett;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);

        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);

        % Prefilter pole locations
        if pf1nopf0
            pfavec = master_settings.pfavec;   
        end

        % Is training (=1) or not (=0)
        istraining = group_settings.istraining;
   
        % Time to simululate for 
        tsim = group_settings.tsim;

        % IRL settings
        irl_setts = group_settings.irl_setts_cell{presetcount};

        % Number of loops executed for this preset
        numloops = irl_setts.numloops;

        % Which loops to train
        doloopvec = irl_setts.doloopvec;
         

        % ***********************
        %
        % GET CONTROLLER SETTINGS FOR THIS SWEEP ITERATION
        %   

        lq_data_cell = master_settings.lq_data_cell;
        lq_data_0 = master_settings.lq_data_0;
        lq_data = get_elt_multidim(lq_data_cell, indnom);

        % ***********************
        %
        % LOOP SETTINGS
        %   

        % Holds loop settings
        loop_cell = cell(numloops,1);
        

                % Sample period
                Ts = 2;         % TNNLS 2023
                nTs_begin = 10; 

            % Initialize learning params in each loop
            for k = 1:numloops
                switch k
    
                    % Loop j = 1
                    case 1

                        % x, y indices
                        tmp.indsx = 1;
                        tmp.indsy = 1;   
                        % Learning settings
                        tmp.istar = 10;
                        tmp.nTs = 6 / Ts;   
                        tmp.l = 15;                             

                        tmp.Q = lq_data.lq_data_11.Q;
                        tmp.R = lq_data.lq_data_11.R;

                        % Initial stabilizing design on nominal model
                        tmp.K0 = lq_data_0.lq_data_11.K; 
                       
    
                    % Loop j = 2   
                    case 2


                         % x, y indices
                        tmp.indsx = (2:4)';
                        tmp.indsy = 2; 
                        % Learning settings
                        tmp.istar = 10;
                        tmp.nTs = 1;
                        tmp.l = 25;   
                        % Normalization   
                        tmp.S = diag([1 1 D2R D2R]);

                        tmp.Q = lq_data.lq_data_22.Q;
                        tmp.R = lq_data.lq_data_22.R;

                        % Initial stabilizing design on nominal model
                        tmp.K0 = lq_data_0.lq_data_22.K;                         
    
                end
    
                % Set current loop settings
                loop_cell{k} = tmp;
            end


        % ***********************
        %
        % EXPLORATION NOISE
        %
        % Note: In post-transformation units (i.e., after applying S_u)
        %

        noise.cos1_sin0 = [    1   0   0
                               0   0   1    ];
        noise.Amat = [       0.05 0 0
                               1   1.5  1  ];
        noise.Tmat = [       250 25  50
                             6  25   100  ];        
        switch numloops
            case 1
                noise.donoisevec = ones(m,1);
            case 2
                noise.donoisevec = doloopvec;
        end
        noise.tag = 'multivar_sum_sinusoid';        


        % ***********************
        %
        % REFERENCE COMMANDS -- FOR TRAINING PHASE
        %

        % Reference command type
        refcmdl = 'sum_sin';

        % Trim
        x_e = model.trimconds.xe;

        switch m
            case 1
                x1_e = x_e(inds_xr(1));
                x1_m = x1_e; 
                biasvec = x1_m;
                nderivvec = 0;                
            case 2
                x1_e = x_e(inds_xr(1));
                x2_e = x_e(inds_xr(2));
                x1_m = x1_e;
                x2_m = x2_e;   
                biasvec = [x1_m; x2_m];
                nderivvec = [0; 0];
        end

        switch refcmdl

            % NO r(t)
            case 'zero'

                cos1_sin0 = zeros(m,1);
                Amat = zeros(m,1);
                Tmat = ones(m,1);

            % INSERT r(t)
            case 'sum_sin'

                cos1_sin0 = [   0   0   1  
                                0   0   1   ];
                Amat = [        50  5   5   
                                0.15 0.3 0   ];
                Tmat = [        100 25  10  
                                15  6   3     ];

                % Dividie \gamma amplitudes by a 10
                Amat(2,:) = 0.1 * Amat(2,:);

                % Convert \gamma amplitudes to rad
                Amat(2,:) = D2R * Amat(2,:);                        

        end

        % Do x_3 loop
        lenx3 = 1;
        dox3 = 1;


        % Do decaying exponential r(t) according to x_0 (=1) or not (=0)
        doexpx0 = group_settings.doexpx0;
        if doexpx0
            x_e = model.trimconds.xe;
            r_sett_train.ty0 = ...
                (x0(model.inds_xr) - x_e(model.inds_xr));  
            r_sett_train.expx0avec = group_settings.expx0avec;
        end


        % ***********************
        %
        % TRANINING SETTINGS
        %

        % Reference command settings
        switch refcmdl
            case 'zero'
                r_sett_train.tag = 'sum_sin';   
            case 'sum_sin'
                r_sett_train.tag = 'sum_sin'; 
        end
        switch refcmdl
            case {'sum_sin';'zero'}
                r_sett_train.biasvec = biasvec;
                r_sett_train.nderivvec = nderivvec;
                r_sett_train.dorefvec = noise.donoisevec;
                r_sett_train.cos1_sin0 = cos1_sin0;
                r_sett_train.Amat = Amat;
                r_sett_train.Tmat = Tmat;
        end
        



        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;     



    % *********************************************************************
    %
    % LQ SERVO INNER/OUTER
    %
    
    case 'lq_servo_inout'
    
        % ***********************
        %
        % ALGORITHM
        %        
        
        alg = 'lq_servo_inout';
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       


        % Get preset count
        presetcount = group_settings.presetcount;

        % Reference command
        r_sett = group_settings.r_sett;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);

        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);     

        % ***********************
        %
        % GET CONTROLLER SETTINGS FOR THIS SWEEP ITERATION
        %   

        lq_data_cell = master_settings.lq_data_cell;
        lq_data = get_elt_multidim(lq_data_cell, indnom);
        lq_data_0 = master_settings.lq_data_0;

        % Get algorithm names
        algnames = master_settings.algnames;

        % Get list of algorithms executed
        alg_list = group_settings.alg_list;

        % Get name of current algorithm
        curralg = alg_list{presetcount};

        % Time to simululate for 
        tsim = group_settings.tsim;
 

        % Nominal, simulation model
        switch curralg
            case algnames.lq_opt_nu
                model_nom_tag = group_settings.model_sim_tag;
                model_sim_tag = group_settings.model_sim_tag;
            otherwise
                model_nom_tag = group_settings.model_nom_tag;
                model_sim_tag = group_settings.model_sim_tag;                
        end  


        % ***********************
        %
        % GET CONTROLLER SETTINGS
        %   

        switch curralg
            case algnames.lq_opt_nu
                K = get_elt_multidim(lq_data_cell, model_sim_ind).K;               
             case algnames.lq_K0
                K = lq_data_0.K;
        end

        

        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;           


    % *********************************************************************
    %
    % HSV -- WANG, STENGEL (2000) -- FBL y = [V, \gamma^T]
    %
    
    case 'ndi_wang_stengel_2000_Vg'

        % ***********************
        %
        % ALGORITHM
        %        
        
        alg = 'ndi_wang_stengel_2000_Vg';
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       
        
%         % Controller/model initialization settings
%         relpath_ctrl = group_settings.relpath_ctrl;

        % Get preset count
        presetcount = group_settings.presetcount;

        % Commanded airspeed, altitude
        r_sett = group_settings.r_sett;
        % Change number of derivatives to 3
        r_sett.nderivvec = [3; 3];

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);

        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);
%         pf1nopf0 = 0;

        % Prefilter pole locations
        if pf1nopf0
            pfavec = master_settings.pfavec;   
        end     
       
        % Time to simululate for 
        tsim = group_settings.tsim;
        

        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;           


    % *********************************************************************    
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    % *********************************************************************
    % *********************************************************************
    
    otherwise
        
        error('*** ERROR: PRESET TAG NOT RECOGNIZED ***');  
       
end





%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE ALGORITHM SETTINGS/DESIGN PARAMETERS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% GENERAL SETTINGS
%
% *************************************************************************

% Preset tag
alg_settings.preset = preset;

% Plot settings -- general
plot_settings.legend_entry = legend_entry;
plot_settings.plotfolder = plotfolder;
% plot_settings.sys_settings = sys_settings;

% Write plot settings
alg_settings.plot_settings = plot_settings;


% *************************************************************************
%
% ALGORITHM-SPECIFIC SETTINGS
%
% *************************************************************************

switch alg

    % *********************************************************************
    %
    % DIRL
    %
    
    case 'dirl_nonlin'

        alg_settings.alg = alg;

        % Nominal, perturbed models
        alg_settings.indmodel = indmodel;

        % Whether or not to perform learning in each loop
        alg_settings.doloopvec = irl_setts.doloopvec;    

        % Simulate w = f(x) - Ax for simulation model (=1) or not (=0)
        % DEBUGGING ONLY
        alg_settings.sim_w = 0;

        % Nominal model
        alg_settings.model_nom_tag = model_nom_tag;
        alg_settings.model_nom_ind = model_nom_ind;

        % Simulation model
        alg_settings.model_sim_tag = model_sim_tag;
        alg_settings.model_sim_ind = model_sim_ind;


        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = ...
            group_settings.lin1nonlin0vec(presetcount);


        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);
        alg_settings.pf1nopf0 = pf1nopf0;

        % Prefilter pole locations
        if pf1nopf0
            alg_settings.pfavec = master_settings.pfavec; 
        end  

        % Exploration noise
        alg_settings.noise = noise;    

        % Reference command settings -- training phase
        alg_settings.r_sett_train = r_sett_train;

        % Reference command settings
        alg_settings.r_sett = group_settings.r_sett;

        % Loop settings
        alg_settings.loop_cell = loop_cell;

        % Sample period
        alg_settings.Ts = Ts;

        % Sampling begin offset
        alg_settings.nTs_begin = nTs_begin;

        % Do x_3 loop
        alg_settings.dox3 = dox3;
        alg_settings.lenx3 = lenx3;

        % Is training preset (=1) or not (=0)
        istraining = group_settings.istraining;
        alg_settings.istraining = istraining;
        
        % Do reference command r(t) injection (=1) or not (=0)
        alg_settings.dort = 1;

        % Overwrite current controller (=1) or not (=0)
        alg_settings.updatecontroller = istraining && ~issweep_IC;

        % ICs, simulation time
        alg_settings.x0 = x0;  
        alg_settings.tsim = group_settings.tsim; 

        % Use the nominal linearization A for w = f(x) - A x at nominal
        % trim (=1) or simulation trim (=0)
        % DEBUGGING ONLY
        alg_settings.wnom1sim0 = 1;

    % *********************************************************************
    %
    % LQ SERVO INNER/OUTER
    %
    
    case 'lq_servo_inout'

        alg_settings.alg = alg;

        % Nominal, perturbed models
        alg_settings.model = model;
        alg_settings.model_nu = model_nu;

        % Nominal model
        alg_settings.model_nom_tag = model_nom_tag;
        alg_settings.model_nom_ind = model_nom_ind;

        % Simulation model
        alg_settings.model_sim_tag = model_sim_tag;
        alg_settings.model_sim_ind = model_sim_ind;

        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = lin1nonlin0;

        % Do prefilter (=1) or not (=0)
        alg_settings.pf1nopf0 = pf1nopf0;

        % Prefilter pole locations -- chosen to achieve design specs
        if pf1nopf0
            switch curralg
                case algnames.lq_K0
                    alg_settings.pfavec = [0.0575; 0.33];
                otherwise
                    alg_settings.pfavec = master_settings.pfavec;          
            end
        end   

        % Commanded airspeed, FPA
        alg_settings.r_sett = r_sett;

        % Controller
        alg_settings.K = K;

        alg_settings.x0 = x0;  
        alg_settings.tsim = tsim; 
  


   
    % *********************************************************************
    %
    % HSV -- WANG, STENGEL (2000) -- FBL y = [V, \gamma^T]
    %
    
    case 'ndi_wang_stengel_2000_Vg'

        alg_settings.alg = alg;
        
        % Output variables to track
        alg_settings.inds_xr = inds_xr;

        % Simulation model
        alg_settings.model_sim_tag = model_sim_tag;
        alg_settings.model_sim_ind = model_sim_ind;

        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = lin1nonlin0;

        % Do prefilter (=1) or not (=0)
        alg_settings.pf1nopf0 = pf1nopf0;

        % Prefilter pole locations -- chosen to achieve design specs
        if pf1nopf0 
            alg_settings.pfavec = [0.061; pfavec(2)]; 
        end

        % Commanded airspeed, FPA
        alg_settings.r_sett = r_sett;

        alg_settings.x0 = x0;  
        alg_settings.tsim = tsim; 

        
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    
    otherwise
        
        error('**** ERROR: ALGORITHM TAG NOT RECOGNIZED ***');  
       
end
