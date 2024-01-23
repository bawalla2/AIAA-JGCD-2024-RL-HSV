function [preset_list, group_settings] = config_preset_group( ...
    preset_group, master_settings)
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
% This program, given a desired preset group, initializes each of the
% presets to run in the group.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% preset_group      (String) Tag corresponding to the preset group to run.
% master_settings   (Struct) Master settings as initialized by main.m
%                   and config_settings.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% preset_list       ('numpresets' x 1 Cell) The i-th entry of this cell
%                   array is a string which is the tag of the desired
%                   preset to execute as the i-th preset in the group. 
% group_settings    (Struct) This is where any group-shared settings (e.g.,
%                   system, initial conditions, etc.) are stored for
%                   subsequent initialization of each of the presets. All
%                   of these settings are user-configurable, and each are
%                   well-commented below.
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
% *************************************************************************
%
% PLOT FORMATTING OPTIONS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Degree/radian conversions
D2R =   pi/180;
R2D =   180/pi;


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE PRESET GROUP
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Check if pre-existing preset group settings have been initialized
if isfield(preset_group, 'group_settings')
    group_settings = preset_group.group_settings;
    preset_group = preset_group.tag;
end

% Store the preset group tag in the 'group_settings' struct
group_settings.preset_group = preset_group;

% Master plot settings
psett_master = group_settings.psett_master;

% Plot formatting -- line width (wide)
psett_linewidthwide = psett_master.psett_linewidthwide;

% Plot formatting -- dashed line
psett_dash = psett_master.psett_dash;

% Plot formatting -- dotted line
psett_dot = psett_master.psett_dot;

% Plot formatting -- dot-dash line
psett_dotdash = psett_master.psett_dotdash;

% Plot formatting -- colors
psett_matlabblue = psett_master.psett_matlabblue;
psett_matlaborange = psett_master.psett_matlaborange;
psett_matlabyellow = psett_master.psett_matlabyellow;
psett_matlabpurple = psett_master.psett_matlabpurple;
psett_matlabgreen = psett_master.psett_matlabgreen;
psett_matlablightblue = psett_master.psett_matlablightblue;
psett_matlabmaroon = psett_master.psett_matlabmaroon;

% Extract system names
sysnames = master_settings.sysnames;

% Extract algorithm names
algnames = master_settings.algnames;

% Array of sweep type names
sweeptypenames = master_settings.sweeptypenames;


% *****************************************************************
%
% PRESET GROUP SHARED SETTINGS      
%        

% Tag of system being executed
systag = group_settings.systag;

% Extract algorithm list
alg_list = group_settings.alg_list;

% Number of algorithms executed
numalgs = size(alg_list,1);

% Is a sweep preset group (=1) or not (=0)
issweep = group_settings.issweep;
issweep_IC = group_settings.issweep_IC;
issweep_nu = group_settings.issweep_nu;

% Get current sweep type if applicable
if issweep
    sweeptype = group_settings.sweeptype;
end

% Model linear (=1) or nonlinear (=0)
lin1nonlin0vec = zeros(numalgs,1);

% Do prefilter (=1) or not (=0)
pf1nopf0vec = group_settings.pf1nopf0 * ones(numalgs,1);

% Legend, formatting cells, algorithm settings
preset_list = cell(numalgs,1);
lgd_p = alg_list;
indiv_sett_cell = cell(numalgs,1);
color_sett_cell = cell(numalgs,1);
for i = 1:numalgs
    switch alg_list{i}
        case algnames.mi_dirl
            color_sett_cell{i} = psett_matlabblue;
            indiv_sett_cell{i} = ...
                [psett_linewidthwide; color_sett_cell{i}];
            lin1nonlin0vec(i) = 0;                        
            preset_list{i} = 'dirl_nonlin';                                 
        case algnames.lq_opt_nu
            color_sett_cell{i} = psett_matlaborange;  
            indiv_sett_cell{i} = ...
                [psett_dash; color_sett_cell{i}];
            lin1nonlin0vec(i) = 0;     
        preset_list{i} = 'lq_servo_inout';                  
        case algnames.lq_K0
            color_sett_cell{i} = psett_matlabyellow;                    
            indiv_sett_cell{i} = color_sett_cell{i};
            lin1nonlin0vec(i) = 0;     
        preset_list{i} = 'lq_servo_inout';            
        case algnames.fbl                   
            color_sett_cell{i} = psett_matlabgreen;            
            indiv_sett_cell{i} = color_sett_cell{i};
            lin1nonlin0vec(i) = 0;                        
            preset_list{i} = 'ndi_wang_stengel_2000_Vg';   
    end

end


% ***********************
%
% GET SYSTEM DATA
%   

% System
sys = master_settings.sys;
model_cell = sys.model_cell;
indnom = sys.indnom;
model = get_elt_multidim(model_cell,indnom);

% Dimensions of system cell array
numnu1 = master_settings.numnu1;
numnu2 = master_settings.numnu2;
nummodels = master_settings.nummodels;

% Trim
x_e = model.trimconds.xe;

% State dimension n
n = size(x_e, 1);

% Input dimension m
m = model.m;

% Sweep variable indices
inds_x_sweep = group_settings.inds_x_sweep;

% Trim speed (ft/s)
x1_e = x_e(model.indV);  
% Trim FPA (rad)
x2_e = x_e(model.indg); 

% Degree/radian conversions
D2R =   pi/180;
R2D =   180/pi;



% ***********************
%
% ICS
%   

% Vectors of initial values (x_{1}(0), x_{2}(0)) to test in the IC
% sweep -- before trim applied
% x_{1}(0) -- in kft/s
%         tx10vec_sweep = 0.1*(-1:1:1)';    % TEST
tx10vec_sweep = 0.1*(-1:0.25:1)';       % FINAL
% x_{2}(0) -- in deg   
%         tx20vec_sweep = (-1:1:1)';      % TEST
tx20vec_sweep = (-1:0.25:1)';      % FINAL

% If current preset group is a sweep, then take sweep ICs. Else,
% use single IC
if issweep_IC
   
    tx10vec = tx10vec_sweep;
    tx20vec = tx20vec_sweep;

else

    tx10vec = 0;
    tx20vec = 0;         

end


% Number of ICs tested in each variable
numx10 = size(tx10vec,1);
numx20 = size(tx20vec,1);
numICs = numx10 * numx20;

% Apply deg -> rad, deg/s -> rad/s conversion for ICs x_{0}
tx10vec = tx10vec * 1000;
tx10vec_sweep = tx10vec_sweep * 1000;                 
tx20vec = tx20vec * D2R;
tx20vec_sweep = tx20vec_sweep * D2R;   

% Apply shift by trim
x10vec = tx10vec + x1_e;
x10vec_sweep = tx10vec_sweep + x1_e;
x20vec = tx20vec + x2_e;
x20vec_sweep = tx20vec_sweep + x2_e;

% Initialize IC matrix: entry (i,j,:) contains the total IC vector
% with x10vec(i), x20vec(j) in their appropriate place
x0mat = zeros(numx10,numx20,n);
for i = 1:numx10
    for j = 1:numx20
        x0mat(i,j,:) = x_e;
        x0mat(i,j,inds_x_sweep(1)) = x10vec(i);
        x0mat(i,j,inds_x_sweep(2)) = x20vec(j);
    end
end

% Find indices of trim states in sweep
indsxe = zeros(2,1);
for i = 1:2
    ind_xri = inds_x_sweep(i);
    xei = x_e(ind_xri);
    switch i
        case 1
            xi0vec = x10vec_sweep;
        case 2
            xi0vec = x20vec_sweep;
    end
    indsxe(i) = find(xi0vec == xei);
end

% ***********************
%
% REFERENCE COMMAND SETTINGS
%
refcmd = group_settings.refcmd;
refcmdtype = group_settings.refcmdtype;

switch refcmd

    case 'training'

        % Simulation, plot time
        tsim = 100; 
        tsim_plot = tsim;          

    case 'step_V'

        % Simulation, plot time
        tsim = 200;          
        tsim_plot = 100;    

        % 100 ft/s step-velocity command
        x1r = 100;
        x2r = 0;

        % Output index step is being applied to
        indstep = 1;

    case 'step_g'

        % Simulation, plot time
        tsim = 20;          
        tsim_plot = 20;  

        % 1 deg FPA command
        x1r = 0;
        x2r = 1*D2R;  

        % Output index step is being applied to
        indstep = 2;

end

% ***********************
%
% REFERENCE COMMANDS
%

switch refcmdtype

    case 'step'

        switch m
            case 1
                % Set params
                x1_m = x1_e + x1r;
        
                biasvec = x1_m;
                nderivvec = 0;
                cos1_sin0 = 0;
                Amat = 0;
                Tmat = 1;  
            otherwise
                % Set params
                x1_m = x1_e + x1r;
                x2_m = x2_e + x2r;
        
                biasvec = [x1_m; x2_m];
                nderivvec = [0; 0];
                cos1_sin0 = zeros(2,1);
                Amat = zeros(2,1);
                Tmat = ones(2,1);  
        
                % Output index step is being applied to
                group_settings.indstep = indstep;                           
        
        end

end

% Decaying IC exponential to ref cmd
doexpx0 = 1;     
expx0avec = [0.025; 1.5];    

% ***********************
%
% PLOT SETTINGS
%

% Print performance metrics (=1) or don't print (=0)
print_metrics = 0;

% Threshold percentages to calculate rise time, settling time (%)
trpctvec = [90];
tspctvec = [10; 1];

numtr = size(trpctvec,1);
numts = size(tspctvec,1);

% Threshold values to determine settling time
% Column 1: thresholds for y_1
% Column 2: thresholds for y_2
threshmat = [ 1     0.01
              10    0.1   ];

% Get variable-name friendly threshold values for storage
numthresh = size(threshmat,1);
threshmat_txt = cell(numthresh,2);
for i = 1:sys.m
    for j = 1:numthresh
        threshmat_txt{j,i} = num2filename(threshmat(j,i));
    end
end      

% ***********************
%
% STORE SETTINGS
%

% Relative path to write data to (on top of relpath)
group_settings.relpath_data = 'data\';        

% Number of presets
group_settings.numpresets = numalgs;

% Print performance metrics (=1) or don't print (=0)
group_settings.print_metrics = print_metrics;

% Thresholds to determine settling time
group_settings.trpctvec = trpctvec;
group_settings.tspctvec = tspctvec;
group_settings.threshmat = threshmat;      
group_settings.threshmat_txt = threshmat_txt;

% Legend entries
group_settings.lgd_p = lgd_p;

% Individual plot settings
group_settings.color_sett_cell = color_sett_cell;        
group_settings.indiv_sett_cell = indiv_sett_cell;

group_settings.tsim = tsim;
group_settings.tsim_plot = tsim_plot;

% ICs
%         group_settings.x0 = x0;
ICs.numx10 = numx10;
ICs.numx20 = numx20;
ICs.numICs = numICs;
ICs.x10vec = x10vec;
ICs.x20vec = x20vec;
ICs.tx10vec = tx10vec_sweep;
ICs.tx20vec = tx20vec_sweep;
ICs.x0mat = x0mat;
ICs.indsxe = indsxe;
group_settings.ICs = ICs;

% Model linear (=1) or nonlinear (=0)
group_settings.lin1nonlin0vec = lin1nonlin0vec;

% Do prefilter (=1) or not (=0)
group_settings.pf1nopf0vec = pf1nopf0vec;
group_settings.doexpx0 = doexpx0;
if doexpx0
   group_settings.expx0avec = expx0avec;
end

% Commanded airspeed, FPA
switch refcmdtype
    case 'step'
        r_sett.tag = 'sum_sin';
        switch m
            case 1
                r_sett.Arvec = x1r;
            case 2
                r_sett.Arvec = [x1r; x2r];
        end
        r_sett.biasvec = biasvec;
        r_sett.nderivvec = nderivvec;
        r_sett.dorefvec = ones(2,1);
        r_sett.cos1_sin0 = cos1_sin0;
        r_sett.Amat = Amat;
        r_sett.Tmat = Tmat;           
    case 'training'
        r_sett = [];             
end
group_settings.r_sett = r_sett;


% ***********************
%
% SWEEP SETTINGS
%          

sweepsetts.issweep = issweep;
if issweep
    % Vector, each entry of which determines the number of
    % sweep parameters in the respective variable (e.g., number
    % of ICs)
    switch sweeptype
        case sweeptypenames.IC
        sweepsizevec = [numx10; numx20; nummodels];              
        case sweeptypenames.nu           
        sweepsizevec = [numnu1; numnu2]; 
        case sweeptypenames.rand_nu           
        sweepsizevec = [numnu1]; 
        otherwise
        sweepsizevec = 1;    
    end
    sweepsetts.sweepsizevec = sweepsizevec;
    % Dimension of the sweep
    sweepsetts.sweepdim = size(sweepsizevec,1);
end

% Store sweep settings
group_settings.sweepsetts = sweepsetts;        


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE GROUP SETTINGS WHICH ARE ALWAYS DECLARED
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Preset group
group_settings.preset_group = preset_group;


