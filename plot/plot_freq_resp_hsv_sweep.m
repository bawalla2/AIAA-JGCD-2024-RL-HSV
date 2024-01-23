function [figcount, out_data] = plot_freq_resp_hsv_sweep(...
    alg_settings_cell, out_data_cell, group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% FREQUENCY RESPONSE PLOTS FOR HSV
%
% Brent Wallace  
%
% 2023-09-18
%
% This program handles frequency response plots for HSVs.
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

% Unpack plot settings
savefigs = master_settings.savefigs;
if savefigs
    relpath = group_settings.relpath;
end

% Initialize figure counter
figcount = master_settings.figcount;

% Pull dEIRL data from sweep data location (=1) or temp data location (=0)
dirldatasweep1temp0 = master_settings.dirldatasweep1temp0;

% Frequency points for SV plots
wvec = master_settings.wvec;

% Master plot settings
psett_master = master_settings.psett_master;

% Plot formatting -- dashed line
psett_dash = psett_master.psett_dash;

% Plot formatting -- colors
psett_matlabblue = psett_master.psett_matlabblue;
psett_matlaborange = psett_master.psett_matlaborange;
psett_matlabyellow = psett_master.psett_matlabyellow;
psett_matlabpurple = psett_master.psett_matlabpurple;
psett_matlabgreen = psett_master.psett_matlabgreen;
psett_matlablightblue = psett_master.psett_matlablightblue;
psett_matlabmaroon = psett_master.psett_matlabmaroon;

% Algorithm names
algnames = master_settings.algnames;


% System
sys = master_settings.sys;             % System array
n = sys.n;                          % System order
m = sys.m;                          % System input dimension

% System cell array
model_cell = sys.model_cell;

% Perturbation params
nuvec = sys.nuvec;
% nueval = nuvec(indmodel);
indsmodelplot = master_settings.indsmodelplot;
nummodelsplot = master_settings.nummodelsplot;
nutag = sys.nutag;

% LQ data cell
lq_data_cell = master_settings.lq_data_cell;

% LQ data cell -- initial stabilizing controller
lq_data_0 = master_settings.lq_data_0;

% Include h in the design plant (=1) or not (=0)
h1noh0 = 1;

% Feedback matrix M_i
if h1noh0
    Mi = [zeros(2) eye(2) zeros(2,1)];
else
    Mi = [zeros(2) eye(2)];
end

% Initialize output data
out_data = cell(nummodelsplot,1);

% ***********************
%       
% LOAD dEIRL SWEEP DATA
%

% Relative path to load dEIRL learning data
relpath_dirl_sweep = master_settings.relpath_dirl_sweep;

% Load dEIRL sweep data
data = load([relpath_dirl_sweep 'alg_settings_cell_master.mat']);
data = data.alg_settings_cell_master;
alg_settings_dirl_sweep = data{1};
data = load([relpath_dirl_sweep 'out_data_cell_master.mat']);
data = data.out_data_cell_master;
out_data_dirl_sweep = data{1};
data = load([relpath_dirl_sweep 'group_settings_master.mat']);
data = data.group_settings_master;
group_settings_dirl_sweep = data{1};


% IC sweep data
ICs = group_settings_dirl_sweep.ICs;
indsxe = ICs.indsxe;

% ***********************
%       
% DATA STORAGE
%

% Holds \Hinf norms at each modeling error
hinfmat = zeros(nummodelsplot,3*4);

%%
% *************************************************************************
% *************************************************************************
%
% MAIN LOOP
%
% *************************************************************************
% *************************************************************************

for mcnt = 1:nummodelsplot

% ***********************
%       
% GET CURRENT MODEL INDEX
%

indmodel = indsmodelplot(mcnt);
nui = nuvec(indmodel);
nui_str = num2filename(nui);

% Set current relative path
if savefigs
    relpath_tmp = [relpath nui_str '/'];
    % Make directory
    if ~exist(relpath_tmp, 'dir')
        mkdir(relpath_tmp);
    end
end


% ***********************
%       
% EXTRACT MODEL DATA AT THIS \nu
%

% Perturbed model
model_nu = get_elt_multidim(model_cell, indmodel); 

% Linearization -- perturbed
lin_nu = model_nu.lin;
io_nu = lin_nu.io;
Pdh_nu = io_nu.Pdh;
Pd_nu = io_nu.Pd;

% ***********************
%       
% EXTRACT OPTIMAL LQ DATA
%

% Get LQ data 
lq_data_nu = get_elt_multidim(lq_data_cell, indmodel);  

% ***********************
%       
% EXTRACT dEIRL DATA
%

% Extract current out_data struct 
% Corresponding to the dEIRL algorithm data run at IC x_0 = x_e for the
% desired model
outdata = out_data_dirl_sweep{indsxe(1),indsxe(2),indmodel};

% Extract current lq_data struct
lq_data_dirl = outdata.dirl_data.lq_data;

% *************************************************************************
% *************************************************************************
%
% COMPARISON OF NOMINAL DESIGN, dEIRL, OPTIMAL LQ DESIGN FOR VARYING \nu
%
% *************************************************************************
% *************************************************************************


% ***********************
%       
% dEIRL SYSTEM
%

% Get LQ servo controller gains
Kz = lq_data_dirl.Kz;
Ky = lq_data_dirl.Ky;
Kr = lq_data_dirl.Kr;

% Inner-loop controller K_i ssr
Ki = ss([],[],[],Kr);

% Outer-loop controller K_o ssr
Ko = ss(zeros(m),-eye(m),-Kz,Ky);

% Set system params
if h1noh0
    sysdirl.P = Pdh_nu;
else
    sysdirl.P = Pd_nu;
end
sysdirl.Ki = Ki;
sysdirl.Ko = Ko;
sysdirl.Mi = Mi;

% ***********************
%       
% OPTIMAL LQ SYSTEM
%

% Get LQ servo controller gains
Kz = lq_data_nu.Kz;
Ky = lq_data_nu.Ky;
Kr = lq_data_nu.Kr;

% Inner-loop controller K_i ssr
Ki = ss([],[],[],Kr);

% Outer-loop controller K_o ssr
Ko = ss(zeros(m),-eye(m),-Kz,Ky);

% Set system params
if h1noh0
    sysc_nu.P = Pdh_nu;
else
    sysc_nu.P = Pd_nu;
end
sysc_nu.Ki = Ki;
sysc_nu.Ko = Ko;
sysc_nu.Mi = Mi;

% ***********************
%       
% INITIAL STABILIZING CONTROLLER K_0
%

% Get LQ servo controller gains
Kz = lq_data_0.Kz;
Ky = lq_data_0.Ky;
Kr = lq_data_0.Kr;

% Inner-loop controller K_i ssr
Ki = ss([],[],[],Kr);

% Outer-loop controller K_o ssr
Ko = ss(zeros(m),-eye(m),-Kz,Ky);

% Set system params
if h1noh0
    sys_K0.P = Pdh_nu;
else
    sys_K0.P = Pd_nu;
end
sys_K0.Ki = Ki;
sys_K0.Ko = Ko;
sys_K0.Mi = Mi;


% ***********************
%       
% PLOT RESPONSES
%

% Frequency response y-limits
ymin = -20;
ymax = 12;
custom_set_tmp.Se.axlim_y = [ymin ymax];
custom_set_tmp.Te.axlim_y = [ymin ymax];
custom_set_tmp.Se_Te.axlim_y = [ymin ymax];
custom_set_tmp.Su.axlim_y = [ymin ymax];
custom_set_tmp.Tu.axlim_y = [ymin ymax];
custom_set_tmp.Su_Tu.axlim_y = [ymin ymax];

% Settings for plotting frequency responses
lgd_tmp = {algnames.lq_K0, algnames.mi_dirl, algnames.lq_opt_nu};
indiv_set_tmp = {psett_matlabyellow; psett_matlabblue; ...
    [psett_dash; psett_matlaborange]};
in_data.sys_cell = {sys_K0; sysdirl; sysc_nu};

in_data.wvec = wvec;
in_data.figcount = figcount;
in_data.lgd = lgd_tmp;
in_data.indiv_sett_cell = indiv_set_tmp;
% in_data.plotgroup = 'all';
% in_data.plotgroup = 'euw_basic';
in_data.plotgroup = 'S_T';
in_data.custom_sett_cell = custom_set_tmp;
if savefigs
    in_data.relpath = relpath_tmp;
    %in_data.postfilename = ['_' nutag '_' nui_str];
end
[figcount, out_datai] = plot_CLTFM_io(in_data);
clear in_data;

% Store data
out_data{mcnt} = out_datai;

hinfmat(mcnt,:) = out_datai.hinfmat(1:12);

end

% *************************************************************************
% *************************************************************************
%
% DISPLAY \Hinf DATA
%
% *************************************************************************
% *************************************************************************

disp('% ***********************')
disp('% ')
disp('% \Hinf DATA')
disp('% ')
disp('% ***********************')
disp(' ')

% Display formatting
onetab = '  ';
twotab = [ onetab onetab ];
threetab = [ twotab onetab ];
fourtab = [ twotab twotab ];
eighttab = [ fourtab fourtab ];
sixteentab = [ eighttab eighttab ];

nomdEIRLopt = ['Nom'  fourtab 'dEIRL'  fourtab 'opt' fourtab];
repnomdEIRLopt = [nomdEIRLopt nomdEIRLopt nomdEIRLopt nomdEIRLopt];
mapsdisp = ['S_e' sixteentab 'T_e' sixteentab 'S_u' sixteentab 'T_u'];

% Display data
disp(mapsdisp)
disp(repnomdEIRLopt)
disp(num2str(hinfmat))
disp(num2str(hinfmat, '%.2f'))


