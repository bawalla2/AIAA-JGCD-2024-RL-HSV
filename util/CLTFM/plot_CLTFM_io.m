function [figcount, out_data] = plot_CLTFM_io(in_data)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% FORM CLOSED-LOOP TRANSFER FUNCTION MATRICES -- INNER-OUTER LOOP STRUCTURE
%
% Brent Wallace  
%
% 2022-10-31
%
% This program, given state-space representations of a plant P, inner-loop
% controller K_i, outer-loop controller K_o, and inner-loop state feedback
% matrix M_i, constructs state-space representations of all relevant
% closed-loop maps in the feedback loop.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%   
% in_data                  (Struct) Input data. Has the following fields:
%
%   sys_cell                ('numpresets' x 1 Cell Array) Contains each
%                           system's plant, controller info. Each entry has
%                           the following fields:
%       P                   (m x p State-Space Object) Linear plant
%                           dynamical system. Order = n.
%       Ki                  (p x m State-Space Object) Inner-loop
%                           controller dynamical system. Order = n_i.
%       Ko                  (m x n_{x_r} State-Space Object) Outer-loop
%                           controller dynamical system. Order = n_o.
%       Mi                  (n_{x_r} x n Matrix) Matrix which reads out the
%                           states to be fed back to the inner loop. For
%                           example, if the system has order n = 5 and we
%                           desire to feed back states 2 and 3, then
%                           n_{x_r} = 2 and M_i would be:
%                               Mi = [  0 1 0 0 0
%                                       0 0 1 0 0   ].
%   wvec                    (Vector) Contains frequency points to evaluate
%                           responses at.
%   figcount                (Integer) Current figure count to begin plots
%                           at.
%   relpath                 (String, OPTIONAL) If it is desired to save
%                           figures, declare this entry. Contains the
%                           relative file path to save figures to.
%   plotgroup               (String, OPTIONAL) Specifies which maps the
%                           user would like to plot. Has the following
%                           options:
%       [empty]             Default option (don't pass this argument).
%                           Plots open/closed loop maps at the error and
%                           controls to the usual outputs of interest.
%       'euw_basic'         Plots open/closed loop maps at the error and
%                           controls, as well as filtered closed-loop maps
%                           to the usual outputs of interest.
%       'euw'               Plots open/closed loop maps at the error and
%                           controls, as well as filtered closed-loop maps.
%       'all'               Plot all maps listed previously, plus those
%                           associated with output disturbance d_o and
%                           inner-loop noise n_i.
%   indiv_sett_cell         ('numpresets' x 1 Cell Array, OPTIONAL)
%                           Contains individual plot formatting settings
%                           for each preset, if custom settings are
%                           desired. See plot_format.m for details.
%   custom_sett_cell        (Struct, OPTIONAL) This cell array
%                           allows the user to customize individual
%                           frequency response plot settings. Each of the
%                           entries is a 'custom_sett' struct (cf.
%                           plot_format.m) containing the custom settings
%                           of the corresponding plot. The name of the
%                           entries must agree with the respective
%                           'filename' entry of the plot. E.g., if one
%                           wanted to make the compsen at the error plot
%                           have y-limits [-20, 10] dB, one would declare
%                           'custom_sett_cell' as:
%                           custom_sett_cell.Te.axlim_y = [-20 10];                           
%   caption                 (String, OPTIONAL) Caption to put after each
%                           figure title (for the sake of identification,
%                           etc.)
%   lgd                     ('numpresets' x 1 Cell Array, OPTIONAL)
%                           Each entry is a string with the legend entry to
%                           label the respective preset with in the plot
%   postfilename            (String, OPTIONAL) String to add behind each
%                           file name of saved figures
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) Output data. Has the following fields:
%
% *** OPEN-LOOP MAPS:
%
%   Le                      OL map e -> y (broken at error)
%   Lu                      OL map u_p -> u (broken at controls)
%
% *** CLOSED-LOOP MAPS:
%
%   KSe                     CL map r -> u
%   Te                      CL map r -> y
%   Se                      CL map r -> e
%   KoSe                    CL map r -> u_o
%   Trxr                    CL map r -> x_r
%   Trui                    CL map r -> u_i
%
%   Tu                      CL map d_i -> u
%   Su                      CL map d_i -> u_p
%   PSu                     CL map d_i -> y
%   KoPSu                   CL map d_i -> u_o
%   Tdixr                   CL map d_i -> x_r
%   Tdiui                   CL map d_i -> u_i
%
%   Tdou                    CL map d_o -> u
%   Tdoy                    CL map d_o -> y
%   Tdoyp                   CL map d_o -> y_p
%   Tdoe                    CL map d_o -> e
%   Tdouo                   CL map d_o -> u_o
%   Tdoxr                   CL map d_o -> x_r
%   Tdoui                   CL map d_o -> u_i
%
%   Tniu                    CL map n_i -> u
%   Tniy                    CL map n_i -> y
%   Tnie                    CL map n_i -> e
%   Tniuo                   CL map n_i -> u_o
%   Tnixr                   CL map n_i -> x_r
%   Tniei                   CL map n_i -> e_i
%   Tniui                   CL map n_i -> u_i 
%
% *************************************************************************
%
% NOTES ON LOOP STRUCTURE
%
% *************************************************************************
%
% ***** STATE PARTITION
%
% For all of the closed-loop maps here, the state is partitioned as
%
%   x = [   x_p
%           x_i         \in R^{n + n_i + n_0}
%           x_o ]       
%
% Where x_p \in R^{n} denotes the plant state, x_i \in R^{n_i} denotes the
% inner-loop controller state, and x_o \in R^{n_o} denotes the outer-loop
% controller state.
%
% ***** INPUTS CONSIDERED
%
% r                         (m-dim) Reference command signal.
% d_i                       (p-dim) Plant input disturbance signal.
% d_o                       (m-dim) Plant output disturbance signal.
% n_i                       (n_{x_r}-dim) noise to inner-loop controller.
%
% ***** OUTPUTS CONSIDERED
%
% e                         (m-dim) Error signal.
% u                         (m-dim) Control signal (pre-input disturbance).
% u_p                       (p-dim) Plant input signal.
% y                         (m-dim) Plant output + output disturbance
%                           signal.
% y_p                       (m-dim) Plant output (pre-output disturbance).
% x_r                       (n_{x_r}-dim) Number of states fed back from
%                           the plant to the inner-loop controller K_i.
% e_i                 (n_{x_r}-dim) Input to inner-loop controller
%                           K_i (inner-loop states + inner-loop noise n_i).
%
% ***** KEY RELATIONSHIPS:
% 
% y = y_p + d_o
% e = r - y
% u = u_o - u_i
% u_p = u + d_i
% x_r = M_i * x_p
% e_i = x_r + n_i
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
% INIT   
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
% *************************************************************************
%
% EXTRACT SETTINGS   
%
% *************************************************************************
% *************************************************************************

% System cell array
sys_cell = in_data.sys_cell;
numpresets = size(sys_cell,1);

% Save figures (=1) or not (=0)
savefigs = isfield(in_data,'relpath');
if savefigs
    relpath = in_data.relpath;
end

% Vector of frequencies
wvec = in_data.wvec;
wvec = wvec(:);             % Make column vector
numwpts = size(wvec,1);

% Figure count
figcount = in_data.figcount;
figcount_start = figcount;

% Check if the user specified which group to plot
groupspecified = isfield(in_data,'plotgroup');
if groupspecified
    plotgroup = in_data.plotgroup;
else
    plotgroup = '';
end

% Check if user desired to add a custom caption to the end of each plot
% title
docaption = isfield(in_data, 'caption');
if docaption   
    caption = in_data.caption;
end

% Check if user desired preset-specific formatting
do_indiv_sett = isfield(in_data, 'indiv_sett_cell');
if do_indiv_sett   
    indiv_sett_cell = in_data.indiv_sett_cell;
end

% Check if the user wanted custom settings for any of the plots
do_custom_sett = isfield(in_data, 'custom_sett_cell');
if do_custom_sett
    % Extract custom plot settings cell
    custom_sett_cell = in_data.custom_sett_cell;
end

% Check if user wants a legend
do_lgd = isfield(in_data, 'lgd');
if do_lgd   
    lgd = in_data.lgd;
end 

% Check if the user wants to add a string behind the file names
haspostfilename = isfield(in_data,'postfilename');
if haspostfilename
    postfilename = in_data.postfilename;
else
    postfilename = '';
end

% ***********************
%       
% MISC SETTINGS
%

% Axis labels for SV plots
wlabel = 'Frequency (rad/s)';
maglabel = 'Magnitude (dB)';

% *************************************************************************
% *************************************************************************
%
% PLOTS TO GENERATE
%
% Each plot is treated as an object with the following fields:
%
%   varname_cell            (nummap x 1 Cell Array) Each entry contains the
%                           variable name of the open/closed loop map of
%                           interest. NOTE: Must match the corresponding
%                           variable name as generated by f_CLTFM_io.m (see
%                           program for details). E.g., {'Le'} for the loop
%                           broken at error (nummap = 1), or
%                           [{'Se'};{'Te'}] for sensitivity and comp.
%                           sensitivity at the error (nummap = 2).
%   engname                 (String) English name of the current plot; 
%                           e.g., 'Loop Broken at Error'.
%   texname                 (String) LaTEX name of the current plot; 
%                           e.g., '$L_{e}$'.
%   filename                (String) Name to save plot file to.
%   
%
% *************************************************************************
% *************************************************************************

% ***********************
%       
% PLANT OPEN-LOOP
%

%   P
map_P.varname_cell = {'P'};
map_P.engname = 'Plant';
map_P.texname = '$P$';
map_P.filename = map_P.varname_cell{1};

% ***********************
%       
% ERROR e
%

%   Le                      OL map e -> y (broken at error)
map_Le.varname_cell = {'Le'};
map_Le.engname = 'Loop Broken at Error';
map_Le.texname = '$L_{e}$';
map_Le.filename = map_Le.varname_cell{1};

%   KSe                     CL map r -> u
map_KSe.varname_cell = {'KSe'};
map_KSe.engname = '$K$-Sen. $T_{ru} =$';
map_KSe.texname = '$KS_{e}$';
map_KSe.filename = map_KSe.varname_cell{1};

%   Te                      CL map r -> y
map_Te.varname_cell = {'Te'};
% map_Te.engname = 'Comp. Sen. at Error $T_{ry} =$';
% map_Te.engname = 'Complementary Sensitivity at Error $T_{ry} =$';
map_Te.engname = 'Complementary Sensitivity at Error';
map_Te.texname = '$T_{e}$';
map_Te.filename = map_Te.varname_cell{1};

%   Se                      CL map r -> e
map_Se.varname_cell = {'Se'};
map_Se.engname = 'Sensitivity at Error';
map_Se.texname = '$S_{e}$';
map_Se.filename = map_Se.varname_cell{1};

%   KoSe                    CL map r -> u_o
map_KoSe.varname_cell = {'KoSe'};
map_KoSe.engname = '$K_{o}$-Sen. $T_{r u_{o}} =$';
map_KoSe.texname = '$K_{o}S_{e}$';
map_KoSe.filename = map_KoSe.varname_cell{1};

%   Trxr                    CL map r -> x_r
map_Trxr.varname_cell = {'Trxr'};
map_Trxr.engname = 'Ref. to Inner-Loop State';
map_Trxr.texname = '$T_{r x_{i}}$';
map_Trxr.filename = map_Trxr.varname_cell{1};

%   Trui                    CL map r -> u_i
map_Trui.varname_cell = {'Trui'};
map_Trui.engname = 'Ref. to Inner-Loop Control';
map_Trui.texname = '$T_{r u_{i}}$';
map_Trui.filename = map_Trui.varname_cell{1};

% S_e and T_e
map_SeTe.varname_cell = [{'Se'}; {'Te'}];
map_SeTe.engname = 'Sen. and Comp. Sen. at Error';
map_SeTe.texname = '$S_{e}$, $T_{e}$';
map_SeTe.filename = 'Se_Te';

% ***********************
%       
% CONTROLS u
%

%   Lu                      OL map u_p -> u (broken at controls)
map_Lu.varname_cell = {'Lu'};
map_Lu.engname = 'Loop Broken at Controls';
map_Lu.texname = '$L_{u}$';
map_Lu.filename = map_Lu.varname_cell{1};

%   Tu                      CL map d_i -> u
map_Tu.varname_cell = {'Tu'};
map_Tu.engname = 'Complementary Sensitivity at Controls';
map_Tu.texname = '$T_{u}$';
map_Tu.filename = map_Tu.varname_cell{1};

%   Su                      CL map d_i -> u_p
map_Su.varname_cell = {'Su'};
map_Su.engname = 'Sensitivity at Controls';
map_Su.texname = '$S_{u}$';
map_Su.filename = map_Su.varname_cell{1};

%   PSu                     CL map d_i -> y
map_PSu.varname_cell = {'PSu'};
map_PSu.engname = '$P$-Sensitivity $T_{d_{i} y} =$';
map_PSu.texname = '$PS_{u}$';
map_PSu.filename = map_PSu.varname_cell{1};

%   KoPSu                   CL map d_i -> u_o
map_KoPSu.varname_cell = {'KoPSu'};
map_KoPSu.engname = 'Input Dist. to Outer-Loop Contr. $T_{d_{i} u_{o}} =$';
map_KoPSu.texname = '$K_{o}PS_{u}$';
map_KoPSu.filename = map_KoPSu.varname_cell{1};

%   Tdixr                   CL map d_i -> x_r
map_Tdixr.varname_cell = {'Tdixr'};
map_Tdixr.engname = 'Input Dist. to Inner-Loop State';
map_Tdixr.texname = '$T_{d_{i} x_{i}}$';
map_Tdixr.filename = map_Tdixr.varname_cell{1};

%   Tdiui                   CL map d_i -> u_i
map_Tdiui.varname_cell = {'Tdiui'};
map_Tdiui.engname = 'Input Dist. to Inner-Loop Control';
map_Tdiui.texname = '$T_{d_{i} u_{i}}$';
map_Tdiui.filename = map_Tdiui.varname_cell{1};

% S_u and T_u
map_SuTu.varname_cell = [{'Su'}; {'Tu'}];
map_SuTu.engname = 'Sen. and Comp. Sen. at Controls';
map_SuTu.texname = '$S_{u}$, $T_{u}$';
map_SuTu.filename = 'Su_Tu';

% ***********************
%       
% OUTPUT DISTURBANCE d_o
%

%   Tdou                    CL map d_o -> u
map_Tdou.varname_cell = {'Tdou'};
map_Tdou.engname = '';
map_Tdou.texname = '$T_{d_{o} u}$';
map_Tdou.filename = map_Tdou.varname_cell{1};

%   Tdoy                    CL map d_o -> y
map_Tdoy.varname_cell = {'Tdoy'};
map_Tdoy.engname = '';
map_Tdoy.texname = '$T_{d_{o} y}$';
map_Tdoy.filename = map_Tdoy.varname_cell{1};

%   Tdoyp                   CL map d_o -> y_p
map_Tdoyp.varname_cell = {'Tdoyp'};
map_Tdoyp.engname = '';
map_Tdoyp.texname = '$T{d_{o} y_{p}}$';
map_Tdoyp.filename = map_Tdoyp.varname_cell{1};

%   Tdoe                    CL map d_o -> e
map_Tdoe.varname_cell = {'Tdoe'};
map_Tdoe.engname = '';
map_Tdoe.texname = '$T_{d_{o} e}$';
map_Tdoe.filename = map_Tdoe.varname_cell{1};

%   Tdouo                   CL map d_o -> u_o
map_Tdouo.varname_cell = {'Tdouo'};
map_Tdouo.engname = '';
map_Tdouo.texname = '$T_{d_{o} u_{o}}$';
map_Tdouo.filename = map_Tdouo.varname_cell{1};

%   Tdoxr                   CL map d_o -> x_r
map_Tdoxr.varname_cell = {'Tdoxr'};
map_Tdoxr.engname = '';
map_Tdoxr.texname = '$T_{d_{o} x_{i}}$';
map_Tdoxr.filename = map_Tdoxr.varname_cell{1};

%   Tdoui                   CL map d_o -> u_i
map_Tdoui.varname_cell = {'Tdoui'};
map_Tdoui.engname = '';
map_Tdoui.texname = '$T_{d_{o} u_{i}}$';
map_Tdoui.filename = map_Tdoui.varname_cell{1};


% ***********************
%       
% INNER-LOOP NOISE n_i
%

%   Tniu                    CL map n_i -> u
map_Tniu.varname_cell = {'Tniu'};
map_Tniu.engname = '';
map_Tniu.texname = '$T_{n_{i} u}$';
map_Tniu.filename = map_Tniu.varname_cell{1};

%   Tniy                    CL map n_i -> y
map_Tniy.varname_cell = {'Tniy'};
map_Tniy.engname = '';
map_Tniy.texname = '$T_{n_{i} y}$';
map_Tniy.filename = map_Tniy.varname_cell{1};

%   Tnie                    CL map n_i -> e
map_Tnie.varname_cell = {'Tnie'};
map_Tnie.engname = '';
map_Tnie.texname = '$T_{n_{i} e}$';
map_Tnie.filename = map_Tnie.varname_cell{1};

%   Tniuo                   CL map n_i -> u_o
map_Tniuo.varname_cell = {'Tniuo'};
map_Tniuo.engname = '';
map_Tniuo.texname = '$T_{n_{i} u_{o}}$';
map_Tniuo.filename = map_Tniuo.varname_cell{1};

%   Tnixr                   CL map n_i -> x_r
map_Tnixr.varname_cell = {'Tnixr'};
map_Tnixr.engname = '';
map_Tnixr.texname = '$T_{n_{i} x_{i}}$';
map_Tnixr.filename = map_Tnixr.varname_cell{1};

%   Tniei                   CL map n_i -> e_i
map_Tniei.varname_cell = {'Tniei'};
map_Tniei.engname = '';
map_Tniei.texname = '$T_{n_{i} e_{i}}$';
map_Tniei.filename = map_Tniei.varname_cell{1};

%   Tniui                   CL map n_i -> u_i 
map_Tniui.varname_cell = {'Tniui'};
map_Tniui.engname = '';
map_Tniui.texname = '$T_{n_{i} u_{i}}$';
map_Tniui.filename = map_Tniui.varname_cell{1};

% ***********************
%       
% PRE-FILTER CL MAPS
%

%   WKSe                    (ss Obj.) CL map r -> u
map_WKSe.varname_cell = {'WKSe'};
map_WKSe.engname = 'Filt. $K$-Sen.';
map_WKSe.texname = '$W K S_{e}$';
map_WKSe.filename = map_WKSe.varname_cell{1};

%   WTe                     (ss Obj.) CL map r -> y
map_WTe.varname_cell = {'WTe'};
map_WTe.engname = 'Filt. Comp. Sen.';
map_WTe.texname = '$W T_{e}$';
map_WTe.filename = map_WTe.varname_cell{1};

%   WSe                     (ss Obj.) CL map r -> e
map_WSe.varname_cell = {'WSe'};
map_WSe.engname = 'Filt. Sensitivity';
map_WSe.texname = '$W S_{e}$';
map_WSe.filename = map_WSe.varname_cell{1};

%   WKoSe                   (ss Obj.) CL map r -> u_o
map_WKoSe.varname_cell = {'WKoSe'};
map_WKoSe.engname = 'Filt. $K_{o}$-Sen.';
map_WKoSe.texname = '$W K_{o} S_{e}$';
map_WKoSe.filename = map_WKoSe.varname_cell{1};

%   WTrxr                   (ss Obj.) CL map r -> x_r
map_WTrxr.varname_cell = {'WTrxr'};
map_WTrxr.engname = '';
map_WTrxr.texname = '$W T_{r x_{i}}$';
map_WTrxr.filename = map_WTrxr.varname_cell{1};

%   WTrui                   (ss Obj.) CL map r -> u_i
map_WTrui.varname_cell = {'WTrui'};
map_WTrui.engname = '';
map_WTrui.texname = '$W T_{r u_{i}}$';
map_WTrui.filename = map_WTrui.varname_cell{1};

% W S_e and W T_e
map_WSeWTe.varname_cell = [{'WSe'}; {'WTe'}];
map_WSeWTe.engname = 'Filt. Sen. and Comp. Sen. at Error';
map_WSeWTe.texname = '$W S_{e}$, $W T_{e}$';
map_WSeWTe.filename = 'WSe_WTe';


% *************************************************************************
% *************************************************************************
%
% MAP GROUPINGS
%
% *************************************************************************
% *************************************************************************

% P only
maps_P = {  map_P   };

% OL maps
maps_ol = { map_Lu
            map_Le  };

% S and T only
maps_S_T = {    map_Se
                map_Te
                map_SeTe
                map_Su
                map_Tu
                map_SuTu    };

% Error e
maps_e_basic = {    map_Te
                    map_Se
                    map_SeTe
                    map_KSe
                    map_KoSe   };
maps_e = [  maps_e_basic
            map_Trxr
            map_Trui    ];


% Controls u
maps_u_basic = {    map_Tu
                    map_Su
                    map_SuTu
                    map_PSu
                    map_KoPSu   };
maps_u = [  maps_u_basic
            map_Tdixr
            map_Tdiui    ];

% Output disturbance d_o
maps_do = { map_Tdou
            map_Tdoy
            map_Tdoyp
            map_Tdoe
            map_Tdouo
            map_Tdoxr
            map_Tdoui   };

% Inner-loop noise n_i
maps_ni = { map_Tniu
            map_Tniy
            map_Tnie
            map_Tniuo
            map_Tnixr
            map_Tniei
            map_Tniui   };

% Pre-filter maps
maps_w_basic = {    map_WTe
                    map_WSe
                    map_WSeWTe
                    map_WKSe
                    map_WKoSe   };
maps_w = [  maps_w_basic
            map_WTrxr
            map_WTrui   ];

% Error e, controls u -- basic
maps_eu_basic = [   maps_ol
                    maps_e_basic
                    maps_u_basic  ];

% Error e, controls u -- basic -- with P
maps_eu_basic_P = [ maps_P
                    maps_eu_basic  ];

% Error e, controls u
maps_eu = [ maps_ol
            maps_e
            maps_u  ];

% Error e, controls u, pre-filter maps -- basic 
maps_euw_basic = [  maps_eu_basic
                    maps_w_basic  ];

% Error e, controls u, pre-filter maps -- basic -- with P
maps_euw_basic_P = [    maps_P
                        maps_euw_basic  ];

% Error e, controls u, pre-filter maps 
maps_euw = [    maps_eu
                maps_w  ];

% All maps
maps_all = [    maps_euw
                maps_do
                maps_ni     ];

% ***********************
%       
% SELECT PLOT GROUP BASED ON USER SETTING
%

% Check if plotting plant frequency response only
isPonly = 0;

switch plotgroup

    case 'P'
        plot_cell = maps_P;
        isPonly = 1;
    case 'euw_basic'
        plot_cell = maps_euw_basic;
    case 'euw_basic_P'
        plot_cell = maps_euw_basic_P;        
    case 'euw'
        plot_cell = maps_euw;   
    case 'euw_P'
        plot_cell = maps_euw_P;         
    case 'S_T'
        plot_cell = maps_S_T;
    case 'all'
        plot_cell = maps_all;
    otherwise
        plot_cell = maps_eu_basic;

end

% Number of maps plotted
numplots = size(plot_cell,1);

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN PLOT LOOP   
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Contains whether to include the legend entry from preset i in plot j
inplot1not0 = zeros(numpresets,numplots);

% Entry (i, j) contains ||T_j||_{H^{\infty}} for preset i
hinfmat = zeros(numpresets,numplots); 

% Entry (i, j) contains frequency of ||T_j||_{H^{\infty}} for preset i
whinfmat = zeros(numpresets,numplots); 

for i = 1:numpresets

% Extract current preset
currpreset = sys_cell{i};

% Get open/closed loop maps
if ~isPonly
    cl_map_data = f_CLTFM_io(currpreset);
else 
    cl_map_data.P = currpreset.P;
end

% Reset figure counter
figcount = figcount_start;

% *************************************************************************
% *************************************************************************
%
% PLOT OPEN/CLOSED LOOP MAPS FOR CURRENT PRESET 
%
% *************************************************************************
% *************************************************************************

for j = 1:numplots

    % Get current map
    currplot = plot_cell{j};
    currengname = currplot.engname;
    currtexname = currplot.texname;
    currfilename = currplot.filename;

    % Get the current list of maps for this plot
    varname_cell = currplot.varname_cell;
    nummaps = size(varname_cell,1);

    % Initialize empty frequency, response vectors
    wwvec = [];
    respvec = [];

    % Temp variable for finding H-inf norm, ind
    hinftmp = - Inf;
    indhinftmp = 0;

    % ***********************
    %       
    % GET RESPONSE DATA FROM THIS PRESET
    %

    for k = 1:nummaps
        
        % Get current map
        currmapname = varname_cell{k};

        % See if current map is present for this preset
        docurrmap = isfield(cl_map_data,currmapname);

        % If current map found, calculate/include the response data
        if docurrmap
            
            % At least one of the maps in this plot is present in the
            % current preset. Flag this preset for later legend creation
            inplot1not0(i,j) = 1;

            % Get map
            currmap = cl_map_data.(currmapname);

            % Calculate response
            svs = sigma(currmap,wvec);
            svs_dB = 20 * log10(svs); 

            % Get the number of curves to add to this entry for this map
            numc = size(svs_dB,1);

            % Include each response in the plot for this entry
            for l = 1:numc

                % Current response
                cursvs_dB = svs_dB(l,:)';
                
                % Get H-inf norm, freq
                [hinfdB, indhinfdB] = max(cursvs_dB);

                % If H-inf norm exceeds current max, make it the new one
                if hinfdB > hinftmp
                    hinftmp = hinfdB;
                    indhinftmp = indhinfdB;
                end

                % Append a copy of the frequency vector
                wwvec = [wwvec; wvec];

                % Include this entry of the response
                respvec = [respvec; cursvs_dB];

                % If there is another curve to add after this one, append a
                % NaN to each of these vectors to avoid the plot function
                % connecing the disjoint points
                if l < numc
                    wwvec = [wwvec; nan];
                    respvec = [respvec; nan];
                end

            end

            % If more maps are to be plotted for this preset, append a NaN
            % to each of these vectors to avoid the plot function
            % connecting the disjoint points
            if k < nummaps
                wwvec = [wwvec; nan];
                respvec = [respvec; nan];
            end

        end

    end

    % ***********************
    %       
    % PLOT RESPONSE DATA, IF ANY
    %

    if inplot1not0(i,j)        
        figure(figcount);
        semilogx(wwvec,respvec);
        hold on;
    end

    % Set ||T_{j}||_{H^{\infty}} and corresponding freq for preset i
    hinfmat(i,j) = hinftmp;
    if indhinftmp > 0
        whinfmat(i,j) = wvec(indhinftmp);
    else
        whinfmat(i,j) = -1;
    end

    % ***********************
    %       
    % FORMAT PLOT IF THIS IS FINAL PRESET AND AT LEAST ONE PRESET IS IN THE
    % PLOT
    %
    if i == numpresets && sum(inplot1not0(:,j)) > 0

        % Title
        ttl = [currengname ' ' currtexname];
        if docaption
            ttl = [ttl caption];
        end
        title(ttl);

        % Axis labels
        xlabel(wlabel);
        ylabel(maglabel);

        % Get vector of which presets are currently in this plot
        inplotvec = logical(inplot1not0(:,j));

        % Legend
        if do_lgd
            % Get legend entries of presets which had responses for this
            % plot
            lgd_tmp = lgd(inplotvec);
            legend(lgd_tmp)
        end

        % Individual settings
        if do_indiv_sett
            % Get individual settings of presets which had responses for
            % this plot
            indiv_sett_cell_tmp = indiv_sett_cell(inplotvec);
            % Set this entry in the plot_settings cell array for
            % plot_format.m
            p_sett.indiv_sett_cell = indiv_sett_cell_tmp;
        end      

        % Custom plot settings for this plot
        if do_custom_sett
            % Check if this plot has custom settings
            if isfield(custom_sett_cell, currfilename)
                p_sett.custom_sett = custom_sett_cell.(currfilename);
            end
        end

        % Check for additons to make to the filename


        % Format plot
        p_sett.figcount = figcount;
        plot_format(p_sett); 
        clear p_sett;
        
        % SAVE PLOT
        if savefigs
            filename = [currfilename postfilename];
            savepdf(figcount, relpath, filename); 
        end

    end

    % Increment figure counter
    figcount = figcount + 1;

end             % END MAIN PLOT LOOP


end             % END MAIN PRESET LOOP


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SET OUTPUT DATA
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Plot data
out_data.plot_cell = plot_cell;

% H-inf data
out_data.hinfmat = hinfmat;
out_data.whinfmat = whinfmat;
