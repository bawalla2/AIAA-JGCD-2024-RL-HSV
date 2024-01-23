function master_settings = config_controllers_hsv(master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIG LQ SERVO AND FBL CONTROLLERS
%
% Brent Wallace  
%
% 2023-04-13
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
model_cell = sys.model_cell;
numnu1 = master_settings.numnu1;
numnu2 = master_settings.numnu2;
nummodels = master_settings.nummodels;
indnom = sys.indnom;

% Controller initialization controls
init1_load0_lq = master_settings.init1_load0_lq;
init1_load0_fbl = master_settings.init1_load0_fbl;

% Relative path to controller data
relpath_lq = master_settings.relpath_lq;
relpath_fbl = master_settings.relpath_fbl;

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE CONTROLLERS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
%
% LQ SERVO
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% LQ SERVO INNER-OUTER DESIGN PARAMETERS
%
% NOTE: State partitioned as (see below)
%
%      = [  z
%           y
%           x_r ]
%

% V
Q1 = diag([1.5 5]);
R1 = 7.5;
% \gamma
Q2 = diag([100 150 0.5 0]);
R2 = 1;


% ***********************
%       
% LQ SERVO INNER-OUTER DESIGN PARAMETERS -- INITIAL STABILIZING CONTROLLER
%

% V
Q10 = diag([1 1]);
R10 = 12.5;
% \gamma
Q20 = diag([1 1 0 0]);
R20 = 0.025;


% ***********************
%
% LQ SERVO INNER/OUTER CONTROLLER
%   

% Relative path to controller params
relpath_ctrl_tmp = [relpath_lq];

% File name to save to
filename_tmp = 'lq_data_cell.mat';

% Cell array to contain LQ data
lq_data_cell = cell(numnu1,numnu2);

if init1_load0_lq

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.Q1 = Q1;
    alg_settings.R1 = R1;
    alg_settings.Q2 = Q2;
    alg_settings.R2 = R2;
    alg_settings.hasintaug = hasintaug;

    % Loop over model cell array
    for i = 1:numnu1
        for j = 1:numnu2

            % Get current model
            alg_settings.model_d = model_cell{i,j};
     
            % Initialize controller 
            lq_datai = config_lq_servo_tito(alg_settings);
    
            % Store lq_data in array
            lq_data_cell{i,j} = lq_datai;

        end
    end

    % Make directory to save lq_data cell to
    mkdir(relpath_ctrl_tmp);

    % Save data 
    varname = 'lq_data_cell';
    save([relpath_ctrl_tmp filename_tmp], varname);

end


% ***********************
%
% LQ SERVO INNER/OUTER CONTROLLER -- APPROXIMATE DECENTRALIZED DESIGN
%   

if init1_load0_lq

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.Q1 = Q1;
    alg_settings.R1 = R1;
    alg_settings.Q2 = Q2;
    alg_settings.R2 = R2;
    alg_settings.hasintaug = hasintaug;

    % Loop over model cell array
    for i = 1:numnu1
        for j = 1:numnu2

            % Get current model
            model_i = model_cell{i,j};
    
            % Modify model parameters for approximate decentralized design
            io = model_i.lin.io;
            io.Ad11 = io.AdTV;
            io.Bd11 = io.BdTV;
            io.Cd11 = io.CdTV;
            io.Dd11 = io.DdTV;
            io.Pd11 = io.PdTV;
            io.Ad22 = io.AdEg;
            io.Bd22 = io.BdEg;
            io.Cd22 = io.CdEg;
            io.Dd22 = io.DdEg;
            io.Pd22 = io.PdEg;
            model_i.lin.io = io;
    
            % Set model
            alg_settings.model_d = model_i;
     
            % Initialize controller 
            lq_datai_d = config_lq_servo_tito(alg_settings);
    
            % Get current lq_data and add on new fields
            lq_datai = lq_data_cell{i,j};
            lq_datai.lq_data_V = lq_datai_d.lq_data_11;
            lq_datai.lq_data_g = lq_datai_d.lq_data_22;
            lq_datai.lq_data_d = lq_datai_d;
    
            % Store lq_data in array
            lq_data_cell{i,j} = lq_datai;

        end
    end

    % Make directory to save lq_data cell to
    mkdir(relpath_ctrl_tmp);

    % Save data 
    varname = 'lq_data_cell';
    save([relpath_ctrl_tmp filename_tmp], varname);

end


% ***********************
%
% GET CONTROLLER -- LQ SERVO INNER/OUTER -- INITIAL STABILIZING CONTROLLER
%   

% File name to save to
filename_tmp0 = 'lq_data_0.mat';

if init1_load0_lq

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.Q1 = Q10;
    alg_settings.R1 = R10;
    alg_settings.Q2 = Q20;
    alg_settings.R2 = R20;
    alg_settings.hasintaug = hasintaug;

    % Get current model -- nominal model
    alg_settings.model_d = get_elt_multidim(model_cell,indnom);

    % Initialize controller 
    lq_data_0 = config_lq_servo_tito(alg_settings);

    % Save data 
    varname = 'lq_data_0';
    save([relpath_ctrl_tmp filename_tmp0], varname);

end

% Load controllers
data = load([relpath_ctrl_tmp filename_tmp]);
lq_data_cell = data.lq_data_cell;
      
% Load initial stabilizing controllers
data = load([relpath_ctrl_tmp filename_tmp0]);
lq_data_0 = data.lq_data_0;


% *************************************************************************
% *************************************************************************
%
% FBL
% 
% *************************************************************************
% *************************************************************************


% ***********************
%       
% FBL DESIGN PARAMETERS
%
% NOTE: (Q_1, R_1): (V, \delta_{T}) (ft/s, -)
%       (Q_2, R_2): (\gamma, \delta_{E}) (rad, rad)
%

Q1_fbl = diag([8.54e-4 0.34 0.86 47.93]);
R1_fbl = 0.89;
Q2_fbl = diag([0.5 0.3 1 0.5]);
R2_fbl = 0.35; 


% ***********************
%
% GET CONTROLLER -- FBL
%   

% Relative path to controller params
relpath_ctrl_tmp = relpath_fbl;

% File name to save to
filename_tmp = 'controller.mat';
if init1_load0_fbl

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.Q1 = Q1_fbl;
    alg_settings.R1 = R1_fbl;
    alg_settings.Q2 = Q2_fbl;
    alg_settings.R2 = R2_fbl;
    alg_settings.relpath_data = relpath_ctrl_tmp;
    alg_settings.filename = filename_tmp;
    
    % Initialize controller
    config_ndi_wang_stengel_2000_Vg(alg_settings);
    
end

% Load controller
data = load([relpath_ctrl_tmp filename_tmp]);
controller_fbl = data.controller;


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE DATA
% 
% *************************************************************************
% *************************************************************************       
% *************************************************************************

% ***********************
%
% LQ SERVO
%   

master_settings.lq_data_cell = lq_data_cell;

lq_data_nom = get_elt_multidim(lq_data_cell,indnom);
master_settings.pfavec = lq_data_nom.pfavec;

% ***********************
%
% LQ SERVO -- INITIAL STABILIZING CONTROLLER
%   

master_settings.lq_data_0 = lq_data_0;

% ***********************
%
% FBL
%   

master_settings.controller_fbl = controller_fbl;
