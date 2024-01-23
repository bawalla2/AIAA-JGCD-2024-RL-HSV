function lq_data = lqr_init(alg_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PERFORM LQR DESIGN
%
% Brent Wallace  
%
% 2023-03-29
%
% This program performs an LQR design from input model data.
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
% *************************************************************************
%
% UNPACK SETTINGS/PARAMETERS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Plant state-space matrices
Ap = alg_settings.Ap;
Bp = alg_settings.Bp;
Cp = alg_settings.Cp;
Dp = alg_settings.Dp;

% Get system dimensions
n = size(Ap,1);
m = size(Bp,2);

% Get Q, R
Q = alg_settings.Q;
R = alg_settings.R;

% Get coord transformations
su = alg_settings.su;
sx = alg_settings.sx;
sy = alg_settings.sy;

% Data saving settings
savedata = isfield(alg_settings, 'relpath_data');

% If data is to be saved, extract the relative path and file name to save
% to
if savedata
    relpath_data = alg_settings.relpath_data;
    filename = alg_settings.filename;
end

%%
% *************************************************************************
% *************************************************************************
%
% FORM DESIGN PLANT
%
%
% Given an original plant of the form P = (A, B, C, D), we would like to
% form a design plant (appropriately scaled) such that the state vector
% assumes the form
%
% x' = [    y
%           x_r     ]
%
% Where y \in R^m are the outputs to be tracked, and x_r \in R^{n-m} forms
% the rest of the state.
%
% Introduce the coordinate transformations:
%
%  u' = S_u * u
%  x' = S_x * x
%  y' = S_y * y
%
% *************************************************************************
% *************************************************************************

% Scaled linear dynamics
Ad = sx * Ap * inv(sx);
Bd = sx * Bp * inv(su);
Cd = sy * Cp * inv(sx);
Dd = sy * Dp * inv(su);

% Scaled design plant
Pd = ss(Ad,Bd,Cd,Dd);

% Rest of state x_r = C_r x'
Cr = [zeros(n-m,m)  eye(n-m)];


% *************************************************************************
%       
% PERFORM LQR DESIGN
%  
% *************************************************************************

% Call LQR function
[K, P, ~] = lqr(Ad, Bd, Q, R);

% Extract parts of feedback gain matrix K corresponding to each of the
% states in the partition
Ky = K(:,1:m);
Kr = K(:,m+1:n);


% *************************************************************************
%       
% FORM OPEN-LOOP SYSTEM STATE SPACE REPRESENTATIONS
%  
% *************************************************************************


% ***********************
%       
% FORM LOOP BROKEN AT ERROR L_e
%
% State: [  x' ]
%
% Input: e
%
% Output: y
%  

% z = \int y
Ae = [  Ad - Bd*Kr*Cr ];
% z = \int y
Be = [  Bd*Ky   ];

Ce = [  Cd ];

De = zeros(m);

Le = ss(Ae,Be,Ce,De);



% ***********************
%       
% FORM LOOP BROKEN AT CONTROLS L_u = G_{LQ}
%
% State: [  z
%           x' ]
%
% Input: u_p
%
% Output: u
%  

Au = Ad;
Bu = Bd;
Cu = K;
Du = zeros(m);

Lu = ss(Au,Bu,Cu,Du);


% *************************************************************************
%       
% FORM CLOSED-LOOP SYSTEM STATE SPACE REPRESENTATIONS
%  
% *************************************************************************

% ***********************
%       
% FORM CLOSED-LOOP DYNAMICS BROKEN AT ERROR L_e
%

Aecl = Ae - Be * Ce;
Becl = Be;
Cecl = Ce;
Decl = De;

% Sensitivity at the error S_e
Se = ss(Aecl, Becl, -Cecl, eye(m) - Decl);

% Comp sensitivity at error T_e
Te = ss(Aecl, Becl, Cecl, Decl);


% ***********************
%       
% FORM CLOSED-LOOP DYNAMICS BROKEN AT CONTROL L_u
%

Aucl = Au - Bu * Cu;
Bucl = Bu;
Cucl = Cu;
Ducl = Du;

% Sensitivity at the control S_u
Su = ss(Aucl, Bucl, -Cucl, eye(m) - Ducl);

% Comp sensitivity at control T_u
Tu = ss(Aucl, Bucl, Cucl, Ducl);



%%
% *************************************************************************
% *************************************************************************
%
% PACK OUTPUT DATA
%
% *************************************************************************
% *************************************************************************

% Q, R
lq_data.Q = Q;
lq_data.R = R;



% ***********************
%       
% LQR DESIGN
%  

lq_data.P = P;

lq_data.K = K;
% lq_data.Kz = Kz;
lq_data.Ky = Ky;
lq_data.Kr = Kr;


% ***********************
%       
% LOOP BROKEN AT ERROR L_e
%  

lq_data.Ae = Ae;
lq_data.Be = Be;
lq_data.Ce = Ce;
lq_data.De = De;
lq_data.Le = Le;



% ***********************
%       
% LOOP BROKEN AT CONTROLS L_u
%  

lq_data.Au = Au;
lq_data.Bu = Bu;
lq_data.Cu = Cu;
lq_data.Du = Du;
lq_data.Lu = Lu;


% ***********************
%       
% CLOSED-LOOP DYNAMICS BROKEN AT ERROR L_e
%

lq_data.Aecl = Aecl;
lq_data.Becl = Becl;
lq_data.Cecl = Cecl;
lq_data.Decl = Decl;
lq_data.Se = Se;
lq_data.Te = Te;


% ***********************
%       
% CLOSED-LOOP DYNAMICS BROKEN AT CONTROL L_u
%

lq_data.Aucl = Aucl;
lq_data.Bucl = Bucl;
lq_data.Cucl = Cucl;
lq_data.Ducl = Ducl;
lq_data.Su = Su;
lq_data.Tu = Tu;



%%
% *************************************************************************
% *************************************************************************
%
% SAVE DATA
%
% *************************************************************************
% *************************************************************************


if savedata

    % Make directory to save data to
    mkdir(relpath_data)

    % Save data 
    varname = 'lq_data';
    save([relpath_data filename], varname);

end

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


