function wk = get_w(x, t, k, nom1sim0_1, nom1sim0_2)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALCULATE w_k = f_k(x) - A_{kk} x_k
%
% Brent Wallace
%
% 2023-03-13
%       
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% % Global variables
% global sys;
% n = sys.n;
% m = sys.m;

% Control settings
% global r_sett;
global u_sett;

% Get coordinate transformations
sx = u_sett.sx;
% sxh = u_sett.sxh;
% su = u_sett.su;
sy = u_sett.sy;
Sxirl_x3 = u_sett.Sxirl_x3;   % [z, y, x_r, x_3] -> [x_1, x_2, x_3]

% Indices of state vector partition
inds = u_sett.inds;

% Has integral augmentation (=1) or not (=0)
hasintaug = u_sett.hasintaug;


% % Get loop settings
% loop_cell = u_sett.loop_cell;
% numloops = u_sett.numloops;

% Get nominal, simulation model
if nom1sim0_1
    model_nom = u_sett.model_nom;
else
    model_nom = u_sett.model_sim;
end

if nom1sim0_2
    model_sim = u_sett.model_nom;
else
    model_sim = u_sett.model_sim;
end


% Extract plant states
xp = x(inds.indsx);

% Get equilibrium point x_e (pre-transformation)
if u_sett.wnom1sim0
    xe = model_nom.trimconds.xe;
else
    xe = model_sim.trimconds.xe;
end

% % Get equilibrium control u_e (pre-transformation)
% ue = model_sim.trimconds.ue_dE;

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(inds.indsxr);

% Evaluate drift dynamics
if u_sett.lin1nonlin0
    % System linear
    f_x = model_nom.lin.Ap * tx;
else
    % System nonlinear
    f_x = model_nom.fx(xp);
end

% Get y(t) (post-transformation)
yp = sy * x(inds.indsxr);

% Get (small-signal) output y (post-transformation)
typ = yp - yep;

% Transformed state derivative (with nominal drift dynamics)
% Coords: [y, x_r, x_3]
fxp = sx * f_x;

% Append integrator state derivatives
% Coords: [\int y, y, x_r, x_3]
if hasintaug
    fxp = [typ; fxp];
end

% Transform state derivatives [\int y, y, x_r, x_3] -> [x_1, x_2, x_3]
fxpirl = Sxirl_x3 * fxp;


% Get A_{kk}
Akk = model_nom.A_cell{k,k};

% Get x_k (post-trans)
xk = get_loop_state(x,t,k,xe);

% Get f_k(x) (post-trans)
fxpk = fxpirl(inds.indsxirl{k});

% Calculate w_k = f_k(x) - A_{kk} x_k
wk = fxpk - Akk * xk;



