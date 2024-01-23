function lq_data = config_lq_servo_tito(alg_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% LQ-SERVO INNER-OUTER LOOP DESIGN
%
% Brent Wallace  
%
% 2023-01-19
%
% This program performs an LQ servo inner-outer design.
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
% LOAD NONLINEAR MODEL
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% System tag
sys = alg_settings.sys;


% *************************************************************************
% 
% UNPACK SETTINGS/PARAMETERS
% 
% *************************************************************************

% System
n = sys.n;                          % System order
nlq = sys.nlq;                      % System order -- LQ servo designs
m = sys.m;                          % System input dimension
% model = sys.model;                  % System model

% Data saving settings
savedata = isfield(alg_settings, 'relpath_data');

% If data is to be saved, extract the relative path and file name to save
% to
if savedata
    relpath_data = alg_settings.relpath_data;
    filename = alg_settings.filename;
end

% Get Q, R
Q1 = alg_settings.Q1;
R1 = alg_settings.R1;
Q2 = alg_settings.Q2;
R2 = alg_settings.R2;

% Has integral augmentation (=1) or not (=0)
hasintaug = alg_settings.hasintaug;

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% Design model
model_d = alg_settings.model_d;


% ***********************
%
% LINEARIZATION TERMS
%

% Linearization params
lin = model_d.lin;
io = lin.io;

% Scaled linear dynamics
Ad = io.Ad;
Bd = io.Bd;
Cd = io.Cd;
Dd = io.Dd;

% Plant (1,1)
Ad11 = io.Ad11;
Bd11 = io.Bd11;
Cd11 = io.Cd11;
Dd11 = io.Dd11;
Pd11 = io.Pd11;

% Plant (2,2)
Ad22 = io.Ad22;
Bd22 = io.Bd22;
Cd22 = io.Cd22;
Dd22 = io.Dd22;
Pd22 = io.Pd22;

% DIRL state transformation 
% x_{lq servo} -> x_{dirl}
% [z y x_{r}] -> [x_1 x_2]
Sxirl = io.Sxirl;


%%
% *************************************************************************
% *************************************************************************
%
% DESIGN -- P_{11}
%
% *************************************************************************
% *************************************************************************

% Coordinate transformations
alg_settings.su = eye(size(Bd11,2));
alg_settings.sx = eye(size(Ad11,1));
alg_settings.sy = eye(size(Cd11,1));

% Plant state-space
alg_settings.Ap = Ad11;
alg_settings.Bp = Bd11;
alg_settings.Cp = Cd11;
alg_settings.Dp = Dd11;

% Q, R
alg_settings.Q = Q1;
alg_settings.R = R1;

% Perform design
if hasintaug
    lq_data_11 = lq_servo_init(alg_settings);
else
    lq_data_11 = lqr_init(alg_settings);
end


%%
% *************************************************************************
% *************************************************************************
%
% DESIGN -- P_{22}
%
% *************************************************************************
% *************************************************************************

% Coordinate transformations
alg_settings.su = eye(size(Bd22,2));
alg_settings.sx = eye(size(Ad22,1));
alg_settings.sy = eye(size(Cd22,1));

% Plant state-space
alg_settings.Ap = Ad22;
alg_settings.Bp = Bd22;
alg_settings.Cp = Cd22;
alg_settings.Dp = Dd22;

% Q, R
alg_settings.Q = Q2;
alg_settings.R = R2;

% Perform design
if hasintaug
    lq_data_22 = lq_servo_init(alg_settings);
else
    lq_data_22 = lqr_init(alg_settings);
end


%%
% *************************************************************************
% *************************************************************************
%
% DESIGN -- OVERALL SYSTEM
%
% *************************************************************************
% *************************************************************************

% Coordinate transformations
alg_settings.su = eye(size(Bd,2));
alg_settings.sx = eye(size(Ad,1));
alg_settings.sy = eye(size(Cd,1));

% Plant state-space
alg_settings.Ap = Ad;
alg_settings.Bp = Bd;
alg_settings.Cp = Cd;
alg_settings.Dp = Dd;

% EQUIVALENT "Q", "R" MATRICES
% Note: Q = diag(Q_1, Q_2) is wrt the DIRL state partition. Need
% transformation to the LQ servo state partition
Qirl = [   Q1                              zeros(size(Q1,1), size(Q2,2))
            zeros(size(Q2,1), size(Q1,2))   Q2                           ];

% Q -- in LQ servo coords
Q = Sxirl' * Qirl * Sxirl;

% R
R = [   R1                              zeros(size(R1,1), size(R2,2))
        zeros(size(R2,1), size(R1,2))   R2                           ];


alg_settings.Q = Q;
alg_settings.R = R;


% Perform design
if hasintaug
    lq_data_c = lq_servo_init(alg_settings);
else
    lq_data_c = lqr_init(alg_settings);
end



% Extract controller
Kc = lq_data_c.K;


%%
% *************************************************************************
% *************************************************************************
%
% FORM COMPOSITE GAIN MATRIX
%
% *************************************************************************
% *************************************************************************


% Extract K_1 controller
K11 = lq_data_11.K;
if hasintaug
    Kz11 = lq_data_11.Kz;
end
Ky11 = lq_data_11.Ky;
Kr11 = lq_data_11.Kr;

% Extract K_2 controller
K22 = lq_data_22.K;
if hasintaug
    Kz22 = lq_data_22.Kz;
end
Ky22 = lq_data_22.Ky;
Kr22 = lq_data_22.Kr;

% % DEBUGGING: DDMR -- manually set controller gains to Kaustav's P-2 values
% % (cf. thesis pp. 148)
% g1P2 = 2;
% z1P2 = 1.79;
% g2P2 = 6.7;
% z2P2 = 3.2;
% Kz11 = g1P2 * z1P2;
% Ky11 = g1P2;
% % Kz22 = g2P2 * z2P2;
% % Ky22 = g2P2;
% K11 = blkdiag(Kz11,Ky11);
% % K22 = blkdiag(Kz22,Ky22);


% % DEBUGGING: Step response (1,1)
% if hasintaug
%     K11ss = ss(0,Kz11,1,Ky11);
% else
% 
% end
% T11 = feedback(series(K11ss,Pd11),1);
% figure(100)
% step(T11)
% 
% 
% % DEBUGGING: Step response (2,2)
% K22ss = ss(0,Kz22,1,Ky22);
% T22 = feedback(series(K22ss,Pd22),1);
% figure(101)
% step(T22)


% Form composite controller
if hasintaug
    Kz = blkdiag(Kz11, Kz22);
end
Ky = blkdiag(Ky11, Ky22);
Kr = blkdiag(Kr11, Kr22);
if hasintaug
    K = [Kz Ky  Kr];
else
    K = [Ky  Kr];
end


%%
% *************************************************************************
% *************************************************************************
%
% FORM COMPOSITE GAIN MATRIX -- IRL STATE PARTITION
%
% *************************************************************************
% *************************************************************************

% EQUIVALENT CONTROLLER -- COMPOSITE SYSTEM
Kcirl = Kc * inv(Sxirl);

% EQUIVALENT CONTROLLER -- DECENTRALIZED DESIGN
Kdirl = blkdiag(K11, K22);

% DEBUGGING: Plot CL freq resp
Cr = [zeros(nlq-m,m)  eye(nlq-m)];
Ai = zeros(m);
Bi = eye(m);
Ci = eye(m);
Di = zeros(m);
% Kz = Kc(:,1:m); Ky = Kc(:,m+1:2*m); Kr = Kc(:,2*m+1:end);
if hasintaug
    Ae = [  Ai      zeros(m,nlq)
            -Bd*Kz  Ad - Bd*Kr*Cr ];
    Be = [  -Bi
            Bd*Ky   ];
    Ce = [  zeros(m)    Cd];
else
    Ae = Ad - Bd*Kr*Cr;
    Be = Bd*Ky;
    Ce = Cd;
end
De = zeros(m);
Aecl = Ae - Be * Ce;
Becl = Be;
Cecl = Ce;
Decl = De;

Se = ss(Aecl, Becl, -Cecl, eye(m) - Decl);
Te = ss(Aecl, Becl, Cecl, Decl);

figure(100)
wvec = logspace(-2,1,500);
svs = sigma(Te,wvec);
svs_dB = 20 * log10(svs);
semilogx(wvec,svs_dB);

%%
% *************************************************************************
% *************************************************************************
%
% PREFILTER POLE LOCATIONS
%
% A prefilter is inserted before the reference command in each
% channel, of form
%
%   W_i(s) =    a_i
%             --------,         i = 1,...,m
%               s + a_i
%
% *************************************************************************
% *************************************************************************

if hasintaug

% Calculate zero locations automatically
g1 = Ky11;
z1 = Kz11 / g1;
g2 = Ky22;
z2 = Kz22 / g2;
pfavec = [z1; z2];


% % Make pre-filter V
% Aw1 = -z1;
% Bw1 = z1;
% Cw1 = 1;
% Dw1 = 0;
% W1 = ss(Aw1,Bw1,Cw1,Dw1);
% lq_data_11.W = W1;

% % Make pre-filter \gamma
% Aw2 = -z2;
% Bw2 = z2;
% Cw2 = 1;
% Dw2 = 0;
% W2 = ss(Aw2,Bw2,Cw2,Dw2);
% lq_data_22.W = W2;

% % Make pre-filter for total system
% Aw = [Aw1 0; 0 Aw2];
% Bw = [Bw1 0; 0 Bw2];
% Cw = [Cw1 0; 0 Cw2];
% Dw = [Dw1 0; 0 Dw2];
% W = ss(Aw,Bw,Cw,Dw);

end

%%
% *************************************************************************
% *************************************************************************
%
% PACK OUTPUT DATA
%
% *************************************************************************
% *************************************************************************

% Individual loop data
lq_data.lq_data_11 = lq_data_11;
lq_data.lq_data_22 = lq_data_22;
lq_data.lq_data_tot = lq_data_c;

% Composite Q, R
lq_data.Q = Q;
lq_data.R = R;

% Composite controller
lq_data.K = K;
if hasintaug
    lq_data.Kz = Kz;
end
lq_data.Ky = Ky;
lq_data.Kr = Kr;

% Composite controller -- with coupling
lq_data.Kc = Kc;

% Composite data -- IRL state partition
lq_data.Qirl = Qirl;
lq_data.Kcirl = Kcirl;
lq_data.Kdirl = Kdirl;

% Prefilter data
if hasintaug
lq_data.pfavec = pfavec;
% lq_data.W = W;
end


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


% % Display done
% disp('************************')
% disp('*')
% disp(['* LQ SERVO INNER/OUTER DESIGN COMPLETE'])
% disp('*')
% disp('************************')
% 
% % Display gains, zeros   
% if hasintaug
% disp(['g_{1} =  ' num2str(g1)])        
% disp(['z_{1} =  ' num2str(z1)])
% disp(['g_{2} =  ' num2str(g2)])
% disp(['z_{2} =  ' num2str(z2)])
% end

