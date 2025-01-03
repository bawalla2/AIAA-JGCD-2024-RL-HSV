function config_ndi_wang_stengel_2000_Vg(alg_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% HSV FEEDBACK LINEARIZATION -- y = (V, \gamma)
%
% Brent Wallace  
%
% 2022-08-13
%
% INITIALIZE HSV PITCH-AXIS MODEL
%
% Brent Wallace  
%
% 2022-09-28
%
% This program performs (approximate) feedback linearization (FBL) of a HSV
% longitudinal model for the outputs y = (V, \gamma). All model data and
% the FBL framework is from:
%
%   Q. Wang and R. F. Stengel. "Robust nonlinear control of a hypersonic
%   aircraft." AIAA J. Guid., Contr., & Dyn., 23(4):577–585, July 2000
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% sys                       (Struct) HSV longitudinal model (see
%                           hsv_wang_stengel_2000_init.m for details).
% Q1                        (3 x 3 Matrix) State penalty matrix for the V
%                           loop. State partition: [z_V, V, \dot{V}], where
%                           z_V denotes the airspeed integrator state.
% R1                        (Scalar) Control penalty for the V loop (i.e.,
%                           on the throttle setting \delta_T).
% Q2                        (3 x 3 Matrix) State penalty matrix for the
%                           \gamma loop. State partition:
%                           [z_{\gamma}, \gamma, \dot{\gamma}], where
%                           z_{\gamma} denotes the altitude integrator
%                           state.
% R2                        (Scalar) Control penalty for the \gamma loop 
%                           (i.e., on the elevator setting \delta_E).
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% controller                (Struct) Contains FBL controller parameters. Is
%                           saved to the relative file path:
%                             'aircraft/hsv_wang_stengel_2000/ndi/data/Vg/'
%                           for subsequent loading.
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% 
% UNPACK SETTINGS/PARAMETERS
% 
% *************************************************************************

% System
sys = alg_settings.sys;             % System array
n = sys.n;                          % System order
m = sys.m;                          % System input dimension
model_cell = sys.model_cell;
nummodels = sys.nummodels;
indnom = sys.indnom;
model = get_elt_multidim(model_cell,indnom);    % Nominal system model


% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% % Trim AOA (deg)
% alpha_e = alg_settings.alpha_e;


% Data saving settings
savedata = isfield(alg_settings, 'relpath_data');

% If data is to be saved, extract the relative path and file name to save
% to
if savedata
    relpath_data = alg_settings.relpath_data;
    filename = alg_settings.filename;
end


% Controller parameters
Q1 = alg_settings.Q1;
R1 = alg_settings.R1;
Q2 = alg_settings.Q2;
R2 = alg_settings.R2;

% Coordinate transformations
su = model.lin.io.sud;
sx = model.lin.io.sxd;

% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% ***********************
%       
% DEFINE SYMBOLIC VARIABLES
%      

% x_v = [V, \gamma, h, \alpha, q, \delta_{T}, \dot{\delta}_{T}] in R^{7}
xvs = sym('xv', [1 7], 'real')';

% u in R^{m}
us = sym('u', [1 m], 'real')'; 

% \nu = [\nu_{C_L}, \nu_{C_D}, \nu_{C_{M_\alpha}}]
n_nu = 3;
nus = sym('nu', [1 n_nu], 'real')'; 

% [x_v, u] in R^{7+m}
xvus = [    xvs
            us   ];

% [x_v, u, \nu] in R^{7+m+n_\nu}
xvunus = [  xvs
            us
            nus  ];

% Symbolic variable xv with nominal value of \nu
xvnunoms = [ xvs
             zeros(m,1)
             ones(n_nu,1) ];

% Index variables (i.e., where each state is in x)
indV = 1;
indg = 2;
indh = 3;
inda = 4;
indq = 5;
inddT = 6;
indddT = 7;
% indVI = 8;
% indhI = 9;

% Index variables (i.e., where each control is in [xv u])
inddTc = 8;
inddE = 9;

% Index variables (i.e., where each \nu is in [xv u \nu])
indnustart = inddE;
nunomvec = ones(n_nu,1);
indnuCL = indnustart + 1;
indnuCD = indnustart + 2;
indnuCMa = indnustart + 3;

% Ordered indices corresponding to
% z = [V, \gamma, \alpha, \delta_{T}, h] in R^{5}
% Cf. Wang, Eqn. (39)
indsz = [   indV
            indg
            inda
            inddT
            indh    ];



% *************************************************************************
%
% COMMON SYMBOLIC FUNCTIONS
% 
% *************************************************************************


% sin(\alpha), cos(\alpha)
sa(xvunus) = sin(xvunus(inda));
ca(xvunus) = cos(xvunus(inda));

% sin(\gamma), cos(\gamma)
sg(xvunus) = sin(xvunus(indg));
cg(xvunus) = cos(xvunus(indg));

% sin(\alpha+\gamma), cos(\alpha+\gamma)
sapg(xvunus) = sin(xvunus(inda) + xvunus(indg));
capg(xvunus) = cos(xvunus(inda) + xvunus(indg));


% *************************************************************************
%
% EXTRACT SYSTEM PARAMETERS
% 
% *************************************************************************

% ***********************
%       
% SYSTEM PARAMETERS
%      

g = model.g;
mu = model.mu;
RE = model.RE;
mref = model.mref;
wref = model.wref;
Iyy = model.Iyy;
S = model.S;
cbar = model.cbar;

k1 = model.k1;
k2 = model.k2;
k3 = model.k3;


% ***********************
%       
% AERO FUNCTIONS
%    

r = model.r;
rho = model.rho;
a = model.a;
M = model.M;

CL = model.CL;
CD = model.CD;

c_CMdE = model.c_CMdE;
CMdEu = model.CMdEu;
CMdE0 = model.CMdE0;
CMdE = model.CMdE;

kxv = model.kxv;
CT = model.CT;

qinf = model.qinf;

T = model.T;
L = model.L;
D = model.D;

Myy0 = model.Myy0;
Myyu = model.Myyu;
Myy = model.Myy;

% ELEVATOR-INDUCED LIFT EFFECTS
LdE0 = model.LdE0;
LdEu = model.LdEu;
LdE = model.LdE;
dgudE = model.dgudE;


% *************************************************************************
%
% PARTIAL DERIVATIVES OF AERO FUNCTIONS
% 
% *************************************************************************

% ***********************
%       
% THRUST T
%      

% Jacobian
dTdz = jacobian(T);
fdTdz = formula(dTdz);

% % Hessian
% d2Tdz2 = jacobian(dTdz);
% fd2Tdz2 = formula(d2Tdz2);

% Extract terms from Jacobian
T_V = fdTdz(indV);
T_a = fdTdz(inda);
T_dT = fdTdz(inddT);
T_h = fdTdz(indh);

% % Extract terms from Hessian
% T_VV = fd2Tdz2(indV,indV);
% T_Va = fd2Tdz2(indV,inda);
% T_VdT = fd2Tdz2(indV,inddT);
% T_Vh = fd2Tdz2(indV,indh);
% T_aa = fd2Tdz2(inda,inda);
% T_adT = fd2Tdz2(inda,inddT);
% T_ah = fd2Tdz2(inda,indh);
% T_dTdT = fd2Tdz2(inddT,inddT);
% T_dTh = fd2Tdz2(inddT,indh);
% T_hh = fd2Tdz2(indh,indh);


% ***********************
%       
% LIFT L
%      

% Jacobian
dLdz = jacobian(L);
fdLdz = formula(dLdz);

% % Hessian
% d2Ldz2 = jacobian(dLdz);
% fd2Ldz2 = formula(d2Ldz2);

% Extract terms from Jacobian
L_V = fdLdz(indV);
L_a = fdLdz(inda);
L_h = fdLdz(indh);

% % Extract terms from Hessian
% L_VV = fd2Ldz2(indV,indV);
% L_Va = fd2Ldz2(indV,inda);
% L_Vh = fd2Ldz2(indV,indh);
% L_aa = fd2Ldz2(inda,inda);
% L_ah = fd2Ldz2(inda,indh);
% L_hh = fd2Ldz2(indh,indh);

% ***********************
%       
% DRAG D
%      

% Jacobian
dDdz = jacobian(D);
fdDdz = formula(dDdz);

% % Hessian
% d2Ddz2 = jacobian(dDdz);
% fd2Ddz2 = formula(d2Ddz2);

% Extract terms from Jacobian
D_V = fdDdz(indV);
D_a = fdDdz(inda);
D_h = fdDdz(indh);

% % Extract terms from Hessian
% D_VV = fd2Ddz2(indV,indV);
% D_Va = fd2Ddz2(indV,inda);
% D_Vh = fd2Ddz2(indV,indh);
% D_aa = fd2Ddz2(inda,inda);
% D_ah = fd2Ddz2(inda,indh);
% D_hh = fd2Ddz2(indh,indh);


% *************************************************************************
%
% NDI -- DERIVATIVE TERMS
%
% Cf. Wang, Stengel (2000), Appendix B
% 
% *************************************************************************

% ***********************
%       
% \omega_{1}
%
% Cf. Wang, Eqn. (B1)
%      

omega1(xvunus) = [ T_V * ca - D_V
                - mref * mu * cg / r^2
                T_a * ca - T * sa - D_a
                T_dT * ca
                T_h * ca - D_h + 2 * mref * mu * sg / r^3     ];

% ***********************
%       
% \Omega_{2}
%
% Cf. Wang, Eqn. (B2)
%    

Omega2(xvunus) = jacobian(omega1, xvunus(indsz))';

% Omega2 = simplify(Omega2);


% ***********************
%       
% \pi_{1}
%
% Cf. Wang, Eqn. (B8)
% 

pi1(xvunus) = [    (L_V + T_V*sa)/(mref*xvunus(indV)) - ...
        (L + T*sa)/(mref*xvunus(indV)^2) + mu*cg/(xvunus(indV)^2*r^2) + cg/r
                mu*sg/(xvunus(indV)*r^2) - xvunus(indV)*sg/r
                (L_a + T_a*sa + T*ca)/(mref*xvunus(indV))
                T_dT*sa/(mref*xvunus(indV))
                (L_h + T_h*sa)/(mref*xvunus(indV)) +...
        (2*mu*cg)/(xvunus(indV)*r^3) -...
        (xvunus(indV)*cg)/r^2                                    ];


% ***********************
%       
% \Xi_{2}
%
% Cf. Wang, Eqn. (B9)
%    

Xi2(xvunus) = jacobian(pi1, xvunus(indsz))';

% Xi2 = simplify(Xi2);

% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS
%
% Cf. Wang, Stengel (2000), Eqns. (20)-(24), (35), (52)
% 
% *************************************************************************

% ***********************
%       
% FIRST DERIVATIVES
%    

% \dot{V}
% Cf. Wang, Eqn. (20)
dV(xvunus) = (T*ca - D)/mref - mu*sg/r^2;

% \dot{\gamma}
% Cf. Wang, Eqn. (21)
dg(xvunus) = (L + T*sa) / (mref*xvunus(indV)) ...
    - ((mu-xvunus(indV)^2*r)*cg) / (xvunus(indV)*r^2);

% % DEBUGGING: \dot{\gamma} terms
% dgr(xvunus) = ((mu-xvunus(indV)^2*r)*cg) / (xvunus(indV)*r^2);
% dgre = double(subs(dgr,xvunus,xve))

% \dot{h}
% Cf. Wang, Eqn. (22)
dh(xvunus) = xvunus(indV)*sg;

% \dot{\alpha}
% Cf. Wang, Eqn. (23)
da(xvunus) = xvunus(indq) - dg;

% \dot{q}
% Cf. Wang, Eqn. (24)
dq0(xvunus) = Myy0 / Iyy;
% dq(xvunus) = Myy / Iyy;

% \dot{z}
% z = [V, \gamma, \alpha, \delta_{T}, h] in R^{5}
dz(xvunus) = [ dV
            dg
            da
            xvunus(indddT)
            dh  ];

% ***********************
%       
% SECOND DERIVATIVES
%    

% \ddot{V}
% Cf. Wang, Eqn. (40)
ddV(xvunus) = 1 / mref * omega1' * dz;

% \ddot{\gamma}
% Cf. Wang, Eqn. (42)
ddg(xvunus) = pi1' * dz;

% \ddot{h}
% Cf. Wang, Eqn. (41)
ddh(xvunus) = dV * sg + xvunus(indV) * dg * cg;

% \ddot{\alpha}
% Cf. Wang, Eqns. (23), (43), (45)
dda0(xvunus) = Myy0 / Iyy - ddg;
ddadE(xvunus) = Myyu / Iyy;
% dda(xvunus) = dq - ddg;

% \ddot{\delta_{T}}
% Cf. Wang, Eqn. (35)
dddT0(xvunus) = k1 * xvunus(indddT) + k2 * xvunus(inddT);
% dddTc(xvunus) = k3;
dddTc = k3;
% dddT(xvunus) = dddT0 + dddTc * xvunus(inddTc);

% \ddot{z}
% z = [V, \gamma, \alpha, \delta_{T}, h] in R^{5}
% Cf. Wang, Eqn. (45)
ddz0(xvunus) = [   ddV
                ddg
                dda0
                dddT0
                ddh     ];

% ddzu(xvunus) = [   0       0
%                 0       0
%                 0       ddadE
%                 dddTc   0
%                 0       0       ];

% ddz(xvunus) = ddz0 + ddzu * [  xvunus(inddTc)
%                             xvunus(inddE) ];

% ***********************
%       
% THIRD DERIVATIVES
%    

% V^{(3)} with u = 0
% Cf. Wang, Eqns. (40), (47)
dddV0(xvunus) = 1/mref * omega1'*ddz0 + 1/mref*dz'*Omega2*dz;

% \gamma^{(3)} with u = 0
% Cf. Wang, Eqns. (42), (47)
dddg0(xvunus) = pi1'*ddz0 + dz'*Xi2*dz;

% h^{(3)}
% Cf. Wang, Eqn. (41)
dddh(xvunus) = ddV*sg + 2*dV*dg*cg - xvunus(indV)*dg^2*sg + xvunus(indV)*ddg*cg;

% % ***********************
% %       
% % FOURTH DERIVATIVES
% % 
% % NOTE: Not needed for y = [V, \gamma]^T
% %    
% 
% % h^{(4)} with u = 0
% % Cf. Wang, Eqns. (41), (47)
% ddddh0(xvunus) = 3*ddV*dg*cg - 3*dV*dg^2*sg + 3*dV*ddg*cg ...
%     - 3*xvunus(indV)*dg*ddg*sg - xvunus(indV)*dg^3*cg + dddV0*sg ...
%     + xvunus(indV)*cg*(pi1'*ddz0 + dz'*Xi2*dz);


% *************************************************************************
%
% NDI -- OUTPUT DYNAMICAL EQUATIONS
%
% Terms f^*(x), G^*(x)
% Cf. Wang, Eqns. (47), (48)
% 
% *************************************************************************

% f^*(x)
% Cf. Wang, Eqn. (47)
fstar(xvunus) = [  dddV0
                dddg0  ];

% G^*(x) -- y = [V, \gamma]^T
% Cf. Wang, Eqn. (48)
Gstar11(xvunus) = T_dT * ca / mref * dddTc;
Gstar12(xvunus) = (T_a * ca - T * sa - D_a) / mref * ddadE;
Gstar21(xvunus) = T_dT * sa / (mref * xvunus(indV)) * dddTc;
Gstar22(xvunus) = (L_a + T_a * sa + T * ca) / (mref * xvunus(indV)) * ddadE;
% Gstar21(xvunus) = T_dT * sapg / mref * dddTc;
% Gstar22(xvunus) = (T*capg + T_a*sapg + L_a*cg - D_a*sg) / mref * ddadE;

Gstar(xvunus) = [  Gstar11     Gstar12
                Gstar21     Gstar22     ];

% det(G^*(x))
% Cf. Wang, Eqn. (49)
detGstaru = T_dT*dddTc*ddadE;
detGstaraero = T + L_a*ca + D_a*sa;
detGstar = (detGstaru / (mref^2*xvunus(indV))) * detGstaraero;

% {G^*}^{-1}(x)
invGstar(xvunus) = 1/detGstar * [  Gstar22     -Gstar12
                                -Gstar21    Gstar11     ];
invGstar = simplify(invGstar);

% *************************************************************************
%
% NDI -- LINEAR CONTROLLER
%
% Cf. Wang, Eqns. (52)-(63)
% 
% *************************************************************************

% Integrator chain corresponding to V (A_1, b_1)
% Cf. Wang, Eqn. (54)
r_1 = 3;
A1 = [  zeros(r_1,1)  eye(r_1)
        0           zeros(1,r_1)  ];
b1 = zeros(r_1+1,1);
b1(end) = 1;

% Integrator chain corresponding to \gamma (A_2, b_2)
% Cf. Wang, Eqn. (56)
r_2 = 3;
A2 = [  zeros(r_2,1)  eye(r_2)
        0           zeros(1,r_2)  ];
b2 = zeros(r_2+1,1);
b2(end) = 1;

% Riccati equation solution P_1 = P_1^T > 0 \in R^{4 x 4}
% And LQR full-state feedback control gain matrix K_1 = r_1^{-1} b_1^T P_1
% \in R^{1 x 4}
% Cf. Wang, Eqn. (60)
[K1, P1] = lqr(A1,b1,Q1,R1);

% Riccati equation solution P_2 = P_2^T > 0 \in R^{5 x 5}
% And LQR full-state feedback control gain matrix K_2 = r_2^{-1} b_2^T P_2
% \in R^{1 x 4}
% Cf. Wang, Eqn. (60)
[K2, P2] = lqr(A2,b2,Q2,R2);

% *************************************************************************
% *************************************************************************
%
% DEBUGGING: PLOT STEP RESPONSES
% 
% *************************************************************************
% *************************************************************************

do_step = 1;

if do_step

tvecV = 0:0.1:100;
tvecg = 0:0.1:20;

% Ref for step response
rvecV = ones(1,length(tvecV));
rvecg = ones(1,length(tvecg));

% V integrator chain ss
CV = [0 1 zeros(1,r_1-1)]; 

% Closed-loop system r -> V
AclV = A1-b1*K1;
BclV = [-1; zeros(r_1,1)];
BclV(end) = K1(2);
clsV = ss(AclV,BclV,CV,0);

% Prefilter ss V
zV = 0.059260117198916;
AwV = -zV;
BwV = zV;
CwV = 1;
DwV = 0;
WV = ss(AwV,BwV,CwV,DwV);

% W T V
AWclV = [   AclV            BclV*CwV
            zeros(1,r_1+1)    AwV         ];
BWclV = [   BclV*DwV
            BwV         ];
CWclV = [CV     0];
DWclV = 0;
WclsV = ss(AWclV,BWclV,CWclV,DWclV);

% \gamma integrator chain ss
Cg = [0 1 zeros(1,r_2-1)]; 

% Closed-loop system \gammma
Aclg = A2-b2*K2;
Bclg = [-1; zeros(r_2,1)];
Bclg(end) = K2(2);
clsg = ss(Aclg,Bclg,Cg,0);

% Prefilter ss \gamma
zg = 0.379057462775670;
Awg = -zg;
Bwg = zg;
Cwg = 1;
Dwg = 0;
Wg = ss(Awg,Bwg,Cwg,Dwg);

% W T \gamma
AWclg = [   Aclg            Bclg*Cwg
            zeros(1,r_2+1)    Awg         ];
BWclg = [   Bclg*Dwg
            Bwg         ];
CWclg = [Cg     0];
DWclg = 0;
Wclsg = ss(AWclg,BWclg,CWclg,DWclg);


% Step response -- V
figure(100)
stepV = lsim(clsV,rvecV,tvecV);
plot(tvecV,stepV);
title('Step Response Feedback Linearized System -- V')
grid on;

% Filtered Step response -- V
figure(101)
stepWV = lsim(WclsV,rvecV,tvecV);
steprV = lsim(WV,rvecV,tvecV);
plot(tvecV,stepWV);
hold on
plot(tvecV,steprV);
title('Filt. Step Response Feedback Linearized System -- V')
grid on;
legend('V(t)', 'V_r(t)')

% Step response -- \gamma
figure(102)
stepg = lsim(clsg,rvecg,tvecg);
plot(tvecg,stepg);
title('Step Response Feedback Linearized System -- \gamma')
grid on;

% Filtered Step response -- \gamma
figure(103)
stepWg = lsim(Wclsg,rvecg,tvecg);
steprg = lsim(Wg,rvecg,tvecg);
plot(tvecg,stepWg);
hold on
plot(tvecg,steprg);
title('Filt. Step Response Feedback Linearized System -- \gamma')
grid on;
legend('\gamma(t)', '\gamma_r(t)')

end


% *************************************************************************
% *************************************************************************
%
% CREATE INLINE FUNCTIONS
% 
% *************************************************************************
% *************************************************************************

% Derivatives of V: [\dot{V}, \ddot{V}]
diffV(xvunus) = [  dV
                ddV     ];
tmp(xvs) = subs(diffV,xvunus,xvnunoms);
diffVil = matlabFunction(tmp, 'vars', {xvs});

% Derivatives of \gamma: [\dot{\gamma}, \ddot{\gamma}]
diffg(xvunus) = [  dg
                ddg    ];
tmp(xvs) = subs(diffg,xvunus,xvnunoms);
diffgil = matlabFunction(tmp, 'vars', {xvs});

% % Derivatives of h: [\dot{h}, \ddot{h}, h^{(3)}]
% diffh(xvunus) = [  dh
%                 ddh
%                 dddh    ];
% diffhil = matlabFunction(diffh, 'vars', {xvunus});

% f^*(x)
% Cf. Wang, Eqn. (47)
tmp(xvs) = subs(fstar,xvunus,xvnunoms);
fstaril = matlabFunction(tmp, 'vars', {xvs});

% G^*(x)
tmp(xvs) = subs(Gstar,xvunus,xvnunoms);
Gstaril = matlabFunction(tmp, 'vars', {xvs});

% {G^*}^{-1}(x)
invGstaril = matlabFunction(invGstar, 'vars', {xvunus});

% det(G*)
detGstaril = matlabFunction(detGstar, 'vars', {xvunus});
% Term in det(G*) -- Terms pertaining to input gains
detGstaruil = matlabFunction(detGstaru, 'vars', {xvunus});
% Term in det(G*) -- T + L_\alpha cos(\alpha) + D_\alpha sin(\alpha)
detGstaraeroil = matlabFunction(detGstaraero, 'vars', {xvunus});



% *************************************************************************
% *************************************************************************
%
% STORE DATA
% 
% *************************************************************************
% *************************************************************************

% Controller LQR weight matrices
controller.Q1 = Q1;
controller.R1 = R1;
controller.Q2 = Q2;
controller.R2 = R2;

% Controller LQR feedback gain matrices
controller.K1 = K1;
controller.K2 = K2;

% Derivatives of V: [\dot{V}, \ddot{V}]
controller.diffVil = diffVil;

% Derivatives of \gamma: [\dot{\gamma}, \ddot{\gamma}, \gamma^{(3)}]
controller.diffgil = diffgil;

% % Derivatives of h: [\dot{h}, \ddot{h}, h^{(3)}]
% controller.diffhil = diffhil;

% f^*(x)
% Cf. Wang, Eqn. (47)
controller.fstaril = fstaril;

% G^*(x)
controller.Gstaril = Gstaril;

% {G^*}^{-1}(x)
controller.invGstaril = invGstaril;

% det(G*)
controller.detGstaril = detGstaril;
% Term in det(G*) -- Terms pertaining to input gains
controller.detGstaruil = detGstaruil;
% Term in det(G*) -- T + L_\alpha cos(\alpha) + D_\alpha sin(\alpha)
controller.detGstaraeroil = detGstaraeroil;


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
    varname = 'controller';
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


% Display done
disp('************************')
disp('*')
disp(['* FBL DESIGN COMPLETE'])
disp('*')
disp('************************')






