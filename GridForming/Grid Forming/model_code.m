clc;close;clear;

T_s = 100e-6;
S_b = 5e6;
P_b = S_b;
Q_b = 0.4*S_b;
V_b = 690;
I_b=S_b/(sqrt(3)*V_b);

f_b = 50;
w_b = f_b*2*pi;


Z_b = V_b^2/S_b;
C_b = 1/(Z_b*w_b);
L_b = Z_b/w_b;

V_m = sqrt(2/3)*V_b;
V_dc = 3*V_m;



%% LCL Filter
I_max = (P_b*sqrt(2))/(3*V_b);
delta_IL_max = (0.1)*I_max;
f_sw = 5e3;


L_f = V_dc/(24*f_sw*delta_IL_max); %LCL filter first Inductor

C_f = (0.066*C_b);                  %LCL filter Capacitor

L_g = 1*L_f; % LCL Filter second inductor

L_f1 = 1.35e-3;
L_g1 = 0.35e-3;
C_f1 = 50e-6;

w_res = sqrt((L_f+L_g)/(L_f*L_g*C_f));
w_res2 = sqrt((L_f1+L_g1)/(L_f1*L_g1*C_f1));
f_res = w_res/(2*pi);
f_res2 = w_res2/(2*pi);

R_f = 0.005*Z_b;
R_g = 0.005*Z_b;
R_d = 0.2;

%% Droop
% mp = ((1.04*w_b)-(0.96*w_b))/S_b;

mp = 0.01*50*2*pi/(S_b); % 0.1 Hz/MW

w_c = 0.1*w_b;

% nq = ((1.2*V_m)-(0.8*V_m))/S_b;

nq = 0.1*V_m/(S_b); %  kV/MVar
%% PI
ti = 0.3e-3;
tiv = 10*ti;
z_damp = 0.707;

w_nc = 2*f_sw/10;

z = 224;
wmc = sqrt(z*ti^-1);
% 
% Kpc = 10*(2*z_damp*L_f*(w_nc/w_b)-R_f);
% Kic = Kpc*w_nc/(2*z_damp-(R_f*w_b)/(L_f*w_nc));

% Kpc = L_f/ti;
% Kic = R_f/ti;

Kpc = L_f/ti;
Kic = R_f/ti;
% Kpv = C_f*w_res;
Kiv = R_f/tiv;


% w_ni = f_sw*2*pi;
% w_nv = f_sw*2*pi/20;
% 
% Kpc = 2*z_damp*w_ni*L_f-(R_f);
% Kic = L_f*w_ni^2;
% 
% Kpv = 2*z_damp*C_f*w_nv;
% Kiv = C_f*w_nv^2;

fbv = 1/tiv;
wb = 2*pi*fbv;
% 
wcf = 2*pi*f_sw/10;
wz = wcf*0.1;


% Kpc = L_f/(ti);
% Kic = 0.1*fb*Kpc;
% Kic = R_f/(ti);
% Kic = 0.8;

zz = 135;

% zz = 0.15/(1.850*ti);
wm = sqrt(zz*fbv);
% 
Kpv = C_f*wm;
% Kiv = Kic;
% Kpv = ((w_res^2-wcf^2)*wcf)/(w_res^2*(Kpc*wcf+Kic));
% Kiv = wz*Kpv*0.1;
% Kiv = 227;
% Kiv = Kpv*wz*0.1;
% Kiv = r_Lf2/(ti);

% Kpv = C_f*w_res;
% Kiv = R_f/ti;
% 
% Kpc = L_f/ti;
% Kic = R_f/ti;
% 
% Kpv = 0.03;
% Kiv = 2.1;
% 
% Kpc = 0.62;
% Kic = 477.5;

% Kpv = 1;
% Kiv = 1;
% 
% Kpc = 1;
% Kic = 1;
%% Active Droop Model Validation

% Initial Conditions for frequency response analysis
I_o_d = 6200;
I_o_q = 0;
V_o_d = V_m;
V_o_q = 0;
V_bus_d = V_m;
V_bus_q = 0;
delta_phi_0 = 1.9;
r_Lf = R_f;
r_Lg = R_g;
w_0 = w_b;
% delta_phi_0 = 0;

A_P = [0    -mp     0;
       0    -w_c    0;
       0    0       -w_c];

B_P = [0    0   0   0   0   0;
       0    0   w_c*I_o_d   w_c*I_o_q   w_c*V_o_d   w_c*V_o_q;
       0    0   w_c*I_o_q   -w_c*I_o_d  -w_c*V_o_q  w_c*V_o_d];

C_P_w_v = [0    -mp     0;
           0    0       -nq;
           0    0       0];

C_P_v = [0    0       -nq;
         0    0       0];

C_P_w = [0      -mp      0];

D_P = zeros(3,6);


%Phi is the angle of the inverter reference frame seen from an arbitrary
%common reference in which differentiation of phi = -mp*P_filtered
%State space model of Power Controller
inputs_power = {'i_L_d','i_L_2','v_o_d','v_o_q','i_o_d','i_o_q'};
states_power = {'phi','P','Q'}; % P and Q is filtered (P_filtered)
outputs_power = {'w','v_o_d_ref','v_o_q_ref'};

linModel_Power = ss(A_P,B_P,C_P_w_v,D_P, 'statename', states_power, 'Inputname', inputs_power, 'outputname', outputs_power);
tfModel_Power = tf(linModel_Power);

%Open Loop Dynamics
%From input "i_L_d" 
i_L_d_2_w = tf(linModel_Power(1,1)); %to output "w ...."
i_L_d_2_v_o_d_ref = tf(linModel_Power(2,1)); %to output "v_o_d_ref ...."
i_L_d_2_v_o_q_ref = tf(linModel_Power(3,1)); %to output "v_o_q_ref ...."
%From input "i_L_q" 
i_L_q_2_w = tf(linModel_Power(1,2)); %to output "w ...."
i_L_q_2_v_o_d_ref = tf(linModel_Power(2,2)); %to output "v_o_d_ref ...."
i_L_q_2_v_o_q_ref = tf(linModel_Power(3,2)); %to output "v_o_q_ref ...."
%From input "v_o_d" 
v_o_d_2_w = tf(linModel_Power(1,3)); %to output "w ...."
v_o_d_2_v_o_d_ref = tf(linModel_Power(2,3)); %to output "v_o_d_ref ...."
v_o_d_2_v_o_q_ref = tf(linModel_Power(3,3)); %to output "v_o_q_ref ...."
%From input "v_o_q" 
v_o_q_2_w = tf(linModel_Power(1,4)); %to output "w ...."
v_o_q_2_v_o_d_ref = tf(linModel_Power(2,4)); %to output "v_o_d_ref ...."
v_o_q_2_v_o_q_ref = tf(linModel_Power(3,4)); %to output "v_o_q_ref ...."
%From input "i_o_d" 
i_o_d_2_w = tf(linModel_Power(1,5)); %to output "w ...."
i_o_d_2_v_o_d_ref = tf(linModel_Power(2,5)); %to output "v_o_d_ref ...."
i_o_d_2_v_o_q_ref = tf(linModel_Power(3,5)); %to output "v_o_q_ref ...."
%From input "i_o_q"
i_o_q_2_w = tf(linModel_Power(1,6)); %to output "w ...."
i_o_q_2_v_o_d_ref = tf(linModel_Power(2,6)); %to output "v_o_d_ref ...."
i_o_q_2_v_o_q_ref = tf(linModel_Power(3,6)); %to output "v_o_q_ref ...."

polesofv_o_d_2_w = pole(v_o_d_2_w);
zeroofv_o_d_2_w= zero(v_o_d_2_w);
pzmap(v_o_d_2_w);
damp(v_o_d_2_w);
bode(v_o_d_2_w);

%% Voltage Controller

K_iv = Kiv;
K_pv = Kpv;
F = 1;

A_V = zeros(4,4);


B_V1 = [1 0;
        0 1];

B_V2 = [0 0 -1  0   0   0;
        0 0 0   -1  0   0];

B_V = [1    0   0   0   0   0   0   0;
       0    1   0   0   0   0   0   0;
       0    0   -1  0   0   0   0   0;
       0    0   0   -1  0   0   0   0];

C_V = [K_iv 0;
        0   K_iv];

C_VV = [K_iv 0   0   0;
       0   K_iv 0   0;
       0    0   0   0
       0    0   0   0];


D_V1 = [K_pv    0;
        0       K_pv];

D_V2 = [0 0 -K_pv  -w_0*C_f   F   0;
        0 0 w_0*C_f  -K_pv  0   F];

D_V = [K_pv 0   0   0   0   0   0   0;
       0    K_pv    0   0   0   0   0   0;
       0    0   0    0   -K_pv   -w_0*C_f    F   0;
       0    0   0    0   -w_0*C_f    -K_pv   0   F];

%State Space and Transfer Function
inputs_vc = {'v_o_d_ref','v_o_q_ref','i_L_d','i_L_q','v_o_d','v_o_q','i_o_d','i_o_q'};
states_vc = {'error_d','error_q','zırt','pırt'}; %V_o_d_ref - v_o_d
outputs_vc = {'i_L_d_ref','i_L_q_ref','zırto','pırto'};
% 
linModel_vc = ss(A_V,B_V,C_VV,D_V, 'statename', states_vc, 'Inputname', inputs_vc,'outputname', outputs_vc);
tfModel_vc = tf(linModel_vc);

%% Current Controller
K_ic = Kic;
K_pc = Kpc;


A_C = zeros(2,2);

B_C1 = [1 0;
        0 1];

B_C2 = [-1 0 0  0   0   0;
        0 -1 0   0  0   0];

C_C = [K_ic 0;
        0   K_ic];

D_C1 = [K_pc 0;
        0   K_pc];

D_C2 = [-K_pc -w_0*L_f 0  0   0   0;
        w_0*L_f -K_pc 0    0 0   0];

%% Output LCL Filter Dynamics

I_L_d = 2500;
I_L_q = 0;

A_LCL = [-r_Lf/L_f      w_0     -1/L_f          0           0                0;
         -w_0       -r_Lf/L_f       0          -1/L_f       0                0;
         1/C_f      0               0           w_0         -1/C_f           0;
         0          1/C_f           -w_0        0           0               -1/C_f;
         0          0             1/L_g         0           -r_Lg/L_g       w_0;
         0          0               0           1/L_g       -w_0            -r_Lg/L_g];

B_LCL1 =    [1/L_f  0;
            0   1/L_f;
            0       0;
            0       0;
            0       0;
            0       0];

B_LCL2 =   [0       0;
            0       0;
            0       0;
            0       0;
            -1/L_g  0;
            0   1/L_g];

B_LCL3 = [I_L_q;     -I_L_d;   V_o_q;      -V_o_d;      I_o_q;       -I_o_d];

%% Complete Inverter Model

T_s1 = [cosd(delta_phi_0)   -sind(delta_phi_0);
       sind(delta_phi_0)    cosd(delta_phi_0)];
T_ss = inv(T_s1);

T_c = [-I_o_d*sind(delta_phi_0)-I_o_q*cosd(delta_phi_0);
        I_o_d*cosd(delta_phi_0)-I_o_q*sind(delta_phi_0)];

T_v = [-V_bus_d*sind(delta_phi_0)+V_bus_q*cosd(delta_phi_0);   -V_bus_d*cosd(delta_phi_0)-V_bus_q*sind(delta_phi_0)];

% T_v = inv(T_v1);

Z1 = zeros(3,2);
Z2 = zeros(2,2);
Z3 = zeros(3,2);
Z4 = zeros(2,2);
Z5 = zeros(2,2);
Z6 = zeros(7,2);
Z7 = zeros(2,10);

% A_INV_3to13 = cat(2,A_P,Z1,Z3,B_P);
% A_INV_5to13 = cat(2,B_V1*C_P_v,Z2,Z4,B_V2);
C_inv_w = [C_P_w   0   0   0    0   0   0   0   0   0   0];
C_inv_c = [T_c  Z7   T_s1];


A_INV = [A_P                                        Z1                  Z3                   B_P;                                          
         B_V1*C_P_v                                 Z2                  Z2                  B_V2;
         B_C1*D_V1*C_P_v                            B_C1*C_V            Z5                   B_C1*D_V2+B_C2;
         (B_LCL1*D_C1*D_V1*C_P_v)+(B_LCL2*T_v)+(B_LCL3*C_P_w)    B_LCL1*D_C1*C_V     B_LCL1*C_C           A_LCL+(B_LCL1*(D_C1*D_V2+D_C2))];


B_INV = [Z6;
         B_LCL2*T_ss];
 
C_INV = [C_inv_w;
        C_inv_c];

D_INV = zeros(3,2);

%State Space and Transfer Function of Grid Forming Model
% Xvd & Xvq stands for voltage controller PI states for 'd' and 'q' frames
% Xcd % Xcq stands for current controller PI states for 'd' and 'q' frames
inputs_inverter = {'v_vsc_d_ref','v_vsc_q_ref'};
states_inverter = {'delta_phi','delta_P','delta_Q','Xvd','Xvq','Xcd','Xcq','i_L_d','i_L_q','v_o_d','v_o_q','i_o_d','i_o_q'}; %V_o_d_ref - v_o_d
outputs_inverter = {'delta_w','i_o_d','i_o_q'};
% Linear and Transfer Function Model
linModel_inverter = ss(A_INV,B_INV,C_INV,D_INV, 'statename', states_inverter, 'Inputname', inputs_inverter,'outputname', outputs_inverter);
tfModel_inverter = tf(linModel_inverter);

%Open Loop Dynamics
%From input "v_vsc_d_ref" 
v_vsc_d_ref_2_delta_w = tf(linModel_inverter(1,1)); %to output "delta_w ...."
v_vsc_d_ref_2_i_o_d = tf(linModel_inverter(2,1)); %to output "i_o_d ...."
v_vsc_d_ref_2_i_o_q = tf(linModel_inverter(3,1)); %to output "i_o_q ...."
%From input "v_vsc_q_ref" 
v_vsc_q_ref_2_delta_w = tf(linModel_inverter(1,2)); %to output "delta_w ...."
v_vsc_q_ref_2_i_o_d = tf(linModel_inverter(2,2)); %to output "i_o_d ...."
v_vsc_q_ref_2_i_o_q = tf(linModel_inverter(3,2)); %to output "i_o_q ...."

% Frequency Response and root analysis
polesofv_vsc_d_ref_2_delta_w = pole(v_vsc_d_ref_2_delta_w);
zeroofv_vsc_d_ref_2_delta_w= zero(v_vsc_d_ref_2_delta_w);
pzmap(v_vsc_d_ref_2_delta_w);
damp(v_vsc_d_ref_2_delta_w);
bode(v_vsc_d_ref_2_delta_w);

polesofv_vsc_d_ref_2_i_o_d = pole(v_vsc_d_ref_2_i_o_d);
zeroofv_vsc_d_ref_2_i_o_d= zero(v_vsc_d_ref_2_i_o_d);
pzmap(v_vsc_d_ref_2_i_o_d);
damp(v_vsc_d_ref_2_i_o_d);
bode(v_vsc_d_ref_2_i_o_d);

polesofv_vsc_d_ref_2_i_o_q = pole(v_vsc_d_ref_2_i_o_q);
zeroofv_vsc_d_ref_2_i_o_q= zero(v_vsc_d_ref_2_i_o_q);
pzmap(v_vsc_d_ref_2_i_o_q);
damp(v_vsc_d_ref_2_i_o_q);
bode(v_vsc_d_ref_2_i_o_q);

polesofv_vsc_q_ref_2_delta_w = pole(v_vsc_q_ref_2_delta_w);
zeroofv_vsc_q_ref_2_delta_w= zero(v_vsc_q_ref_2_delta_w);
pzmap(v_vsc_q_ref_2_delta_w);
damp(v_vsc_q_ref_2_delta_w);
bode(v_vsc_q_ref_2_delta_w);

polesofv_vsc_q_ref_2_i_o_d = pole(v_vsc_q_ref_2_i_o_d);
zeroofv_vsc_q_ref_2_i_o_d= zero(v_vsc_q_ref_2_i_o_d);
pzmap(v_vsc_q_ref_2_i_o_d);
damp(v_vsc_q_ref_2_i_o_d);
bode(v_vsc_q_ref_2_i_o_d);

polesofv_vsc_q_ref_2_i_o_q = pole(v_vsc_q_ref_2_i_o_q);
zeroofv_vsc_q_ref_2_i_o_q= zero(v_vsc_q_ref_2_i_o_q);
pzmap(v_vsc_q_ref_2_i_o_q);
damp(v_vsc_q_ref_2_i_o_q);
bode(v_vsc_q_ref_2_i_o_q);

%Design of the LQR Controller

Q_lqr = [1  0   0   0   0   0   0   0   0   0   0   0   0;
         0  0.5   0   0   0   0   0   0   0   0   0   0   0;
         0  0   0.5   0   0   0   0   0   0   0   0   0   0;
         0  0   0   0.5   0   0   0   0   0   0   0   0   0;
         0  0   0   0   1   0   0   0   0   0   0   0   0;
         0  0   0   0   0   1   0   0   0   0   0   0   0;
         0  0   0   0   0   0   1   0   0   0   0   0   0;
         0  0   0   0   0   0   0   1   0   0   0   0   0;
         0  0   0   0   0   0   0   0   1   0   0   0   0;
         0  0   0   0   0   0   0   0   0   1   0   0   0;
         0  0   0   0   0   0   0   0   0   0   1   0   0;
         0  0   0   0   0   0   0   0   0   0   0   1   0;
         0  0   0   0   0   0   0   0   0   0   0   0   1];

R_lqr = [1  0;
         0  1];
%          0  0;
%          0  0;
%          0  0;
%          0  0;
%          0  0;
%          0  0;
%          0  0;
%          0  0;
%          0  0;
%          0  0;
%          0  0];

[K,S,CLP] = lqr(linModel_inverter,Q_lqr,R_lqr);

G1 = inv(A_INV-B_INV*K);
G2 = C_INV*G1;
% G3 = inv(G2);
% G = -inv(C_INV*G1*B_INV);

sys2 = ss(A_INV-B_INV*K,B_INV,C_INV,D_INV);
step(sys2);
%% Controller
% Complete Inverter Model
V_vsc_d = V_m;
V_vsc_q = V_m;

Iggd = I_o_d;
Iggq = I_o_q;
Vgfd = V_o_d;
Vgfq = V_o_q;
w_1 = w_b;
Vn = V_b;
Kqp = nq;
Kqi = nq;
Igid = I_L_d;
Igiq = I_L_q;
Vgid = V_vsc_d;
Vgiq = V_vsc_q;
Dp = 0;
H = 5;
Dq = 10;

A_inv = zeros(15,15);
B_inv = zeros(15,4);
C_inv = zeros(2,15);
D_inv = zeros(2,4);


A_inv(1,1) = -Dp/(2*H);     A_inv(1,2) = (3*((Iggd*Vgfq)-(Iggq*Vgfd)-(Iggd*Vgfq)+(Iggq*Vgfd)))/(2*S_b);     A_inv(1,3) = 0;     A_inv(1,4) = 0;     A_inv(1,5) = 0;     A_inv(1,6) = 0;     A_inv(1,7) = 0;     A_inv(1,8) = 0;
A_inv(1,9) = 0;             A_inv(1,10) = 0;         A_inv(1,11) = 0;    A_inv(1,12) = -(3*Iggd)/(2*S_b);   A_inv(1,13) = -(3*Iggq)/(2*S_b);        A_inv(1,14) = -(3*Vgfd)/(2*S_b);        A_inv(1,15) = -(3*Vgfq)/(2*S_b);

A_inv(2,1) = w_1/(2*H);     A_inv(2,2) = 0;          A_inv(2,3) = 0;     A_inv(2,4) = 0;     A_inv(2,5) = 0;     A_inv(2,6) = 0;     A_inv(2,7) = 0;     A_inv(2,8) = 0;
A_inv(2,9) = 0;             A_inv(2,10) = 0;         A_inv(2,11) = 0;    A_inv(2,12) = 0;   A_inv(1,13) = 0;     A_inv(1,14) = 0;    A_inv(1,15) = 0;

A_inv(3,1) = 0;             A_inv(3,2) = (3*((Iggd*Vgfd)+(Iggq*Vgfq)-(Iggd*Vgfd)-(Iggq*Vgfq)))/(2*S_b);     A_inv(3,3) = -Dq*Kqi;     A_inv(3,4) = 0;     A_inv(3,5) = 0;     A_inv(3,6) = 0;     A_inv(3,7) = 0;     A_inv(3,8) = 0;
A_inv(3,9) = 0;             A_inv(3,10) = 0;         A_inv(3,11) = 0;    A_inv(3,12) = (3*Iggq)/(2*S_b);   A_inv(3,13) = -(3*Iggd)/(2*S_b);        A_inv(3,14) = -(3*Vgfq)/(2*S_b);        A_inv(3,15) = (3*Vgfd)/(2*S_b);

A_inv(4,1) = 0;             A_inv(4,2) = -Vgfq;      A_inv(4,3) = Kqi*Vn;     A_inv(4,4) = 0;     A_inv(4,5) = 0;     A_inv(4,6) = 0;     A_inv(4,7) = 0;     A_inv(4,8) = 0;
A_inv(4,9) = 0;             A_inv(4,10) = 0;         A_inv(4,11) = 0;         A_inv(4,12) = -1;   A_inv(4,13) = 0;        A_inv(4,14) = 0;        A_inv(4,15) = (3*Kqp*Vgfd*Vn)/(2*S_b);

A_inv(5,1) = 0;             A_inv(5,2) = Vgfd;       A_inv(5,3) = 0;          A_inv(5,4) = Kiv;     A_inv(5,5) = 0;     A_inv(5,6) = 0;     A_inv(5,7) = 0;     A_inv(5,8) = 0;
A_inv(5,9) = 0;             A_inv(5,10) = -1;         A_inv(5,11) = 0;         A_inv(5,12) = 0;     A_inv(5,13) = -1;        A_inv(5,14) = 0;        A_inv(5,15) = 0;

A_inv(6,1) = 0;             A_inv(6,2) = -Kpv*Vgfq-Igiq;       A_inv(6,3) = Kqi*Kpv*Vn;          A_inv(6,4) = 0;     A_inv(6,5) = Kiv;     A_inv(6,6) = 0;     A_inv(6,7) = 0;     A_inv(6,8) = 0;
A_inv(6,9) = 0;             A_inv(6,10) = 0;         A_inv(6,11) = -1;         A_inv(6,12) = -Kpv;     A_inv(6,13) = 0;        A_inv(6,14) = 0;        A_inv(6,15) = 0;

A_inv(7,1) = 0;             A_inv(7,2) = Kpv*Vgfd-Igid;       A_inv(7,3) = 0;          A_inv(7,4) = 8*Kpc*Kiv/(3*T_s);     A_inv(7,5) = 0;     A_inv(7,6) = 8*Kic/(3*T_s);     A_inv(7,7) = 0;     A_inv(7,8) = -4/(3*T_s);
A_inv(7,9) = 0;             A_inv(7,10) = -(8*Kpc)/(3*T_s);         A_inv(7,11) = 0;         A_inv(7,12) = 0;     A_inv(7,13) = -Kpv;        A_inv(7,14) = 0;        A_inv(7,15) = 0;

A_inv(8,1) = 0;             A_inv(8,2) = -(8*Kpc)*(Kpv*Vgfq+Igiq)/(3*T_s);       A_inv(8,3) = (8*Kpc*Kqi*Kpv*Vn)/(3*T_s);          A_inv(8,4) = 0;     A_inv(8,5) = 8*Kpc*Kiv/(3*T_s);     A_inv(8,6) = 0;     A_inv(8,7) = 8*Kic/(3*T_s);     A_inv(8,8) = 0;
A_inv(8,9) = -4/(3*T_s);    A_inv(8,10) = 0;         A_inv(8,11) = -(8*Kpc)/(3*T_s);         A_inv(8,12) = -(8*Kpc*Kpv)/(3*T_s);     A_inv(8,13) = 0;        A_inv(8,14) = 0;        A_inv(8,15) = 0;

A_inv(9,1) = 0;             A_inv(9,2) = ((8*Kpc)*((Kpv*Vgfd)+Igid))/(3*T_s);       A_inv(9,3) = 0;          A_inv(9,4) = -(Kpc*Kiv)/L_f;     A_inv(9,5) = 0;     A_inv(9,6) = -Kic/L_f;     A_inv(9,7) = 0;     A_inv(9,8) = 1/L_f;
A_inv(9,9) = 0;             A_inv(9,10) = Kpc/L_f;         A_inv(9,11) = w_1;         A_inv(9,12) = 0;     A_inv(9,13) = -(8*Kpc*Kpv)/(3*T_s);        A_inv(9,14) = 0;        A_inv(9,15) = 0;

A_inv(10,1) = 0;            A_inv(10,2) = ((Igiq*Kpc)-Vgiq+(Kpc*Kpv*Vgfq))/(L_f);       A_inv(10,3) = -(Kpc*Kqi*Kpv*Vn)/L_f;          A_inv(10,4) = 0;     A_inv(10,5) = -(Kpc*Kiv)/L_f;     A_inv(10,6) = 0;     A_inv(10,7) = -Kic/L_f;     A_inv(10,8) = 0;
A_inv(10,9) = 1/L_f;        A_inv(10,10) = -w_1;         A_inv(10,11) = Kpc/L_f;         A_inv(10,12) = ((Kpc*Kpv)-1)/L_f;     A_inv(10,13) = 0;        A_inv(10,14) = 0;        A_inv(10,15) = 0;

A_inv(11,1) = 0;            A_inv(11,2) = -((Igid*Kpc)-Vgid+(Kpc*Kpv*Vgfd))/(L_f);       A_inv(11,3) = 0;          A_inv(11,4) = 0;     A_inv(11,5) = 0;     A_inv(11,6) = 0;     A_inv(11,7) = 0;     A_inv(11,8) = 0;
A_inv(11,9) = 0;            A_inv(11,10) = 1/C_f;         A_inv(11,11) = 0;         A_inv(11,12) = 0;     A_inv(11,13) = ((Kpc*Kpv)-1)/L_f;        A_inv(11,14) = 0;        A_inv(11,15) = 0;

A_inv(12,1) = 0;            A_inv(12,2) = 0;       A_inv(12,3) = 0;          A_inv(12,4) = 0;     A_inv(12,5) = 0;     A_inv(12,6) = 0;     A_inv(12,7) = 0;     A_inv(12,8) = 0;
A_inv(12,9) = 0;            A_inv(12,10) = 0;         A_inv(12,11) = 1/C_f;         A_inv(12,12) = 0;     A_inv(12,13) = w_1;        A_inv(12,14) = -1/C_f;        A_inv(12,15) = 0;

A_inv(13,1) = 0;            A_inv(13,2) = 0;       A_inv(13,3) = 0;          A_inv(13,4) = 0;     A_inv(13,5) = 0;     A_inv(13,6) = 0;     A_inv(13,7) = 0;     A_inv(13,8) = 0;
A_inv(13,9) = 0;            A_inv(13,10) = 0;         A_inv(13,11) = 0;         A_inv(13,12) = -w_1;     A_inv(13,13) = 0;        A_inv(13,14) = 0;        A_inv(13,15) = -1/C_f;

A_inv(14,1) = 0;            A_inv(14,2) = 0;       A_inv(14,3) = 0;          A_inv(14,4) = 0;     A_inv(14,5) = 0;     A_inv(14,6) = 0;     A_inv(14,7) = 0;     A_inv(14,8) = 0;
A_inv(14,9) = 0;            A_inv(14,10) = 0;         A_inv(14,11) = 0;         A_inv(14,12) = 1/L_g;     A_inv(14,13) = 0;        A_inv(14,14) = 0;        A_inv(14,15) = w_1;

A_inv(15,1) = 0;            A_inv(15,2) = 0;       A_inv(15,3) = 0;          A_inv(15,4) = 0;     A_inv(15,5) = 0;     A_inv(15,6) = 0;     A_inv(15,7) = 0;     A_inv(15,8) = 0;
A_inv(15,9) = 0;            A_inv(15,10) = 0;         A_inv(15,11) = 0;         A_inv(15,12) = 0;     A_inv(15,13) = 1/L_g;        A_inv(15,14) = -w_1;        A_inv(15,15) = 0;


B_inv(1,1) = 1;
B_inv(3,3) = -w_1;
B_inv(4,2) = 1;
B_inv(14,3) = -1/L_g;
B_inv(15,4) = -1/L_g;

C_inv(1,2) = 3*((Iggd*Vgfq)-(Iggq*Vgfd)-(Iggd*Vgfq)+(Iggq*Vgfd))/2;
C_inv(2,2) = (3*((Iggd*Vgfd)-(Iggq*Vgfq)-(Iggd*Vgfd)+(Iggq*Vgfq)))/(2*S_b);
C_inv(1,12) = 3*Iggd/2;     C_inv(1,13) = 3*Iggq/2;     C_inv(1,14) = 3*Vgfd/2;     C_inv(1,15) = 3*Vgfq/2;
C_inv(2,12) = -3*Iggq/2;     C_inv(2,13) = 3*Iggd/2;     C_inv(2,14) = 3*Vgfq/2;     C_inv(2,15) = -3*Vgfd/2;

%State Space and Transfer Function of Grid Forming Model
% Xvd & Xvq stands for voltage controller PI states for 'd' and 'q' frames
% Xcd % Xcq stands for current controller PI states for 'd' and 'q' frames
inputs_inverter2 = {'delta_Pg_ref_pu','delta_Qg_ref_pu','delta_V_out_d','delta_V_out_q'};
states_inverter2 = {'delta_X_pe','delta_phi','delta_X_qe','Xvd','Xvq','Xcd','Xcq','Xdd','Xdq','i_L_d','i_L_q','v_o_d','v_o_q','i_o_d','i_o_q'}; %V_o_d_ref - v_o_d
outputs_inverter2 = {'delta_Pg_c','delta_Qg_c'};
% Linear and Transfer Function Model
linModel_inverter2 = ss(A_inv,B_inv,C_inv,D_inv, 'statename', states_inverter2, 'Inputname', inputs_inverter2,'outputname', outputs_inverter2);
tfModel_inverter2 = tf(linModel_inverter2);

%Open Loop Dynamics
%From input "delta_Pg_ref_pu" 
delta_Pg_ref_pu_2_delta_Pg_c = tf(linModel_inverter(1,1)); %to output "delta_Pg_c ...."
delta_Pg_ref_pu_2_delta_Qg_ = tf(linModel_inverter(2,1)); %to output "delta_Qg_c ...."

%From input "delta_Qg_ref_pu" 
delta_Qg_ref_pu_2_delta_Pg_c = tf(linModel_inverter(1,2)); %to output "delta_Pg_c ...."
delta_Qg_ref_pu_2_delta_Qg_c = tf(linModel_inverter(2,2)); %to output "delta_Qg_c ...."

%From input "delta_V_out_d" 
delta_V_out_d_pu_2_delta_Pg_c = tf(linModel_inverter(1,2)); %to output "delta_Pg_c ...."
delta_V_out_d_pu_2_delta_Qg_c = tf(linModel_inverter(2,2)); %to output "delta_Qg_c ...."

%From input "delta_V_out_q" 
delta_V_out_q_pu_2_delta_Pg_c = tf(linModel_inverter(1,2)); %to output "delta_Pg_c ...."
delta_V_out_q_pu_2_delta_Qg_c = tf(linModel_inverter(2,2)); %to output "delta_Qg_c ...."

% Frequency Response and root analysis
polesofdelta_Pg_ref_pu_2_delta_Pg_c = pole(delta_Pg_ref_pu_2_delta_Pg_c);
zeroofdelta_Pg_ref_pu_2_delta_Pg_c= zero(delta_Pg_ref_pu_2_delta_Pg_c);
pzmap(delta_Pg_ref_pu_2_delta_Pg_c);
damp(delta_Pg_ref_pu_2_delta_Pg_c);
bode(delta_Pg_ref_pu_2_delta_Pg_c);

polesofdelta_Pg_ref_pu_2_delta_Qg_ = pole(delta_Pg_ref_pu_2_delta_Qg_);
zeroofdelta_Pg_ref_pu_2_delta_Qg_= zero(delta_Pg_ref_pu_2_delta_Qg_);
pzmap(delta_Pg_ref_pu_2_delta_Qg_);
damp(delta_Pg_ref_pu_2_delta_Qg_);
bode(delta_Pg_ref_pu_2_delta_Qg_);

polesofdelta_Qg_ref_pu_2_delta_Pg_c = pole(delta_Qg_ref_pu_2_delta_Pg_c);
zeroofdelta_Qg_ref_pu_2_delta_Pg_c= zero(delta_Qg_ref_pu_2_delta_Pg_c);
pzmap(delta_Qg_ref_pu_2_delta_Pg_c);
damp(delta_Qg_ref_pu_2_delta_Pg_c);
bode(delta_Qg_ref_pu_2_delta_Pg_c);

polesofdelta_Qg_ref_pu_2_delta_Qg_c = pole(delta_Qg_ref_pu_2_delta_Qg_c);
zeroofdelta_Qg_ref_pu_2_delta_Qg_c= zero(delta_Qg_ref_pu_2_delta_Qg_c);
pzmap(delta_Qg_ref_pu_2_delta_Qg_c);
damp(delta_Qg_ref_pu_2_delta_Qg_c);
bode(delta_Qg_ref_pu_2_delta_Qg_c);

polesofdelta_V_out_d_pu_2_delta_Pg_c = pole(delta_V_out_d_pu_2_delta_Pg_c);
zeroofdelta_V_out_d_pu_2_delta_Pg_c= zero(delta_V_out_d_pu_2_delta_Pg_c);
pzmap(delta_V_out_d_pu_2_delta_Pg_c);
damp(delta_V_out_d_pu_2_delta_Pg_c);
bode(delta_V_out_d_pu_2_delta_Pg_c);

polesofdelta_V_out_d_pu_2_delta_Qg_c = pole(delta_V_out_d_pu_2_delta_Qg_c);
zeroofdelta_V_out_d_pu_2_delta_Qg_c= zero(delta_V_out_d_pu_2_delta_Qg_c);
pzmap(delta_V_out_d_pu_2_delta_Qg_c);
damp(delta_V_out_d_pu_2_delta_Qg_c);
bode(delta_V_out_d_pu_2_delta_Qg_c);

polesofdelta_V_out_q_pu_2_delta_Pg_c = pole(delta_V_out_q_pu_2_delta_Pg_c);
zeroofdelta_V_out_q_pu_2_delta_Pg_c= zero(delta_V_out_q_pu_2_delta_Pg_c);
pzmap(delta_V_out_q_pu_2_delta_Pg_c);
damp(delta_V_out_q_pu_2_delta_Pg_c);
bode(delta_V_out_q_pu_2_delta_Pg_c);

polesofdelta_V_out_q_pu_2_delta_Qg_c = pole(delta_V_out_q_pu_2_delta_Qg_c);
zeroofdelta_V_out_q_pu_2_delta_Qg_c= zero(delta_V_out_q_pu_2_delta_Qg_c);
pzmap(delta_V_out_q_pu_2_delta_Qg_c);
damp(delta_V_out_q_pu_2_delta_Qg_c);
bode(delta_V_out_q_pu_2_delta_Qg_c);