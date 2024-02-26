clc;close;clear;

activate_statu = 0.5;

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

nq = 2*V_m/(S_b); %  kV/MVar

%% VSM
m_p=(2*pi*0.5)/(S_b);
% synchronverter control
D_p=(1/(m_p*w_b));
tau_f=0.02;
J=D_p*tau_f;
% AC voltage control--------------------------------
% ki_v_ac=8*5.2500e-04;
% ki_v_ac=0.01;
% kp_v_ac=0.001;
%% PI
ti = 0.3e-3;
tiv = 20*ti;
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

% Kpv = 4.7;
% Kiv = 42;
% 
% Kpc = 0.86;
% Kic = 0.08;
%% Active Droop Model Validation

% Initial Conditions for frequency response analysis
I_o_d = 2600;
I_o_q = 0;
V_o_d = V_m;
V_o_q = 0;
V_bus_d = V_m;
V_bus_q = 0;
delta_phi_0 = 1.9;
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