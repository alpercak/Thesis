%% Solver sample time
T_s=1e-4;%Increasing Ts makes simulation run faster, but it may increase the numerical error as well!
Tend=50;
T_loss=Tend;
T_load=25;
T_en=T_load-0.5;%Enabling the DC source saturation after the initial synchronization
T_ms=0.001;
%% Network base values
S_b=10*(10^6);
V_b=1000;%L-L rms voltage
f_b=50;w_b=2*pi*f_b;
P_b=S_b;Q_b=S_b;
I_b=S_b/(sqrt(3)*V_b);
Z_b=(V_b^2)/S_b;L_b=Z_b/w_b;C_b=1/(w_b*Z_b);
%% Converter params
V1_rms=1000; %Low voltage side
I_b_LV=S_b/(sqrt(3)*V1_rms);
V_m=sqrt(2/3)*V1_rms;
Vdc_n=3*V_m;
n=200;
C_dc=0.008*(n);
R_f=0.001/n;
L_f=(1/n)*200*10^-6;
C_f=n*300*10^-6;
R_dc=(Vdc_n/(0.05*(S_b)/Vdc_n));
w_f=2*pi*0.1*f_b;
f_0 = 50;
%% Control parameters
%DC source and governor-turbine time constants
tau_dc=0.05;tau_g=5;
%defining SM governer gain----------------------
droop_percentage=1;
% grid-forming converter control----------------
I_b_dc=S_b/Vdc_n;
i_loss_dc=Vdc_n/R_dc;
i_ul=5*1.15*(S_b/Vdc_n)+i_loss_dc;%dc source saturation limits
i_ll=5*-1.15*(S_b/Vdc_n)-i_loss_dc;%dc source saturation limits
% DC voltage control--------------------------------
eta_1= w_b/Vdc_n;
m_p=(2*pi*(f_0*0.01))/(S_b);
k_dc=eta_1/(Vdc_n*m_p);
K_p=(1/Vdc_n);
K_r=1/R_dc;
% AC voltage control--------------------------------
ki_v_ac=2*0.25;
kp_v_ac=0.001;
% Voltage loop----------------------------------------oynanıyor
n=200;
Kp_v = 0.52;
Ki_v = (n)*1.161022;
Kff_v = 1;
Ti_v = Kp_v/Ki_v; 
% Current loop
Kp_i =0.738891;
Ki_i =(1/n)*1.19;
Kff_i = 1;
Ti_i = Kp_i / Ki_i;
%% Network loading and set-points
base=2.25; % base load
load_change=0.75;% load disturbance
ps=base/3; %set-ponit in [MW]
pl=S_b*ps; %loads in [W]
load_step=S_b*load_change; %disturbance in [W]
%% Grid
Vg = 1000;
f_0 = 50;
