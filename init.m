clear
%%%%%%%%%%%%%%%%%%%%Defination of Parameters%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts=1/(16e3);        %开关频率、周期
Fs=1/Ts;

Ts_Dead= 0;      %死区 7us下 THD为5%
Ts_FEA = 1e-7;      %仿真步长  
Ts_step= 1e-7;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  电机参数  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n=2000;                                         %额定转速(rpm)
p=4;                                             %极对数
f1=n*p/60;                                       %额定频率
We_ref=2*pi*f1;
Tn=734.56/10;

Jm=0.36;                                          %转动惯量
B=0.0000;                                        %摩擦系数

faif=0.1351;                                       %基波磁链
Rs=9.823e-3;                                       %定子电阻 (欧姆)  

L = ...
[1.0007e-04 -2.1566e-05 -2.1497e-05 4.5031e-05 -4.5102e-05 4.6515e-08 ;
-2.1566e-05 1.0002e-04 -2.1527e-05 -9.1374e-09 4.5057e-05 -4.5072e-05 ;
-2.1497e-05 -2.1527e-05 1.0006e-04 -4.5051e-05 3.9938e-09 4.5048e-05 ;
4.5031e-05 -9.1374e-09 -4.5051e-05 1.0011e-04 -2.1540e-05 -2.1510e-05 ;
-4.5102e-05 4.5057e-05 3.9938e-09 -2.1540e-05 1.0003e-04 -2.1540e-05 ; 
4.6515e-08 -4.5072e-05 4.5048e-05 -2.1510e-05 -2.1540e-05 1.0010e-04];

L_11 = L(1 :3 ,1 :3 );
L_12 = L(1 :3 ,4 :6 );

L_21 = L(4 :6 ,1 :3 );
L_22 = L(4 :6 ,4 :6 );
Labc_pall=(L_11+L_12);
C = [cos(0)         -sin(0)             1;...
    cos(0-2/3*pi)   -sin(0-2/3*pi)      1;...
    cos(0+2/3*pi)   -sin(0+2/3*pi)      1;];

L_dq011 = C\L_11*C;
% Ld=L_dq011(1,1)*1.2;
% Lq=L_dq011(2,2)*1.2;
Ld = 40.2e-6;
Lq = 40.2e-6;
Ls=(Ld+Lq)/2;
L_dq012 = C\L_12*C;
% Ldd=L_dq012(1,1);
% Lqq=L_dq012(2,2);
Ldd = 37e-6;
Lqq = 37e-6;
LD = Ld+Ldd;       % 总d轴电感
LQ = Lq+Lqq;       % 总q轴电感
Lx = Ld-Ldd;       % xy平面的x轴电感，等于相绕组的漏感
Ly = Lq-Lqq;       % xy平面的y轴电感，等于相绕组的漏感
Ldz = Ld-Ldd;      % o1o2平面的x轴电感，等于相绕组的漏感
Lqz = Lq-Lqq;      % o1o2平面的y轴电感，等于相绕组的漏感
% L_dq021 = C\L_21*C;
% L_dq022 = C\L_22*C;

%%%%%%%%%%%%%%%%%%%%%Converter Parameters%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 电池参数
VDC=1200;                              %电池电压/V，（试试母线电压再抬高到1500V），空载与满载情况下电压的变换
INV_Vdc=1/VDC;
% Vdc=1200;
hvss_R_BatteryResistance_Ohm = 0.014; %等效线路电阻/ohm
hvss_L_BatteryInductance_H = 61.9e-6;    %等效线路电感/H

%%%%%%%%%%%%%%%%%%%%%       双dq模型PI参数设计      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Current Controller Parameters%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fc=500;                          %电流环控制带宽
wc=fc*2*pi;                      %电流环剪切角频率

Kp_id=Ld*wc;                       %电流环控制器比例系数 
Kp_iq=Lq*wc;
Ki_id=Rs*wc;                       %电流环控制器积分系数
Ki_iq=Rs*wc; 
%Kawc=Kic/Kpc;                    %电流环控制器抗饱和系数

duty_limit=0.999;                 %Duty limit for two current regulators together
duty_xy_limit=0.1;
%%%%%%%%%%%%%%%%%%%%% Speed Controller Parameters%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

KT=3/2*p*faif;                 %什么系数
fs=5;                          %速度环控制带宽
ws=fs*2*pi;                     %速度环剪切角频率
yita=0.707;

Kis=ws^2*Jm/KT;                 %速度环控制器比例系数
Kps=yita*ws*2*Jm/KT;            %速度环控制器积分系数
kaws=Kis/Kps;                   %速度环控制器抗饱和系数       
Iq_limit=Tn/(1.5*p*faif)*1;

Ts_spc=Ts*0.2;                  %速度环控制时间


%%%%%%%%%%%%%%%%%%%%%       VSD模型PI参数设计      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% VSD模型永磁同步电机PI控制器参数设计脚本
Ld_eff = LD;    % 等效d轴电感 (来自LD) [H]
Lq_eff = LQ;    % 等效q轴电感 (来自LQ) [H]

%% 2. 控制器设计目标（带宽选择）

% 电流环带宽 (建议值为 Fs/10)
fc = 1000;              % [Hz]

% x-y 轴电流环带宽 (建议与d-q环保持一致，确保动态性能统一)
fc_xy = 1000;           % [Hz]

% 转速环带宽 (建议值为 fc/10 ~ fc/5)
fs = 5;                % [Hz]

% 转速环阻尼比 (0.707可获得快速且无超调的响应)
zeta = 0.707;           

%% 3. PI参数计算

% --- 电流环PI参数计算 ---
wc = 2 * pi * fc;       % 电流环角频率 [rad/s]
Kp_id_vsd = Ld_eff * wc;    % d轴比例增益
Ki_id_vsd = Rs * wc;        % d轴积分增益
Kp_iq_vsd = Lq_eff * wc;    % q轴比例增益
Ki_iq_vsd = Rs * wc;        % q轴积分增益

% --- b) x-y 轴电流环 ---
wc_xy = 2 * pi * fc_xy;       % x-y环角频率 [rad/s]
Kp_x = Lx * wc_xy;            % x轴比例增益
Ki_x = Rs * wc_xy;            % x轴积分增益
Kp_y = Ly * wc_xy;            % y轴比例增益
Ki_y = Rs * wc_xy;            % y轴积分增益

% --- 转速环PI参数计算 ---
KT = 1.5 * p * faif;    % 电机转矩系数 [Nm/A]
ws = 2 * pi * fs;       % 转速环角频率 [rad/s]
Kps_vsd = (2 * zeta * ws * Jm) / KT; % 转速环比例增益
Kis_vsd = (ws^2 * Jm) / KT;          % 转速环积分增益

% kp1=0.3;
% ki1=50;
% kp2=0.3;
% ki2=50;
% kp3=7.5;
% ki3=20;
% 双dq模型
kp1=Kp_iq;
ki1=Ki_iq;
kp2=Kp_id;
ki2=Ki_id;
kp3=Kps;
ki3=Kis;
% 
% kp4=0.3;
% ki4=50;
% kp5=0.3;
% ki5=50;
% kp6=5;
% ki6=10.5;
kp4=Kp_y;
ki4=Ki_y;
kp5=Kp_x;
ki5=Ki_x;
kp6=5;
ki6=10.5;

%% Lz前馈
K_lz = 500;
tao_lz = Ts*10;
