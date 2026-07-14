
function result = simulateFuzzyPID(fis,vref,Ts,Kp0,Ki0,Kd0)
%% Vehicle parameters
m   = 1500;      % kg
Cr  = 0.015;     % Rolling resistance
rho = 1.225;     % Air density
Cd  = 0.32;      % Drag coefficient
A   = 2.2;       % Frontal area (m^2)
g   = 9.81;
theta = 0;
N = length(vref);
uP_hist=zeros(1,N);
uI_hist=zeros(1,N);
uD_hist=zeros(1,N);
Kp=Kp0;
Ki=Ki0;
Kd=Kd0;
Kp_min = 0.5*Kp0;   Kp_max = 2*Kp0;
Ki_min = 0;         Ki_max = 2*Ki0;   % Ki không nên âm - tránh đảo dấu tích phân
Kd_min = 0;         Kd_max = 2*Kd0;
%% Scaling
Ke  = 0.1;
Kec = 0.02;
Kkp = Kp0*0.5;
Kki = Ki0*0.1;
Kkd = Kd0*0.35;
%% Variable
y=zeros(1,N);
u=zeros(1,N);
e=zeros(1,N);
de=zeros(1,N);
Kp_hist=zeros(1,N);
Ki_hist=zeros(1,N);
Kd_hist=zeros(1,N);
I=0;
for k=2:N
    %% Error
    e(k)=vref(k)-y(k-1);
    de(k)=(e(k)-e(k-1))/Ts;
% de(k)=0;
    %% Normalize
    ef=Ke*e(k);
    dec=Kec*de(k);
    ef=max(min(ef,3),-3);
    dec=max(min(dec,3),-3);
    %% Fuzzy
    out=evalfis(fis,[ef dec]);
    %% Adaptive PID
    Kp=Kp+Kkp*out(1);
    Ki=Ki+Kki*out(2);
    Kd=Kd+Kkd*out(3);
% Clamp (saturation) để tránh drift/windup ở tầng gain-adaptation
    Kp = min(max(Kp, Kp_min), Kp_max);
    Ki = min(max(Ki, Ki_min), Ki_max);
    Kd = min(max(Kd, Kd_min), Kd_max);
    %% PID
    I=I+e(k)*Ts;
    D=de(k);
    u(k)=Kp*e(k)+Ki*I+Kd*D;
% u(k)=min(max(u(k),uMin),uMax);
% u(k)=-(Kp*e(k)+Ki*I+Kd*D);
    %% Vehicle dynamics
if abs(y(k-1)) < 1e-3
        Froll = 0;
else
        Froll = Cr*m*g*sign(y(k-1));
end
    Fdrag = 0.5*rho*Cd*A*y(k-1)*abs(y(k-1));
    Fgrade = m*g*sin(theta);
    dv = (u(k)-Froll-Fdrag-Fgrade)/m;
    %% Euler Integration
    y(k)=y(k-1)+Ts*dv;
    %% Save
    Kp_hist(k)=Kp;
    Ki_hist(k)=Ki;
    Kd_hist(k)=Kd;
    uP_hist(k)=Kp*e(k);
    uI_hist(k)=Ki*I;
    uD_hist(k)=Kd*D;
end
result.y=y;
result.u=u;
result.e=e;
result.de=de;
result.Kp=Kp_hist;
result.Ki=Ki_hist;
result.Kd=Kd_hist;
result.u=u;
result.uP=uP_hist;
result.uI=uI_hist;
result.uD=uD_hist;
end