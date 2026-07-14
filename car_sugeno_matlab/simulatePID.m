function result = simulatePID(vref,Ts,Kp,Ki,Kd)

%% Vehicle parameters

m   = 1500;      % kg
Cr  = 0.015;     % Rolling resistance
rho = 1.225;     % Air density
Cd  = 0.32;      % Drag coefficient
A   = 2.2;       % Frontal area (m^2)
g   = 9.81;
theta = 0;
uMax = 4000;
uMin = -4000;
N = length(vref);
uP_hist=zeros(1,N);
uI_hist=zeros(1,N);
uD_hist=zeros(1,N);

y=zeros(1,N);
u=zeros(1,N);
e=zeros(1,N);
de=zeros(1,N);


I=0;

for k=2:N

    e(k)=vref(k)-y(k-1);
    de(k)=(e(k)-e(k-1))/Ts;
    % de(k)=0;

    I=I+e(k)*Ts;
    D=de(k);

    u(k)=Kp*e(k)+Ki*I+Kd*D;
    % u(k)=min(max(u(k),uMin),uMax);
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

    uP_hist(k)=Kp*e(k);
    uI_hist(k)=Ki*I;
    uD_hist(k)=Kd*D;

end

result.y=y;
result.u=u;
result.e=e;
result.de=de;

result.u=u;
result.uP=uP_hist;
result.uI=uI_hist;
result.uD=uD_hist;

end