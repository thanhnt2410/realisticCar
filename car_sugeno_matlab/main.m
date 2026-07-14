clear;
clc;
Kp = 1500;
Ki = 900;
Kd = 400;
s = tf('s');
% plant = 1/((0.5*s+1)*(s+1));
% plant = 1/(0.5*s+1);
% C = pidtune(plant,'PID');
%% Create Fuzzy System
fis = createFuzzyRules();
writeFIS(fis,"FuzzyPID.fis");

e = -3:1:3;
ec = -3:1:3;

dKp = zeros(7,7);
dKi = zeros(7,7);
dKd = zeros(7,7);

for i = 1:7
    for j = 1:7
        out = evalfis(fis,[e(i) ec(j)]);
        dKp(i,j) = out(1);
        dKi(i,j) = out(2);
        dKd(i,j) = out(3);
    end
end

disp(dKp)
disp(dKi)
disp(dKd)
%% ================= Simulation =================

Ts = 0.01;
Tend = 100;

t = 0:Ts:Tend;
N = length(t);

%% -------- Ramp reference --------

vrefRamp = zeros(1,N);
vrefRampNoise = zeros(1,N);

for k = 1:N
    tl = mod(t(k),100);

    if tl < 5
        % 10 -> 50
        vrefRamp(k) = 0 + 8*tl;

    elseif tl < 15
        % Giữ 40
        vrefRamp(k) = 40;

    elseif tl < 20
        % 40 -> 20
        vrefRamp(k) = 40 - 4*(tl-15);

    elseif tl < 30
        % Giữ 20
        vrefRamp(k) = 20;

    elseif tl < 35
        % 10 -> 40
        vrefRamp(k) = 20 + 5*(tl-30);

    elseif tl < 45
        % Giữ 40
        vrefRamp(k) = 45;

    elseif tl < 50
        % 40 -> 10
        vrefRamp(k) = 45 - 4*(tl-45);

    elseif tl < 60
        % Giữ 10
        vrefRamp(k) = 25;

    elseif tl < 65
        % 10 -> 60
        vrefRamp(k) = 25 + 10*(tl-60);

    elseif tl < 75
        % Giữ 60
        vrefRamp(k) = 75;

    elseif tl < 80
        % 60 -> 10
        vrefRamp(k) = 75 - 8*(tl-75);

    elseif tl < 90
        % Giữ 10
        vrefRamp(k) = 35;
    else 
        vrefRamp(k)= 35 - 35/10*(tl-90);
    end
    vrefRampNoise(k) = vrefRamp(k) + 0.5*randn;
end

%% -------- Step reference --------

vrefStep = zeros(1,N);

for k=1:N
    tl = mod(t(k),100);

    if tl<5
        vrefStep(k)=10;
    elseif tl<15
        vrefStep(k)=50;
    elseif tl<25
        vrefStep(k)=20;
    elseif tl<40
        vrefStep(k)=10;

    elseif tl<55
        vrefStep(k)=60;

    elseif tl<70
        vrefStep(k)=30;
    elseif tl<85
        vrefStep(k)=50;

    elseif tl<100
        vrefStep(k)=40;
    end

end
%% -------- Ramp reference noise--------
% vrefRampNoise = zeros(1,N);
% 
% for k = 1:N
%     tl = mod(t(k),100);
% 
%     if tl < 5
%         % 10 -> 50
%         vrefRampNoise(k) = 0 + 8*tl;
% 
%     elseif tl < 15
%         % Giữ 40
%         vrefRampNoise(k) = 40;
% 
%     elseif tl < 20
%         % 40 -> 20
%         vrefRampNoise(k) = 40 - 4*(tl-15);
% 
%     elseif tl < 30
%         % Giữ 20
%         vrefRampNoise(k) = 20;
% 
%     elseif tl < 35
%         % 10 -> 40
%         vrefRampNoise(k) = 20 + 5*(tl-30);
% 
%     elseif tl < 45
%         % Giữ 40
%         vrefRampNoise(k) = 45;
% 
%     elseif tl < 50
%         % 40 -> 10
%         vrefRampNoise(k) = 45 - 4*(tl-45);
% 
%     elseif tl < 60
%         % Giữ 10
%         vrefRampNoise(k) = 25;
% 
%     elseif tl < 65
%         % 10 -> 60
%         vrefRampNoise(k) = 25 + 10*(tl-60);
% 
%     elseif tl < 75
%         % Giữ 60
%         vrefRampNoise(k) = 75;
% 
%     elseif tl < 80
%         % 60 -> 10
%         vrefRampNoise(k) = 75 - 8*(tl-75);
% 
%     elseif tl < 90
%         % Giữ 10
%         vrefRampNoise(k) = 35;
%     else 
%         vrefRampNoise(k)= 35 - 35/10*(tl-90);
%     end
%     vrefRampNoise(k) = vrefRampNoise(k) + 0.5*randn;
% end

%% -------- Interpolation reference (interp1) --------

tNode = [ ...
     0  10  20  35  45  60  75  90 100];

vNode = [ ...
    0  50  50  0  0  60  60  0  0];

vrefInterp = interp1(tNode, vNode, mod(t,100), 'linear');


%% ===== Run Case 1 =====

resultRamp = simulateFuzzyPID(...
    fis,...
    vrefRamp,...
    Ts,...
    Kp,...
    Ki,...
    Kd);
resultRampPID = simulatePID(vrefRamp,Ts,Kp,Ki,Kd);

%% ===== Run Case 2 =====

resultStep = simulateFuzzyPID(...
    fis,...
    vrefStep,...
    Ts,...
    Kp,...
    Ki,...
    Kd);
resultStepPID = simulatePID(vrefStep,Ts,Kp,Ki,Kd);

%% ===== Run Case 3 =====
resultRampNoise = simulateFuzzyPID(...
    fis,...
    vrefRampNoise,...
    Ts,...
    Kp,...
    Ki,...
    Kd);
resultRampNoisePID = simulatePID(vrefRampNoise,Ts,Kp,Ki,Kd);

%% ========================= Plot =========================

plotResult(t,vrefRamp,resultRamp,resultRampPID,"Ramp");

plotResult(t,vrefStep,resultStep,resultStepPID,"Step");

plotResult(t,vrefRampNoise,resultRampNoise,resultRampNoisePID,"Interpolation");

