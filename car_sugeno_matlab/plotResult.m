function plotResult(t,vref,resultFuzzy,resultPID,name)

figure('Name', sprintf('%s Reference - Full Analysis', name));

%% Velocity
subplot(3,1,1)

plot(t,vref,'r--','LineWidth',2)
hold on
plot(t,resultFuzzy.y,'b','LineWidth',2)
plot(t,resultPID.y,'g-.','LineWidth',1.5)

grid on
title(sprintf('%s Reference - Velocity', name));
xlabel('Time (s)')
ylabel('Velocity')
legend('Reference','Fuzzy PID','PID')

%% Adaptive Gain
subplot(3,1,2)

plot(t,resultFuzzy.Kp,'LineWidth',2)
hold on
plot(t,resultFuzzy.Ki,'LineWidth',2)
plot(t,resultFuzzy.Kd,'LineWidth',2)

grid on
title(sprintf('%s Reference - Adaptive Gain', name));
xlabel('Time (s)')
ylabel('Gain')
legend('Kp','Ki','Kd')

%% Error
subplot(3,1,3)

yyaxis left
plot(t,resultFuzzy.e,'b','LineWidth',1.5)
ylabel('Error')

yyaxis right
plot(t,resultFuzzy.de,'m','LineWidth',1.2)
ylabel('dError')

grid on
title(sprintf('%s Reference - Error and dError', name));
xlabel('Time (s)')
legend('Error','dError')

%% Control Signal Breakdown (u, uP, uI, uD)
subplot(4,1,4)
plot(t,resultFuzzy.u,'k','LineWidth',2)
hold on
plot(t,resultFuzzy.uP,'b','LineWidth',1.2)
plot(t,resultFuzzy.uI,'r','LineWidth',1.2)
plot(t,resultFuzzy.uD,'g','LineWidth',1.2)
plot(t,resultPID.u,'m--','LineWidth',1)
grid on
title(sprintf('%s Reference - Control Signal Breakdown (Fuzzy PID)', name));
xlabel('Time (s)')
ylabel('Control effort (N)')
legend('u_{total} (Fuzzy)','u_P','u_I','u_D','u_{total} (PID fixed)')
end