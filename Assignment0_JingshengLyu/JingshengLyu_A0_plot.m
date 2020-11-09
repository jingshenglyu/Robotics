%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TU Berlin robotics WS2020/2021 %
%                                %
% Author: Jingsheng Lyu          %
%                                %
% Lab Assignment #0              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
subplot(2,1,1)
plot(C(1:500,1),C(1:500,2),'LineWidth',1.5)
h=legend("without damper")
xlabel('Time(s)')
ylabel('Position(m)')
title('Position Trajectory of the object without damper')

subplot(2,1,2)
plot(C(1:500,1),C(1:500,3),'LineWidth',1.5)


%set basic info
h=legend("without damper")
xlabel('Time(s)')
ylabel('Velocity(m/s)')
title('Velocity Trajectory of the object without damper')

figure;grid on
subplot(2,1,1)
plot(D(1:500,1),D(1:500,2),'LineWidth',1.5)
h=legend("with damper")
xlabel('Time(s)')
ylabel('Position(m)')
title('Position Trajectory of the object with damper')


subplot(2,1,2)
plot(D(1:500,1),D(1:500,3),'LineWidth',1.5)


%set basic info
h=legend("with damper")
xlabel('Time(s)')
ylabel('Velocity(m/s)')
title('Velocity Trajectory of the object with damper')
