clear all;
close all;
clc;

Re = 6371.2*1000;

u = load('sim_data_3.mat');

LVLH_Quat = u.out.SS_LVLH_Quat;
R_Quat = u.out.SS_R_Quat;
State = u.out.T_statevec;
Pos_I = State(:,1:3);
Vel_I = State(:,4:6);
ECI2LVLH = u.out.SS_ECI2LVLH.signals.values;

figure;
plot3(Pos_I(:,1), Pos_I(:,2), Pos_I(:,3), 'k');
hold on;    grid on;
sat_pos = plot3(Pos_I(1,1), Pos_I(1,2), Pos_I(1,3), 'k.');
axis equal;
axis([-Re*1.2, Re*1.2, -Re*1.2, Re*1.2, -Re*1.2, Re*1.2]);

C_ECI2LVLH = ECI2LVLH(:,:,1)*1000000;
LVLH_X = quiver3(Pos_I(1,1), Pos_I(1,2), Pos_I(1,3), C_ECI2LVLH(1,1), C_ECI2LVLH(2,1), C_ECI2LVLH(3,1), 1.5,'Color',[0 0 0]);
LVLH_Y = quiver3(Pos_I(1,1), Pos_I(1,2), Pos_I(1,3), C_ECI2LVLH(1,2), C_ECI2LVLH(2,2), C_ECI2LVLH(3,2), 1.5,'Color',[0 0 0]);
LVLH_Z = quiver3(Pos_I(1,1), Pos_I(1,2), Pos_I(1,3), C_ECI2LVLH(1,3), C_ECI2LVLH(2,3), C_ECI2LVLH(3,3), 1.5,'Color',[0 0 0]);

C_ECI2BODY = C_ECI2LVLH*quat2dcm(LVLH_Quat(1,:));
BODY_X = quiver3(Pos_I(1,1), Pos_I(1,2), Pos_I(1,3), C_ECI2BODY(1,1), C_ECI2BODY(2,1), C_ECI2BODY(3,1), 1.5,'Color',[1 0 0]);
BODY_Y = quiver3(Pos_I(1,1), Pos_I(1,2), Pos_I(1,3), C_ECI2BODY(1,2), C_ECI2BODY(2,2), C_ECI2BODY(3,2), 1.5,'Color',[0 1 0]);
BODY_Z = quiver3(Pos_I(1,1), Pos_I(1,2), Pos_I(1,3), C_ECI2BODY(1,3), C_ECI2BODY(2,3), C_ECI2BODY(3,3), 1.5,'Color',[0 0 1]);

str = sprintf("LVLH Attitude Control");
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

for i=1:100:length(State)
    title(str,"FontSize",20);

    set(sat_pos, 'XData', Pos_I(i,1), 'YData', Pos_I(i,2), 'ZData', Pos_I(i,3),'MarkerSize',1.5);

    C_ECI2LVLH = ECI2LVLH(:,:,i)*1000000;
    set(LVLH_X, 'XData', Pos_I(i,1), 'YData', Pos_I(i,2), 'ZData', Pos_I(i,3), ...
                       'UData', C_ECI2LVLH(1,1),'VData', C_ECI2LVLH(2,1),'WData', C_ECI2LVLH(3,1),...
                       'LineWidth',2,'LineStyle','--');
    set(LVLH_Y, 'XData', Pos_I(i,1), 'YData', Pos_I(i,2), 'ZData', Pos_I(i,3), ...
                       'UData', C_ECI2LVLH(1,2),'VData', C_ECI2LVLH(2,2),'WData', C_ECI2LVLH(3,2),...
                       'LineWidth',2,'LineStyle','--');
    set(LVLH_Z, 'XData', Pos_I(i,1), 'YData', Pos_I(i,2), 'ZData', Pos_I(i,3), ...
                       'UData', C_ECI2LVLH(1,3),'VData', C_ECI2LVLH(2,3),'WData', C_ECI2LVLH(3,3),...
                       'LineWidth',2,'LineStyle','--');

    C_ECI2BODY = C_ECI2LVLH*quat2dcm(LVLH_Quat(i,:));
    set(BODY_X, 'XData', Pos_I(i,1), 'YData', Pos_I(i,2), 'ZData', Pos_I(i,3), ...
                       'UData', C_ECI2BODY(1,1),'VData', C_ECI2BODY(2,1),'WData', C_ECI2BODY(3,1));
    set(BODY_Y, 'XData', Pos_I(i,1), 'YData', Pos_I(i,2), 'ZData', Pos_I(i,3), ...
                       'UData', C_ECI2BODY(1,2),'VData', C_ECI2BODY(2,2),'WData', C_ECI2BODY(3,2));
    set(BODY_Z, 'XData', Pos_I(i,1), 'YData', Pos_I(i,2), 'ZData', Pos_I(i,3), ...
                       'UData', C_ECI2BODY(1,3),'VData', C_ECI2BODY(2,3),'WData', C_ECI2BODY(3,3));

    drawnow;

end

ECI2LVLH_euler = zeros(length(LVLH_Quat),3);
ECI2BODY_euler = zeros(length(LVLH_Quat),3);
for i=1:length(LVLH_Quat)
    ECI2LVLH_euler_elem = ECI2LVLH(:,:,i);
    ECI2LVLH_euler_elem_rpy = rotm2eul(ECI2LVLH_euler_elem);
    ECI2LVLH_euler(i,1) = ECI2LVLH_euler_elem_rpy(1);
    ECI2LVLH_euler(i,2) = ECI2LVLH_euler_elem_rpy(2);
    ECI2LVLH_euler(i,3) = ECI2LVLH_euler_elem_rpy(3);

    ECI2BODY_euler_elem = ECI2LVLH_euler_elem*quat2dcm(LVLH_Quat(i,:));
    ECI2BODY_euler_elem_rpy = rotm2eul(ECI2BODY_euler_elem);
    ECI2BODY_euler(i,1) = ECI2BODY_euler_elem_rpy(1);
    ECI2BODY_euler(i,2) = ECI2BODY_euler_elem_rpy(2);
    ECI2BODY_euler(i,3) = ECI2BODY_euler_elem_rpy(3);
end

tout = u.out.tout;
figure;
subplot(3,1,1);
plot(tout,ECI2BODY_euler(:,1),'Color','red','LineStyle','--','LineWidth',1);
hold on; grid on;
plot(tout,ECI2LVLH_euler(:,1),'Color','black','LineStyle','-','LineWidth',1);
legend(["Body","LVLH"]);
xlabel("t [sec]","FontSize",15);
ylabel("angle [rad]","FontSize",15);
title("Roll \phi","FontSize",20);
fontsize(20,"points");
subplot(3,1,2);
plot(tout,ECI2BODY_euler(:,2),'Color','green','LineStyle','--','LineWidth',1);
hold on; grid on;
plot(tout,ECI2LVLH_euler(:,2),'Color','black','LineStyle','-','LineWidth',1);
legend(["Body","LVLH"]);
xlabel("t [sec]","FontSize",15);
ylabel("angle [rad]","FontSize",15);
title("Pitch \theta","FontSize",20);
fontsize(20,"points");
subplot(3,1,3);
plot(tout,ECI2BODY_euler(:,3),'Color','blue','LineStyle','--','LineWidth',1);
hold on; grid on;
plot(tout,ECI2LVLH_euler(:,3),'Color','black','LineStyle','-','LineWidth',1);
legend(["Body","LVLH"]);
xlabel("t [sec]","FontSize",15);
ylabel("angle [rad]","FontSize",15);
title("Yaw \psi","FontSize",20);
fontsize(20,"points");