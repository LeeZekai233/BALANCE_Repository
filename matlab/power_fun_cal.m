% clc;
% clear;

% syms k1 k2 k3 w P bT_gain Vmax Kv Pmax T;

T = 0.741.*(Motor6020_Encoder2current*3/16384);
w = ChassisHeading_Speed_Fdb2/6;
P = judge_rece_mesgpower_heat_datachassis_power;



% err = 2*Vmax;
% T = bT_gain + err*Kv;
% f = k1*(T^2) + (w*T/9.55) + k2*(w^2) + k3 - Pmax
% Vmax = solve(f,Vmax)
% matlabFunction(Vmax,'File','Vmax_cal');


input_power = P;

machine_power = (w .* T) / 9.550;

% figure;
% plot(linspace(1,size(machine_power,1)/500,size(machine_power,1)),T);
% hold on;
% plot(linspace(1,size(machine_power,1)/500,size(machine_power,1)),motor_chassis0given_current);
% legend('Give current', 'Given current'); 
% figure;



figure;
plot(linspace(1,size(machine_power,1)/500,size(machine_power,1)),machine_power);
hold on;
plot(linspace(1,size(machine_power,1)/500,size(machine_power,1)),input_power);
legend('Machine power', 'Input power'); 




Pother = input_power - machine_power;
g = fittype('k1*w^2+k2*T.^2+c','independent',{'w','T'}, ...
    'dependent','Pother','coefficients',{'k1','k2','c'});
myfit = fit([w,T],Pother,g);

 Pre_Pother = 0.0003723  *w.^2+31.99 *T.^2  +1.304;


figure;
plot(linspace(1,size(Pre_Pother,1),size(Pre_Pother,1)),Pre_Pother);
hold on;
plot(linspace(1,size(Pre_Pother,1),size(Pre_Pother,1)),Pother);
predicte_power = machine_power + Pre_Pother;
legend('predicte other power','other power'); 


figure;
plot(linspace(1,size(input_power,1)/500,size(input_power,1)),input_power);
hold on;
plot(linspace(1,size(predicte_power,1)/500,size(predicte_power,1)),predicte_power);
legend('real power', 'predicte power'); 

