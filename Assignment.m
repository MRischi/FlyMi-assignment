clear;
clc;
close all;

%Given information and assumptions (see PDF for more detail)

weight = 10*9.81; %N
structural_weight = 0.5*weight;
payload = 1.25*9.81; %N
to_distance = 14.1; %m
flight_time = 900; %s
max_speed = 25; %m/s
max_E = 25.5;
max_CL = 1.46;
TO_CD = 0.061;
b = 3.7; %m
c = 0.15; %m
rho = 1.225; %kg/m^3


%Calculating the minimum force required for the takeoff roll

S = b*c;
v_stall = sqrt((weight)/(0.5*rho*S*max_CL));

F_drag_TO = 0.5*rho*(v_stall^2)*S*TO_CD; %N
F_TO_roll = (0.5*(v_stall^2)*10)/(to_distance); %N

F_max = F_drag_TO;

%Checking which force is greater
if (F_TO_roll>F_drag_TO)
    F_max = F_TO_roll;
end

%Calculating air resistance in cruise, power and energy for flight
%(more details in PDF)

energy_takeoff = F_TO_roll*to_distance; %conservative estimation of 3s for take off
D_cruise = weight/max_E;
P_cruise = D_cruise*max_speed; %W
energy_required = P_cruise*flight_time + energy_takeoff; %J
energy_total = energy_required + 0.4*energy_required;

%Converting energy into Wh
energy_Wh = energy_total/3600; %Wh

%Calculating weight for motors

battery_weight = 0.321; %kg
weight_allowed = ((weight-structural_weight-payload)/9.81)-battery_weight;

%Converting force units

force = F_max/9.81; %kgf

%Working on the motor table
M = readtable('Motors.dat');

%Dividing between single motor or double motor configurations
%0=single, 1=double

motor_vector = 1: 1: 19;
for i = 1:19
    if(M{i, 1}>force)
        motor_vector(i)=0;
    else
        motor_vector(i)=1;
    end
end

%Calculating total weight and creating a thrust vector
weight_vector = 1: 1: 19;
thrust_vector = 1: 1: 19;
for i = 1:19
    if(motor_vector(i))
        weight_vector(i) = M{i, 2}*2;
        thrust_vector(i) = M{i, 1}*2;
    else
        weight_vector(i) = M{i, 2};
        thrust_vector(i) = M{i, 1};
    end
end

%Finding the lowest weight configuration
lowest = weight_vector(1);
pos = 1;
for i=1:19
    if(lowest>weight_vector(i))
        lowest = weight_vector(i);
        pos = i;
    end
end

%Visualizing the results

scatter(weight_vector, thrust_vector);
hold on;
title('Selected UAV motors on the thrust-weight graph')
xlabel('Weight (kg)')
ylabel('Thrust (kgf)')

plot(weight_vector(pos), thrust_vector(pos), '.', 'MarkerSize', 10)

highest_ratio = thrust_vector(1)/weight_vector(1);
pos2 = 1;
for i = 1:19
    ratio = thrust_vector(i)/weight_vector(i);
    if(ratio>highest_ratio)
        highest_ratio = ratio;
        pos2 = i;
    end
end

plot(weight_vector(pos2), thrust_vector(pos2), '.', 'MarkerSize', 10, 'Color', 'g')

legend('', 'Lowest weight solution', 'Highest T/W ratio')