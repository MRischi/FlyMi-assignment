clear;
clc;
close all;

%Given information and assumptions (see PDF for more detail)

mass = 10; %kg
weight = mass*9.81; %N
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
h = 0.2; %m
mu = 0.1; %
rho = 1.225; %kg/m^3


%Calculating the minimum force required for the takeoff roll

S = b*c;
v_stall = sqrt((weight)/(0.5*rho*S*max_CL));
phi = (16*h/b)^2/(1+(16*h/b)^2);
AR = (b^2)/S;

D = 0.5*rho*((0.7*v_stall)^2)*S*(TO_CD+phi*(max_CL^2)/(pi*0.80*AR));
L = 0.7*weight;

T = 0.1;
dist = (1.44*(v_stall^2)*mass)/(2*(T-(D+mu*(weight-L))));

% Unefficient algorithm but I didn't know any better
while (abs(dist) > to_distance)
    T = T+0.1;
    dist = (1.44*(v_stall^2)*mass)/(2*(T-(D+mu*(weight-L))));
end

F_max = T; %N


%Calculating air resistance in cruise, power and energy for flight
%(more details in PDF)

energy_takeoff = F_max*to_distance;
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

%Dividing between motor configurations
%1=single, 2=double, 3=triple

motor_vector = 1: 1: 19;
for i = 1:19
    if(M{i, 1}>force)
        motor_vector(i)=1;
    else
        if(M{i, 1}*2>force)
            motor_vector(i)=2;
        else
            motor_vector(i)=3;
        end
    end
end


%Calculating total weight and creating a thrust vector
weight_vector = 1: 1: 19;
thrust_vector = 1: 1: 19;
for i = 1:19
    if(motor_vector(i)==2)
        weight_vector(i) = M{i, 2}*2;
        thrust_vector(i) = M{i, 1}*2;
    else
        if(motor_vector(i)==3)
            weight_vector(i) = M{i, 2}*3;
            thrust_vector(i) = M{i, 1}*3;
        else
            weight_vector(i) = M{i, 2};
            thrust_vector(i) = M{i, 1};
        end
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

for i=1:19
    if (motor_vector(i)==2)
        plot(weight_vector(i), thrust_vector(i), '.','MarkerSize', 10, 'Color', 'r')
    elseif (motor_vector(i)==3)
        plot(weight_vector(i), thrust_vector(i), '.', 'MarkerSize', 10, 'Color', 'g')
    end
end

plot(weight_vector(pos), thrust_vector(pos), '*', 'MarkerSize', 10, 'Color', 'b')

legend('1 motor', '3 motors', '2 motors')