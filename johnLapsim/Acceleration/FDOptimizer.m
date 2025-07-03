% This code has been updated to reflect a time-based vehicle model.
% Gearshift times are included, although rudimentary.
% Launch traction is now defined as a maximum A_x, preventing unrealistically hard launches.
% Now includes dyno data for torque curve as a function of RPM.
clear
clc
close all hidden

%% Define Variables - User Adjustable
accel_dist=75; %Meters, acceleration distance
etq_eng = 48; %Engine torque in ft/lbs
%f_drive = 3.700; %Sprocket Ratio
p_drive = 2.11; %primary drive ratio
red_line = 15000; %RPM at red line
tire_d_eng = 16; %tire diameter
mass_car = 213.61; %mass of car in kg
mass_driver = 60; %mass of driver in kg
shift_t = 0.0; %shift time
launch_rpm = 9000; %define driver launch rpm
a_xmax = 1; %maximum tractive G-force that the tires are capable of at launch
trq_poly = readmatrix("torque_polynomial.csv"); %read torque-rpm polynomial from excel sheet

%% Gearing Set Up
gears = 6; %Number of gears in the engine
gratio = zeros(1,gears);
gratio(1) = 2.75; %1st gear ratio
gratio(2) = 2.0;%2nd gear ratio
gratio(3) = 1.67;%3rd gear ratio
gratio(4) = 1.444;%4th gear ratio
gratio(5) = 1.304;%5th gear ratio
gratio(6) = 1.208;%6th gear ratio

%% Define Track
dt = 0.01; %time step size
tmax = 10; %max time of simulation, 10s
steps = tmax/dt; %number of steps in simulation

%% Pre-calculations
etq = etq_eng * 1.3558; %converts nm to ft/lbs
tire_r_eng = tire_d_eng/2; %converts tire diamitre to radius
tire_r=tire_r_eng * 0.0254; %converts tire radius to meters
system_kg = mass_car + mass_driver; %calculates system mass
tire_d = tire_r*2; %calculates tire diamitre in d

%% Intitializing Arrays
dist = zeros(1, steps);
spd = zeros(1,steps);
rpm = zeros(1,steps);
engine_tq = zeros(1,steps);
gearstr = zeros(1,steps);
accelstr = zeros(1,steps);
time = 0:dt:tmax-dt;

%% Initialize Numbers
shift_start = 0; %initialize shift event start time
shift_end = 0; %initialize shift event end time
gear = 1; %initial gear
rpm(1) = launch_rpm;
engine_tq(1) = polyval(trq_poly, launch_rpm); %calculate engine torque at launch
%%Set up array numbers
fd_min = 2.000; %minimum final drive
df = 0.1; %increments of increase between final drive
fd_max = 7; %max final drive
steps_fd = (fd_max-fd_min)/df; %steps between initial and final FD

%% creates array for Final Drive Iterations
final_drive = fd_min:df:fd_max;
final_time = zeros(1,steps_fd+1);
%%Final Drive Iterations

for j=1:steps_fd+1
   %enter code from LapSim_Dyno 
f_drive=final_drive(j);
%% Internal Loop
for i=2:steps
    % calculate engine torque as a function of rpm
    engine_tq(i) = polyval(trq_poly, rpm(i-1));
    % calculate wheel torque and constant acceleration values
    wheel_tq = engine_tq(i)*gratio(gear)*f_drive; %calculates wheel torque
    wheel_f = wheel_tq/tire_r; %calculates wheel force
    accel = wheel_f/system_kg; %calculates acceleration speed in m/s^2

    % check if accel value is greater than the limit of the tires' tractive force, and correct if so
    if accel > (a_xmax * 9.81)
        accel = a_xmax * 9.81;
    end

    if (time(i) >= shift_start) && (time(i) < shift_end) % check for shift event
        spd(i) = spd(i-1);
        rpm(i) = spd(i) * (60*gratio(gear)*f_drive*p_drive)/(2*pi*tire_r); %calculates rpm at given speed
        dist(i) = dist(i-1) + ((spd(i)+spd(i-1))/2) * dt; %calculates distance traveled since last time step
    else
        % calculate speed, rpm, and distance traveled from previous time step
        spd(i) = spd(i-1) + accel*dt; %calculates the speed at incremental timesteps
        rpm(i) = spd(i) * (60*gratio(gear)*f_drive*p_drive)/(2*pi*tire_r); %calculates rpm at given speed
        if rpm(i) > red_line && gear == 6
            rpm(i) = red_line;
            spd(i) = spd(i-1);
        end
        if (gear == 1) && (rpm(i) < launch_rpm)
            rpm(i) = launch_rpm;
        end
        dist(i) = dist(i-1) + ((spd(i)+spd(i-1))/2) * dt; %calculates distance traveled since last time step
    end

    % controls gear shifting by checking rpm vs redline, and iterating a gear counter if needed
    if rpm(i) > red_line && gear ~= 6
        gear = gear+1;
        shift_start = time(i);
        shift_end = time(i) + shift_t;
    end
    
    % store the current gear and acceleration values in an array for plotting
    gearstr(i) = gear;
    accelstr(i) = accel;


    % breaks the loop once the distance value exceeds 75m
    if dist(i) > accel_dist
        break
    end
end


shift_start = 0; %initialize shift event start time
shift_end = 0; %initialize shift event end time
gear = 1; %initial gear
rpm(1) = launch_rpm;
engine_tq(1) = polyval(trq_poly, launch_rpm); %calculate engine torque at launch
final_time(:,j) = time(i);
end

figure
plot(final_drive, final_time)
title('Final Drive Ratio vs. 75-m Acceleration Time')
xlabel('Final Drive Ratio')
ylabel('Elapsed Time (s)')