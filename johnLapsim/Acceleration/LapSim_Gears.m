% This code will use a basic kinematic equations to "simulate" the acceleration event at competition.
% Now includes instantaneous gear shifting.
clear
clc
%clf
%% Define Variables
x=75; %Meters, acceleration distance
etq_eng = 48; %Engine torque in ft/lbs
f_drive = 3.70; %Sprocket Ratio
tire_d_eng = 16; % Tire diamitre
mass_car = 213.61; %mass of car in kg
mass_driver = 60; %mass of driver in kg
p_drive = 2.11; %primary drive ratio
red_line = 16000; %RPM at red line
gear = 1; %Initial gear
shift_t = 0.1; %shift time
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
dx = 0.01; %step size
nx = x/dx; %number of steps to travel distance

%% Pre-calculations
etq = etq_eng * 1.3558; %converts nm to ft/lbs
tire_r_eng = tire_d_eng/2; %converts tire diamitre to radius
tire_r=tire_r_eng * 0.0254; %converts tire radius to meters
system_kg = mass_car + mass_driver; %calculates system mass
tire_d = tire_r*2; %calculates tire diamitre in d



%% Intitializing Arrays
dist = 0:dx:x;
spd = zeros(1,nx+1);
rpm = zeros(1,nx+1);
time = zeros(1,nx+1);
gearstr = zeros(1,nx+1);

%%Array 
for i=2: nx+1
    wheel_tq = etq*gratio(gear)*f_drive; %calculates wheel torque
    wheel_f = wheel_tq/tire_r; %calculates wheel force
    accel = wheel_f/system_kg; %calculates acceleration speed in m's^2

    spd(i) = sqrt((spd(i-1)^2)+(2*accel*dx)); %Calculates the speed at incremental distance
    rpm(i) = spd(i) * (60*gratio(gear)*f_drive*p_drive)/(2*pi*tire_r); %calculates rpm at given speed
    time(i) = time(i-1)+(spd(i)-spd(i-1))/accel; %calculates time to reach speed and rpm
    if rpm(i)> red_line
        gear = gear+1;
    end
    gearstr(i) = gear;
end
%% Final Calculation

fprintf('The acceleration time was: %2.2f seconds.\nThe final speed was: %3.2f meters/second.\n',time(nx+1), spd(nx+1))


plot(time, rpm)
