clear
clc

%% %% CAR GENERATOR (Basic) TOP FILE
%% Code written by Eli Cossel 
% and inspired heavily by John Psyck's Car Gen Matlab documentation.
% 
% This file will call several functions and allow users to punch in their
% own car data. This includes variability in:
% 
% Aerodynamics
% Cl, Cd, Frontal Area, Aero Mass
%
% Suspension
% Diver Mass, Car Mass, Fuel Mass, 

%% Aerodynamics
prompt = {'Coefficient of Lift (CL)','Coefficient of Drag (CD)','Frontal Area','Aero Mass'};
dlgtitle = 'Aerodynamics Inputs';
dims = [1, 35];
definput = {'0.24','0.55', '0.75', '10'}; % Defaults from John's Code
answer = inputdlg(prompt,dlgtitle,dims,definput);

CL = str2double(answer(1));
CD = str2double(answer(2));
frontalArea = str2double(answer(3));
aeroMass = str2double(answer(4));

%% Suspension
prompt = {'Driver Mass', 'Car Mass (w/o fuel)', 'Fuel Mass', 'Tire Diameter (in)', 'Tire Friction Coefficeint',
    'Swept Rotor Diameter (in)', 'Brake Caliper Area (in^2)', 'Brake Caliper Pistons', 'Max. Braking Pressure (PSI)', 'Brake Pad Friction Coefficient'};
dlgtitle = 'Suspension Inputs';
dims = [1, 35];
definput = {'60','154','13.6','16','1.4','6.6','0.79','2','2000','0.5'}; % Defaults from John's Code
answer = inputdlg(prompt,dlgtitle,dims,definput); % definput can be used to get default inputs once I have imported data

driverMass = str2double(answer(1));
carMass = str2double(answer(2));
fuelMass = str2double(answer(3));
totalMass = driverMass + carMass + fuelMass; % Without aero and engine masses
tireDiameter = str2double(answer(4)) * 0.0127; % Converted from in to m
tireFrictionCoeff = str2double(answer(5));
sweptRotorDiameter = str2double(answer(6));
brakeCaliperArea = str2double(answer(7));
brakeCaliperPistons = str2double(answer(8));
maxBrakingPressure = str2double(answer(9));
brakePadFrictionCoeff = str2double(answer(10));

%% Powertrain / Engine
engine_filename = 'cbr_dat.csv';
engine_data = readmatrix(engine_filename);
rpmLog = engine_data(:,1);
maxRpm = max(rpmLog);
torqueLog = engine_data(:,2);
gears = engine_data(1:6,3);
clutchRatio = engine_data(1,4);
sprocketRatio = engine_data(1,5);
weightKg = engine_data(1,6);
totalMass = weightKg + totalMass;
shiftTime = engine_data(1,7);
coeffRolling = engine_data(1,8);

%% Establish Constants
g = 9.81; % gravity in m/s^2
rho = 1.293; % air density at sea level kg/m^3
padForceMax = brakePadFrictionCoeff * maxBrakingPressure * brakeCaliperArea * brakeCaliperPistons;
brakeForceMax = 4 * padForceMax * ( sweptRotorDiameter / ( ( tireDiameter / 2 ) * 0.0127 ) );

%% Torque Curve
figure
hold on
plot(rpmLog,torqueLog);
xlabel("RPM")
ylabel("Torque [ftlbs]")
hold off

%% Normal Forces
maxVelocity = (maxRpm / (sprocketRatio * clutchRatio * gears(6))) * ((pi * tireDiameter) / 60); % maximum velocity due to gearing
velocity = linspace(0,maxVelocity);
aeroForce = CL * (1/2) * rho * velocity.^2 * frontalArea;
massForce = g * (totalMass + aeroMass);
totalNormalForce = aeroForce + massForce;

%figure
%hold on
%plot(velocity,totalNormalForce);
%xlabel("Velocity [m/s]")
%ylabel("Normal Force [N]")
%hold off

%% Drag (D=(1/2)*rho*Cd*v^2*S)
drag = CD * (1/2) * rho * velocity.^2 * frontalArea;

%% Forward Tractive Forces
rpmTable = zeros(numel(velocity),numel(gears));
tracTable = zeros(numel(velocity),numel(gears));
fTrac = zeros(numel(velocity),numel(gears));
for i = 1:numel(gears)   
    for j = 1:numel(velocity)
        rpmTable(j,i) = (60 * velocity(j) / (pi * tireDiameter)) * sprocketRatio * clutchRatio * gears(i); % loop fills all 6 gear accel rows with their corresponding engine rpm
        tracTable(j,i) = interp1(rpmLog,torqueLog,rpmTable(j,i),'spline')*(sprocketRatio * gears(i))/(tireDiameter / 2);  % calculate forward tractive force given an rpm
         if tracTable(j, i) > (totalNormalForce(i) * tireFrictionCoeff) % check if tractive drive force is greater than available traction. if so, clamp value.
            tracTable(j, i) = totalNormalForce(i) * tireFrictionCoeff;
        end
        if rpmTable(j, i) > maxRpm % delete all values where rpm is invalid (over redline)
            rpmTable(j, i) = NaN;
            tracTable(j, i) = NaN;
        end
        if tracTable(j, i) == max(tracTable(j, 1:numel(gears))) % store best gear value in tractive force table
            fTrac(j, 2) = i;
        end
    end
end

%% Tractive Force Graph
figure
gearLegend = cell(numel(gears) + 1,1);
for i = 1:numel(gears)
    plot(velocity, tracTable(:,i))
    gearLegend{i}=append('Gear: ', num2str(i));
    hold on
end
gearLegend{numel(gears) + 1} = 'Available Traction';
plot(velocity, totalNormalForce .* tireFrictionCoeff)
gearLegend{numel(gears) + 2} = 'Drag Forces';
plot(velocity, drag);
legend(gearLegend);
title('Tractive Forces Against Velocity');
xlim([0,max(velocity)]);
xlabel('Speed [m/s]');
ylabel('Tractive Forces [N]');
hold off

%% Rearward Tractive Forces
brakeForce = zeros(numel(velocity),1);
for i = 1:numel(velocity)
    if brakeForceMax > totalNormalForce * tireFrictionCoeff % check to see if braking is traction limited
        brakeForce(i) = (totalNormalForce(i) * tireFrictionCoeff) + drag(i); % set brake force to traction limit if needed
    else
        brakeForce(i) = brakeForceMax + drag(i); % set brake force to maximum if not traction limited
    end
end 

%% Accel/Decel Table
ax = zeros(numel(velocity),5);
ax(:,1) = velocity;
ax(:,2) = totalNormalForce;
for i = 1:numel(velocity)
    ax(i,3) = (fTrac(i,1) - drag(i)) / totalMass; % acceleration
    ax(i,4) = (brakeForce(i) + drag(i)) / totalMass; % deceleration
end
ax(:,5) = fTrac(:,2); % optimum gear based on velocity

%% Max/Min Accel/Decel Graphs
figure
plot(ax(:,1),ax(:,3))
hold on
plot(ax(:,1),ax(:,4));
title('Acceleration Capabilities')
legend("Acceleration Tractive","Deceleration Tractive")
xlim([0,max(velocity)])
xlabel('Velocity [m/s]');
ylabel('Acceleration [m/s^2]');

%% Generate Final Car File for Simulation




disp("Done!");