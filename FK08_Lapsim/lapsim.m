clc
clear
close all hidden

%% Import Car/Data
fprintf('Select a vehicle file.\n')
carname = uigetfile('*.csv');
disp('Selected: ' + carname)

fprintf('Select a track file. \n')
trackname = uigetfile('*csv');
disp('Selected: ' + trackname)
