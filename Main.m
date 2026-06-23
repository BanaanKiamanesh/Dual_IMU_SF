clear
close all
clc


projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot))

%% Config
config = BuildConfig();

config.SensorMode = 'dual';
config.FilterMode = 'eskf';

%% Simulation
results = RunAHRSSimulation(config);

%% Results
PrintResults(results)
PlotResults(results)