clear
close all
clc


projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot))

%% Config
config = BuildConfig();

sensorMode = 'single';
filterMode = 'mahony';

%% Simulation
results = RunAHRSSimulation(config);

%% Results
PrintResults(results)
PlotResults(results)