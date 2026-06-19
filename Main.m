clear
close all
clc

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot))

%% Config and Simulation
config = BuildConfig();
config.SensorMode = 'single';

results = RunAHRSSimulation(config);

%% Show Results
PrintResults(results)
PlotResults(results)