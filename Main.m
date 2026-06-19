clear
close all
clc

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot))

%% Param Declaration and Run
config = BuildConfig();

results = RunAHRSSimulation(config);

%% Results
PrintResults(results)
PlotResults(results)
