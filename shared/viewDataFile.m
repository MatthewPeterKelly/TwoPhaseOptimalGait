clc; clear; addpath ../computerGeneratedCode; addpath ../Shared;

cd ..;
uiload();
cd Shared

plotSolution(plotInfo);
animation(plotInfo);
