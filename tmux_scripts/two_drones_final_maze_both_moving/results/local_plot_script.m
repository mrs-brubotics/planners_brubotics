clear
close all
clc

%% TODO BRYAN:
% use of matlab classes for this project
% stepPred = 80;%2;

%% Load data from the .mat-file
matFileName = 'matlabData_01.mat'; % [USER INPUT]
data = load(matFileName);
%% axisTimeShift
extra_time_offset = -1.35;
[data, timeAxisStartVal, timeAxisEndVal] = axisTimeShift(data,extra_time_offset);
%% [USER INPUT]: time shift?
% uncomment if you want to manually set the plotted time axis start and end
% values.
% timeAxisStartVal = 0; %[s]
% timeAxisEndVal = 12; %inf; %50; %5 ; [s]  
%% getPlotGraphicsProps
plotGraphicsProps = getPlotGraphicsProps(timeAxisStartVal, timeAxisEndVal);
%% [USER INPUT OPTIONAL]: 
% overwrite custom values if you are not satisfied.
plotGraphicsProps.samplesPlotPrediction = 20; % overwrite the default value
%% [USER INPUT]: Select plot functions. 
% If you are not the author, do not change these functions.
%% [USER INPUT OPTIONAL]: 
% If the existing plot_functions do not satisfy your needs your own custom plot_function in
% /testing_brubotics/generic_matlab_plots/add_your_custom_plot_functions_here

%% 1) Thrust command
plot_thrust(data, plotGraphicsProps)
%% 2) Position 
plot_positions_obstacle(data, plotGraphicsProps)
%% [USER INPUT OPTIONAL]: saving data and figures
% 1) Save all data: 
% - rosbags,
% -.mat file,
% - figures in formats (at least .mat, .eps, .svg required),
% - pictures (i.e. photos of sims / exps), 
% - videos,
% In the google drive, same folder name as your test.
% I added the structure in gdrive --> Bryan the structure similar to tmux_scripts in testing_brubotics. 
% You should use the same structure in the folder thesis A and thesis B.
% That's the place to store all your data. Even if you were able to push it (which won't work all the time or not for all files), put all your data still in the drive in the specific folder.From the google drive you can add the figures which are there to your thesis overleaf.So in the gdrive we see the current state of all simulations / experiments of which (some of them) will go in your thesis.
% Saving figures as vector graphics (.eps, .svg) to keep quality high, .mat to make some manual adjustments.
% Don't forget to save all tabs over multiple uavs. For .fig this is automatically done, but don't forget this for other formats.
% Monthly git storage and bandwidth is not large enough to push them.
% *  For saving tabbed figures the grey background and tabbed text cannot be
% deleted before saving. It is an open issue see, e.g.
% - https://nl.mathworks.com/matlabcentral/answers/195575-change-the-uitabgroup-s-tab-color-or-font-color
% - https://nl.mathworks.com/matlabcentral/answers/476010-how-do-i-copy-a-tabbed-figure-to-clipboard-without-showing-the-tabs
% Workarounds are:
% - A) save the full figure with tabs and when importing in latex cut /
%   crop the top part: https://texblog.org/2012/02/23/crop-figures-with-includegraphics/
% - B) save each tile in the figure seperate by hoivering over the tile's
%   top right corner and click the arrow and save. Afterwards use Latex
%   subfigures as explained https://www.overleaf.com/learn/latex/How_to_Write_a_Thesis_in_LaTeX_(Part_3):_Figures,_Subfigures_and_Tables