clc
clear
close all
%%

timeFormat = 'tx-rx';  %tx-rx|rx|tx
viconDataDir='dumper/transformServer/transforms_o/';
viconRootString='Vicon_ROOT';
viconParser = YarpViconParser(viconDataDir, timeFormat, viconRootString);
viconParser = viconParser.parseViconLog();
viconParser.printMetaDataMap();

%%
save_file_name = 'vicon-marker-traj.mat';
subjectName = 'base-mount-ext-trial';
markerNames = {'origin', ...
               'yzmarker', ...
               'zaxismarker', ...
               'base_link4', ...
               'base_link5', ...
               'base_link6', ...
               'base_link7', ...
               'base_link8'};
nrMarkers = length(markerNames);

for idx = 1:length(markerNames)
    markerName = markerNames{idx};
    markerTraj = viconParser.getMarkerTrajectory(subjectName, markerName);
    time = genvarname(['marker', num2str(idx), 'time']);
    xyz = genvarname(['marker', num2str(idx), 'xyz']);
    eval([time '= markerTraj.rxTime;']);
    eval([xyz '= markerTraj.xyz;']);
end

vars = {'idx', 'markerName', 'markerNames', ...
    'markerTraj', 'subjectName', 'time', ...
    'timeFormat', 'viconDataDir', 'viconParser', 'xyz'};
clear(vars{:});

%%
clear vars
save(save_file_name, '-v7.3');
disp(['Saving workspace to ' save_file_name ' ... done!']);
