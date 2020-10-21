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

%% options
save_markers = false;
save_segment_traj = true;

%%
subjectName = 'base-mount-full';
save_file_name = [subjectName '-traj.mat'];
%% relevant marker names for base-mount-rear
if (strcmp(subjectName, 'base-mount-rear'))
markerNames = {'base_link8', ...
               'base_link13', ...
               'base_link12', ...
               'base_link11', ...
               'base_link10', ...
               'base_link9'};
end           
%% relevant marker names for base-mount-ext-trial     
if (strcmp(subjectName, 'base-mount-ext-trial'))
markerNames = {'origin', ...
               'yzmarker', ...
               'zaxismarker', ...
               'base_link4', ...
               'base_link5', ...               
               'base_link7'};
end
%% relevant marker names for base-mount-trial   
if (strcmp(subjectName, 'base-mount-trial'))
markerNames = {'origin', ...
               'yzplanemarker', ...
               'zaxismarker', ...
               'base_link4', ...
               'base_link5', ...
               'base_link6', ...
               'base_link7', ...
               'base_link8'};    
end

%% save marker trajctories
if save_markers
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
    'timeFormat', 'viconDataDir', 'viconParser', 'xyz', 'save_markers'};
clear(vars{:});


clear vars
save(save_file_name, '-v7.3');
disp(['Saving workspace to ' save_file_name ' ... done!']);

end


%% 
if save_segment_traj
    subject_name = 'base-mount-full';
    segment_name = 'base-mount-full';
    seg_traj = viconParser.getSegmentTrajectory(subject_name, segment_name);
    data.simTime = seg_traj.rxTime;
    data.simBasePos = seg_traj.xyz;
    data.simBaseRot = zeros(size(seg_traj.xyz));
    data.simBaseLinVel = zeros(size(seg_traj.xyz));
    data.simBaseAngVel = zeros(size(seg_traj.xyz));
    for idx = 1:length(seg_traj.quat)
        data.simBaseRot(idx, :) = Core.rot2rpy(Core.quat2rot(seg_traj.quat(idx, :)));
    end
    
    subject_name = 'left-foot';
    segment_name = 'left-foot';
    seg_traj = viconParser.getSegmentTrajectory(subject_name, segment_name);
    data.simlFootTime =  seg_traj.rxTime;
    simlsolePosTraj = seg_traj.xyz;
    simlsoleRotTraj = zeros(size(seg_traj.xyz));
    for idx = 1:length(seg_traj.quat)
        simlsoleRotTraj(idx, :) =  Core.rot2rpy(Core.quat2rot(seg_traj.quat(idx, :)));
    end
    
    subject_name = 'right-foot';
    segment_name = 'right-foot';
    seg_traj = viconParser.getSegmentTrajectory(subject_name, segment_name);
    data.simrFootTime =  seg_traj.rxTime;
    simrsolePosTraj = seg_traj.xyz;
    simrsoleRotTraj = zeros(size(seg_traj.xyz));
    for idx = 1:length(seg_traj.quat)
        simrsoleRotTraj(idx, :) =  Core.rot2rpy(Core.quat2rot(seg_traj.quat(idx, :)));
    end
    
    
    vars = {'idx', 'subject_name', 'segment_name', ...
    'seg_traj', 'save_segment_traj', 'viconParser'};
    clear(vars{:});
    
    clear vars
    save(save_file_name, '-v7.3');
    disp(['Saving workspace to ' save_file_name ' ... done!']);
end
