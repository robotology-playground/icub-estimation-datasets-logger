clear
clc
close all

%%  options
parseImgsMetaData = false;
use_sgolay = false;
sgolay_smooth_or_deriv = 'deriv'; % smooths incoming Gazebo velcoticy/acceleration signals or takes sgolay derivatives of positions/velocities instead of directly using Gazebo velocites/accelerations
sim = false;
time_format = 'tx-rx';  %tx-rx|rx|tx
if sim
    save_file_name = 'sim-walking-base-estimation.mat';
else
    save_file_name = 'walking-base-estimation.mat';
end
%% parse image names
if (parseImgsMetaData && ~sim)
    imgDataDir ='dumper/depthCamera/rgbImage/';
    
    [imgTime, imgFileNameArray] = parseRGBImages(imgDataDir, time_format);
    disp('Finished parsing images...');
end

%% datadirs
if (sim)
    waistDataDir = 'dumper/icubSim/waist/inertial/';
    leftFootDataDir = 'dumper/icubSim/left_foot/cartesianEndEffectorWrench_o/';
    rightFootDataDir = 'dumper/icubSim/right_foot/cartesianEndEffectorWrench_o/';
    stateExtDataDir = 'dumper/icubSim/all_joints/stateExt_o/';
    simBaseStateDataDir = 'dumper/icubSim/floating_base/state_o/';
    
    lfootstateDataDir = 'dumper/icubSim/l_foot/state_o/';
    rfootstateDataDir = 'dumper/icubSim/r_foot/state_o/';
else
    waistDataDir = 'dumper/icub/waist/inertial/';
    leftFootDataDir = 'dumper/icub/left_foot/cartesianEndEffectorWrench_o/';
    rightFootDataDir = 'dumper/icub/right_foot/cartesianEndEffectorWrench_o/';
    stateExtDataDir = 'dumper/icub/all_joints/stateExt_o/';
end

%% parse gazebo base state
if (sim)
    [simbaseTime, simBasePos, simBaseRot, simBaseLinVel, simBaseAngVel, simBaseLinAcc, simBaseAngAcc] = parseGazeboBaseState(simBaseStateDataDir, time_format);
    [simlFootTime, simlFootPos, simlFootRot, simlFootLinVel, simlFootAngVel, simlFootLinAcc, simlFootAngAcc] = parseGazeboBaseState(lfootstateDataDir, time_format);
    [simrFootTime, simrFootPos, simrFootRot, simrFootLinVel, simrFootAngVel, simrFootLinAcc, simrFootAngAcc] = parseGazeboBaseState(rfootstateDataDir, time_format);
    
    if use_sgolay
        polyOrder = 5;
        windowSize = 151;
        dt = mean(diff(simbaseTime));
        [~, g] = sgolay(polyOrder, windowSize);
        smoothedSimBaseAngVel = zeros(length(simBaseAngVel), 3);
        smoothedSimBaseLinVel = zeros(length(simBaseLinVel), 3);
        smoothedSimBaseLinAcc = zeros(length(simBaseLinVel), 3);
        smoothedSimBaseAngAcc = zeros(length(simBaseAngVel), 3);
        
        if strcmp(sgolay_smooth_or_deriv, 'smooth')
            for idx = 1:3
                smoothedSimBaseAngVel(:, idx) = conv(simBaseAngVel(:, idx), factorial(0)/(-dt)^0 * g(:, 1), 'same');
                smoothedSimBaseLinVel(:, idx) = conv(simBaseLinVel(:, idx), factorial(0)/(-dt)^0 * g(:, 1), 'same');
                smoothedSimBaseAngAcc(:, idx) = conv(smoothedSimBaseLinAcc(:, idx), factorial(0)/(-dt)^0 * g(:, 1), 'same');
                smoothedSimBaseLinAcc(:, idx) = conv(smoothedSimBaseAngAcc(:, idx), factorial(0)/(-dt)^0 * g(:, 1), 'same');
            end
        elseif strcmp(sgolay_smooth_or_deriv, 'deriv')
            for idx = 1:3
                smoothedSimBaseAngVel(:, idx) = conv(simBaseAngVel(:, idx), factorial(0)/(-dt)^0 * g(:, 1), 'same');
                smoothedSimBaseLinVel(:, idx) = conv(simBaseLinVel(:, idx), factorial(0)/(-dt)^0 * g(:, 1), 'same');
                smoothedSimBaseAngAcc(:, idx) = conv(simBaseAngVel(:, idx), factorial(1)/(-dt)^1 * g(:, 2), 'same');
                smoothedSimBaseLinAcc(:, idx) = conv(simBaseLinVel(:, idx), factorial(1)/(-dt)^1 * g(:, 2), 'same');
            end
        end
        
        simbaseTime = simbaseTime(windowSize:end-windowSize, :);
        simBasePos = simBasePos(windowSize:end-windowSize, :);
        simBaseRot = simBaseRot(windowSize:end-windowSize, :);
        simBaseLinVel = smoothedSimBaseLinVel(windowSize:end-windowSize, :);
        simBaseAngVel = smoothedSimBaseAngVel(windowSize:end-windowSize, :);
        simBaseLinAcc = smoothedSimBaseLinAcc(windowSize:end-windowSize, :);
        simBaseAngAcc = smoothedSimBaseAngAcc(windowSize:end-windowSize, :);
    end
    disp('Finished parsing Gazbeo base state data...');
end

%% parse root link IMU
[imuTime, imuAcc, imuOmega, imuRot, imuMag] = parseInertial(waistDataDir, time_format);
disp('Finished parsing inertial data...');

%% parse foot wrenches
[lfTime, lfWrench] = parseFootWrench(leftFootDataDir, time_format);
[rfTime, rfWrench] = parseFootWrench(rightFootDataDir, time_format);

disp('Finished parsing wrenches data...');

%% parse joint states
if (sim)
    all_jointsPortJointOrdering = ["neck_pitch","neck_roll","neck_yaw", ...
        "torso_yaw","torso_roll","torso_pitch", ...
        "l_shoulder_pitch","l_shoulder_roll","l_shoulder_yaw", ...
        "l_elbow","l_wrist_prosup","l_wrist_pitch","l_wrist_yaw", ...
        "r_shoulder_pitch","r_shoulder_roll","r_shoulder_yaw", ...
        "r_elbow","r_wrist_prosup","r_wrist_pitch","r_wrist_yaw", ...
        "l_hip_pitch","l_hip_roll","l_hip_yaw","l_knee","l_ankle_pitch","l_ankle_roll", ...
        "r_hip_pitch","r_hip_roll","r_hip_yaw","r_knee","r_ankle_pitch","r_ankle_roll"];

else
    all_jointsPortJointOrdering = ["neck_pitch","neck_roll", ...
        "neck_yaw", ...
        "torso_yaw","torso_roll","torso_pitch", ...
        "r_shoulder_pitch","r_shoulder_roll","r_shoulder_yaw", "r_elbow",...
        "r_wrist_prosup","r_wrist_pitch","r_wrist_yaw","r_hand_finger", ...
        "r_thumb_oppose","r_thumb_proximal","r_thumb_distal", "r_index_proximal",...
        "r_index_distal","r_middle_proximal","r_middle_distal","r_pinky", ...
        "l_shoulder_pitch","l_shoulder_roll","l_shoulder_yaw", "l_elbow",...
        "l_wrist_prosup","l_wrist_pitch","l_wrist_yaw", "l_hand_finger",...
        "l_thumb_oppose","l_thumb_proximal","l_thumb_distal", "l_index_proximal",...
        "l_index_distal","l_middle_proximal","l_middle_distal","l_pinky", ...
        "l_hip_pitch","l_hip_roll","l_hip_yaw","l_knee", ...
        "l_ankle_pitch","l_ankle_roll", ...
        "r_hip_pitch","r_hip_roll","r_hip_yaw","r_knee",...
        "r_ankle_pitch","r_ankle_roll"];
end
nr_joints_remote = length(all_jointsPortJointOrdering);
stateExtString = prepareStateExtFormatString(nr_joints_remote, sim, time_format);
[allJointsTime, allJointsPos, allJointsVel] = parseStateExt(stateExtDataDir, stateExtString, nr_joints_remote, time_format);

disp('Finished parsing stateExt data...');

%% get estimator relevant joint states
estimatorJointsOrdering = ["neck_pitch", "neck_roll", "neck_yaw", ...
    "torso_pitch", "torso_roll", "torso_yaw",  ...
    "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw",  ...
    "l_elbow",  ...
    "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", ...
    "r_elbow",  ...
    "l_hip_pitch", "l_hip_roll", "l_hip_yaw",  ...
    "l_knee", "l_ankle_pitch", "l_ankle_roll",  ...
    "r_hip_pitch", "r_hip_roll", "r_hip_yaw",  ...
    "r_knee", "r_ankle_pitch", "r_ankle_roll"];
nr_joints_est = length(estimatorJointsOrdering);
estimatorJointsTime = allJointsTime;
estimatorJointsPos = zeros(size(estimatorJointsTime, 1), nr_joints_est);
estimatorJointsVel = zeros(size(estimatorJointsTime, 1), nr_joints_est);
for est = 1 : length(estimatorJointsOrdering)
    for all = 1 : length(all_jointsPortJointOrdering)
        if (strcmp(all_jointsPortJointOrdering(all), estimatorJointsOrdering(est)))
            estimatorJointsPos(:, est) = allJointsPos(:, all);
            estimatorJointsVel(:, est) = allJointsVel(:, all);
            disp(["Copying ", all_jointsPortJointOrdering(all), " at all_index", num2str(all), "to est_index", num2str(est)])
            break;
        end
    end
end

if use_sgolay
    polyOrder = 3;
    windowSize = 51;
    dt = mean(diff(estimatorJointsTime));
    [~, g] = sgolay(polyOrder, windowSize);
    
    diffjointsVel = zeros(length(estimatorJointsTime), nr_joints_est);
    
    if strcmp(sgolay_smooth_or_deriv, 'smooth')
        for idx = 1:nr_joints_est
            diffjointsVel(:, idx) = conv(estimatorJointsVel(:, idx), factorial(0)/(-dt)^0 * g(:, 1), 'same');
        end        
    elseif strcmp(sgolay_smooth_or_deriv, 'deriv')
        for idx = 1:nr_joints_est
            diffjointsVel(:, idx) = conv(estimatorJointsPos(:, idx), factorial(1)/(-dt)^1 * g(:, 2), 'same');
        end
    end
        
    estimatorJointsTime = estimatorJointsTime(windowSize:end-windowSize, :);
    estimatorJointsPos = estimatorJointsPos(windowSize:end-windowSize, :);
    estimatorJointsVel = diffjointsVel(windowSize:end-windowSize, :);
end
disp('Finished parsing kinematics data...');

%% clear unwanted vars and save workspace into mat
vars = {'all', 'all_jointsPortJointOrdering', 'allJointsPos', 'allJointsVel', 'allJointsTime', ...
    'est', 'imgVisualize', 'nr_joints_remote', 'stateExtString', 'minStartTime', ...
    'waistDataDir','leftFootDataDir','rightFootDataDir','stateExtDataDir', 'imgDataDir', 'systemClockIsUsed', ...
    'dt', 'polyOrder', 'windowSize', 'halfsize', 'smoothedSimBaseAngAcc', 'smoothedSimBaseAngVel', ...
    'smoothedSimBaseLinAcc', 'smoothedSimBaseLinVel', 'diffjointsVel', ...
    'parseImgsMetaData', 'sgolay_smooth_or_deriv', 'use_sgolay', 'startTimes'};
clear(vars{:})
clear vars

if (sim)    
    clear 'simBaseStateDataDir';
    clear 'lfootstateDataDir';
    clear 'rfootstateDataDir';
end
clear sim;

save(save_file_name, '-v7.3');
disp(['Saving workspace to ' save_file_name ' ... done!']);
