function [time, jointPos, jointVel, jointAcc, jointTrqs, dutyCycle, currents] = parseStateExt(dataDir, formatString, n_joints, timeformat)
if nargin == 3
    timeformat = 'tx';
end

switch timeformat
    case {'rx', 'tx'}
        fid = fopen([dataDir 'data.log']);
        C = textscan(fid, formatString);
        fclose(fid);
        
        time = C{1, 2};
        
        jointPos = deg2rad(cell2mat(C(1, 3:3+n_joints-1)));
        jointVel = deg2rad(cell2mat(C(3+n_joints: 3+(2*n_joints)-1)));
        jointAcc = deg2rad(cell2mat(C(3+(2*n_joints): 3+(3*n_joints)-1)));
        jointTrqs = cell2mat(C(1, 3+6*n_joints: 3+(7*n_joints)-1));
        dutyCycle = cell2mat(C(1, 3+7*n_joints: 3+(8*n_joints)-1));
        currents = cell2mat(C(1, 3+8*n_joints: 3+(9*n_joints)-1));
    case 'tx-rx'
        fid = fopen([dataDir 'data.log']);
        C = textscan(fid, formatString);
        fclose(fid);
        
        time = C{1, 3};
        
        jointPos = deg2rad(cell2mat(C(1, 4:4+n_joints-1)));
        jointVel = deg2rad(cell2mat(C(4+n_joints: 4+(2*n_joints)-1)));
        jointAcc = deg2rad(cell2mat(C(4+(2*n_joints): 4+(3*n_joints)-1)));
        jointTrqs = cell2mat(C(1, 4+6*n_joints: 4+(7*n_joints)-1));
        dutyCycle = cell2mat(C(1, 4+7*n_joints: 4+(8*n_joints)-1));
        currents = cell2mat(C(1, 4+8*n_joints: 4+(9*n_joints)-1));
end

% get unique timestamps
[time, ia] = unique(time);
jointPos = jointPos(ia, :);
jointVel = jointVel(ia, :);
jointAcc = jointAcc(ia, :);
jointTrqs = jointTrqs(ia, :);
dutyCycle = dutyCycle(ia, :);
currents = currents(ia, :);
end
