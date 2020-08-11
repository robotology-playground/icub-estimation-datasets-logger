function [time, xyz, rpy, v, omega, a, alpha] = parseGazeboBaseState(dataDir, timeformat)
if nargin == 1
    timeformat = 'tx';
end

switch timeformat
    case {'rx', 'tx'}
        formatString = '%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f';
        fid = fopen([dataDir 'data.log']);
        C = textscan(fid, formatString);
        fclose(fid);
        
        time = C{1, 2};
        
        xyz = cell2mat(C(1, 3:5));
        rpy = cell2mat(C(1, 6:8));  %incoming in radians
        v = cell2mat(C(1, 9:11));
        omega = cell2mat(C(1, 12:14)); %incoming in radians
        a = cell2mat(C(1, 15:17));
        alpha = cell2mat(C(1, 18:20)); %incoming in radians
    case {'tx-rx'}
        formatString = '%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f';
        fid = fopen([dataDir 'data.log']);
        C = textscan(fid, formatString);
        fclose(fid);
        
        time = C{1, 3};
        
        xyz = cell2mat(C(1, 4:6));
        rpy = cell2mat(C(1, 7:9));  %incoming in radians
        v = cell2mat(C(1, 10:12));
        omega = cell2mat(C(1, 13:15)); %incoming in radians
        a = cell2mat(C(1, 16:18));
        alpha = cell2mat(C(1, 19:21)); %incoming in radians
end

% get unique timestamps
[time, ia] = unique(time);
xyz = xyz(ia, :);
rpy = rpy(ia, :);
v = v(ia, :);
omega = omega(ia, :);
a = a(ia, :);
alpha = alpha(ia, :);
end

