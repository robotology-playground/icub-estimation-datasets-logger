function [time, acc, angvel, rot, mag] = parseInertial(dataDir, timeformat)
if nargin == 1
    timeformat = 'tx';
end

switch timeformat
    case {'rx', 'tx'}
        formatString = '%d %f %f %f %f %f %f %f %f %f %f %f %f %f';
        fid = fopen([dataDir 'data.log']);
        C = textscan(fid, formatString);
        fclose(fid);
        
        time = C{1, 2};
       
        rot = deg2rad(cell2mat(C(1, 3:5)));
        acc = cell2mat(C(1, 6:8));
        angvel = deg2rad(cell2mat(C(1, 9:11)));
        mag = cell2mat(C(1, 12:14));
        
    case 'tx-rx'
        formatString = '%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f';
        fid = fopen([dataDir 'data.log']);
        C = textscan(fid, formatString);
        fclose(fid);
        
        time = C{1, 3};
        
        rot = deg2rad(cell2mat(C(1, 4:6)));
        acc = cell2mat(C(1, 7:9));
        angvel = deg2rad(cell2mat(C(1, 10:12)));
        mag = cell2mat(C(1, 13:15));        
end

% get unique timestamps
[time, ia] = unique(time);
rot = rot(ia, :);
acc = acc(ia, :);
angvel = angvel(ia, :);
mag = mag(ia, :);

end
