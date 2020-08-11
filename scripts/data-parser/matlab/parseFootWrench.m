function [time, footWrench] = parseFootWrench(dataDir, timeformat)
if nargin == 1
    timeformat = 'tx';
end

switch timeformat
    case {'rx', 'tx'}
        formatString = '%d %f %f %f %f %f %f %f';
        fid = fopen([dataDir 'data.log']);
        C = textscan(fid, formatString);
        fclose(fid);
        
        time = C{1, 2};       
        
        footWrench = cell2mat(C(1, 3:8));
    case {'tx-rx'}
        formatString = '%d %f %f %f %f %f %f %f %f';
        fid = fopen([dataDir 'data.log']);
        C = textscan(fid, formatString);
        fclose(fid);
        
        time = C{1, 3};        
        
        footWrench = cell2mat(C(1, 4:9));
end

% get unique timestamps
[time, ia] = unique(time);
footWrench = footWrench(ia, :);

end
