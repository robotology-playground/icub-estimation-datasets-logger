function [time, ImageFileNameArray] = parseRGBImages(dataDir, timeformat)
if nargin == 1
    timeformat = 'tx';
end

switch timeformat
    case {'rx', 'tx'}
        formatString = '%d %f %s [rgb]';
        fid = fopen([dataDir 'data.log']);
        C = textscan(fid, formatString);
        fclose(fid);
        
        time = C{1, 2};
        
        imageFileNames = string(C{1, 3});
    case 'tx-rx'
        formatString = '%d %f %f %s [rgb]';
        fid = fopen([dataDir 'data.log']);
        C = textscan(fid, formatString);
        fclose(fid);
        
        time = C{1, 3};
        
        imageFileNames = string(C{1, 4});
        
end
ImageFileNameArray = strings(length(time), 1);

for i = 1: length(imageFileNames)
    filename = imageFileNames(i);
    ImageFileNameArray(i) = filename;
end

end
