function formatString = prepareStateExtFormatString(n_joints, sim, timeformat)

if nargin < 2
    sim =  false;
end

if nargin < 3
    timeformat =  'tx';
end
switch timeformat
    case {'rx', 'tx'}
        formatString = '%d %f '; %idx timestamp
    case 'tx-rx'
        formatString = '%d %f %f '; %idx tx-timestamp rx-timestamp
end

for i = 1:11
    formatString = [formatString, '('];
    for j = 1:n_joints
        if j == n_joints
            if i < 10
                formatString = [formatString, '%f' ];
            else
                formatString = [formatString, '%d' ];
            end
        else
            if i < 10
                formatString = [formatString, '%f ' ];
            else
                formatString = [formatString, '%d ' ];
            end
        end
    end
    if (i >=4 && i <=6)
        if (sim)
            formatString = [formatString, ') [fail] '];
        else
            formatString = [formatString, ') [ok] '];
        end
        continue;
    end
    formatString = [formatString, ') [ok] '];
end
end