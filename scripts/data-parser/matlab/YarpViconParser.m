classdef YarpViconParser
    properties (Access = private)
        dataDir,
        timeFormat
        fileID
        viconRoot
        dataMap
        parsed
        subjects        
    end
    
    methods
        function obj = YarpViconParser(dataDir, timeFormat, viconRoot)
            obj.dataDir = ' ';                     
            obj.timeFormat = 'tx';
            obj.viconRoot = 'Vicon_ROOT';
            obj.dataMap = containers.Map('KeyType','char','ValueType','any');
            obj.subjects = containers.Map('KeyType','char','ValueType','any');
            obj.parsed = false;
            
            timeFormatCond = strcmp(timeFormat, 'tx') || ...
                             strcmp(timeFormat, 'rx') || ...
                             strcmp(timeFormat, 'tx-rx');
            assert( timeFormatCond , 'Invalid time format');
            
            fid = fopen([dataDir '/data.log']);
            assert(fid ~= -1, 'Invalid file');
            fclose(fid);
            
            obj.dataDir = dataDir;
            obj.timeFormat = timeFormat;
            obj.viconRoot = viconRoot;
        end
        
        function obj = parseViconLog(obj)            
            fid = fopen([obj.dataDir '/data.log']);
            nLines = 0;
                        
            time = [];
            while (1)
                line = fgetl(fid);
                if line == -1
                    break
                end
                                                
                % separate timestamps and data string
                switch obj.timeFormat
                    case {'rx', 'tx'}
                        expression='(\d+) (\d+\.\d+) \((.*)\)';
                        tokens = regexp(line, expression, 'tokens');
                        time = [time str2double(tokens{1}(2))];                        
                        viconData = tokens{1}(3);
                        viconData = viconData{1};
                    case 'tx-rx'
                        expression='(\d+) (\d+\.\d+) (\d+\.\d+) \((.*)\)';
                        tokens = regexp(line, expression, 'tokens');
                        time = [time str2double(tokens{1}(3))];                        
                        viconData = tokens{1}(4);
                        viconData = viconData{1};
                end
                tfCollection = YarpViconParser.parseTFCollectionForViconTF(viconData, obj.viconRoot);
                obj = obj.updateViconDataMap(tfCollection, time(end));
                nLines = nLines + 1;               
            end        
            obj.populateMetaData();
            fclose(fid);
        end        
        
        function printMetaDataMap(obj)
            for key = obj.subjects.keys
                sub = obj.subjects(key{1});
                disp("===================")
                disp(['Subject Name: ', key{1}]);
                seg_str={};
                for seg = sub.segments
                    seg_str(end+1) = seg;
                end                
                disp(['Available Segments: ', strjoin(seg_str, ', ')]);
                marker_str={};
                for marker = sub.markers
                    marker_str(end+1) = marker;
                end                
                disp(['Available Markers: ', strjoin(marker_str, ', ')]);
                disp("===================")
            end
        end
        
        function segmentData = getSegmentTrajectory(obj, subject_name, segment_name)
            assert(isKey(obj.subjects, subject_name), 'Specified subject unavailable');
            subj = obj.subjects(subject_name);
            assert(any(strcmp(subj.segments, segment_name)), 'Specified segment unavailable');
            key = {'Subj_', subject_name,'::Seg_', segment_name};
            query=strjoin(key, '');
            assert(isKey(obj.dataMap, query), 'Malformed subject segment query combination');
            segmentData = obj.dataMap(query);
        end
        
        function markerData = getMarkerTrajectory(obj, subject_name, marker_name)
            assert(isKey(obj.subjects, subject_name), 'Specified subject unavailable');
            subj = obj.subjects(subject_name);
            assert(any(strcmp(subj.markers, marker_name)), 'Specified marker unavailable');
            key = {'Subj_', subject_name,'::Marker_', marker_name};
            query=strjoin(key, '');
            assert(isKey(obj.dataMap, query), 'Malformed subject marker query combination');
            markerData = obj.dataMap(query);
        end
        
        function markerNames = getAllMarkerNames(obj, subject_name)
            assert(isKey(obj.subjects, subject_name), 'Specified subject unavailable');
            subj = obj.subjects(subject_name);
            markerNames = subj.markers;
        end
        
    end
    
    methods (Access = private)
        function obj = updateViconDataMap(obj, tfCollection, rxTime)
            for tfIdx = 1:length(tfCollection)
                tf = tfCollection(tfIdx);
                xyz = [str2double(tf.x) str2double(tf.y) str2double(tf.z)];
                quat = [str2double(tf.qw) str2double(tf.qx) str2double(tf.qy) str2double(tf.qz)];
                
                % avoid this corner case scenario, means the marker was
                % occluded in the vicon volume and was not visible
                if (xyz(1) == 0.0 && xyz(2) == 0.0 && xyz(3) == 0.0)
                    continue
                end
                
                updated = struct('bodyID', '', ...
                                  'rxTime', [], ...
                                  'xyz', [], ...
                                  'quat', []);
                if (isKey(obj.dataMap, tf.child))
                    existing = obj.dataMap(tf.child);
                    updated.bodyID = existing.bodyID;
                    updated.rxTime = [existing.rxTime; rxTime];                                    
                    updated.xyz = [existing.xyz; xyz];
                    updated.quat = [existing.quat; quat];
                else
                    updated.bodyID = tf.child;
                    updated.rxTime = [rxTime];
                    updated.xyz = [xyz];
                    updated.quat = [quat];
                end                
                obj.dataMap(tf.child) = updated;                
            end
        end
        
        function obj = populateMetaData(obj)
            keys = obj.dataMap.keys;
            for key = keys
                keystr = key{1};
                % ignore unlabeled markers
                if strcmp(keystr(1:10),'UnlMarker#') == 1
                    continue
                end
                expr='Subj_(?<subject>.*\ ?.*)::(?<seg_or_marker>Marker|Seg)_(?<Name>.*)';
                meta = regexp(keystr, expr, 'names');                
                subj = struct('subject_name', '', ...
                              'markers', cell(1), ...
                              'segments', cell(1));

                if (isKey(obj.subjects, meta.subject))   
                    subj = obj.subjects(meta.subject);
                    if (strcmp(meta.seg_or_marker, 'Marker'))
                        subj.markers(end+1) = {meta.Name};
                    elseif (strcmp(meta.seg_or_marker, 'Seg'))
                        subj.segments(end+1) = {meta.Name};
                    end                    
                else                    
                    if (strcmp(meta.seg_or_marker, 'Marker'))
                        subj.markers(1) = {meta.Name};
                    elseif (strcmp(meta.seg_or_marker, 'Seg'))
                        subj.segments(1) = {meta.Name};
                    end
                    subj.subject_name = meta.subject;
                end
                
                obj.subjects(meta.subject) = subj;
            end
            
        end
    end
    
    methods (Static)
        function tfCollection = parseTFCollectionForViconTF(line, viconRoot)            
            
            splitExpr = ') (';
            tokens =  regexp(line, splitExpr, 'split');
            N = length(tokens);
            tfCollection = repmat(struct('parent', '', ...
                                         'child', '', ...
                                         'txTimeStamp','', ...
                                         'qw', '', ...
                                         'qx', '', ...
                                         'qy', '', ...
                                         'qz', '', ...
                                         'x', '', ...
                                         'y', '', ...
                                         'z', ''), N, 1 );
            
            C ={'(?<parent>', viconRoot, ')\ \"(?<child>.*)\"\ (?<txTimeStamp>\d+\.\d+)\ (?<x>[+-]?\d+\.?\d*([eE][+-]?\d+)?)\ (?<y>[+-]?\d+\.?\d*([eE][+-]?\d+)?)\ (?<z>[+-]?\d+\.?\d*([eE][+-]?\d+)?)\ (?<qw>[+-]?\d+\.?\d*([eE][+-]?\d+)?)\ (?<qx>[+-]?\d+\.?\d*([eE][+-]?\d+)?)\ (?<qy>[+-]?\d+\.?\d*([eE][+-]?\d+)?)\ (?<qz>[+-]?\d+\.?\d*([eE][+-]?\d+)?)'};
            expression = strjoin(C, '');
            for idx = 1:N                  
                tfStruct = regexp(tokens{idx}, expression, 'names');
                assert(~any(structfun(@isempty, tfStruct)), 'Could not parse the Vicon log, maybe the ViconRoot variable is wrong');
                tfCollection(idx) = tfStruct;
            end            
        end
    end
end
