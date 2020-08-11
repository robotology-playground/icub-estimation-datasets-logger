clc
clear
close all
%%

timeFormat = 'tx-rx';  %tx-rx|rx|tx
viconDataDir='dumper/transformServer/transforms_o/';
viconRootString='Vicon_ROOT';
x = YarpViconParser(viconDataDir, timeFormat, viconRootString);
x = x.parseViconLog();
x.printMetaDataMap();

subject_name = 'base-mount';
segment_name =  'base_link';
baseLinkSegmentDataInViconWorld  = x.getSegmentTrajectory(subject_name, segment_name);

marker_name = 'marker_2';
baseLinkMarkerDataInViconWorld = x.getMarkerTrajectory(subject_name, marker_name);



