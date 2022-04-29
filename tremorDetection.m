clc
clear all
load PD008-OFF_long_walk_new.mat

[sst,f,t] = fsst(tree.jointData(14).jointAngle(1:8500,1),60,'yaxis');

sst1=sst; %%%creating a sample555
sst(1:13,1) = 0;

xrc = ifsst(sst);
plot((1:8500)/3600,[tree.jointData(14).jointAngle(1:8500,1) xrc xrc-tree.jointData(14).jointAngle(1:8500,1)]);

