close all;
load ('Experiencias1e2.mat');

high=50;
low=25;

staticValue1=mean(I1mean(10:end));
staticValue2=mean(I2mean(10:end));
staticValue1High=mean(I1meanH(10:end));
staticValue2High=mean(I2meanH(10:end));

staticError1=abs(staticValue1-low)
staticError1High=abs(staticValue1High-high)
staticError2=abs(staticValue2-low)
staticError2High=abs(staticValue2High-high)
