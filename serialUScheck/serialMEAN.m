clc
clear all
close all
port = serialportlist;
s1 = serialport(port(1), 115200,"Timeout",10);
s1.Timeout = 2;
flush(s1);
val='';
clear A
A=[];
for c = 1:100



val=str2num(convertStringsToChars(readline(s1)));

A=[A,val-6405];


%figure

end
plot(A)
hold on
line([0 100], [mean(A) mean(A)])
hold off
disp(mean(A)/80)
clear s1