clc
clear all
close all
port = serialportlist;
s1 = serialport(port(1), 115200,"Timeout",10);
s1.Timeout = 2;
flush(s1);
val='';
A = []
for c = 1:1e5



    val=str2num(convertStringsToChars(readline(s1)))
    A=[A,val];
    

end
%figure

%plot(A)
%pause(0.1)

clear s1