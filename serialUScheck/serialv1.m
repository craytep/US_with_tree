clc
clear all
close all
port = serialportlist;
A = []
s1 = serialport(port(2), 115200,"Timeout",10);
s1.Timeout = 20000;
flush(s1);

val=str2num(convertStringsToChars(readline(s1)));

while val~= 5000
val=str2num(convertStringsToChars(readline(s1)));
end
A = [];
for c =1:40
while all(A ~= 5000)
A=[A; str2num(convertStringsToChars(read(s1,6e4,"string")))];
end
figure
A(length(A))=[];
plot(A(1:find(A == 5000)-1))

pause(0.1)
A = A(find(A == 5000)+1:end)
end


clear s1