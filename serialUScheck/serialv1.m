clc
clear all
close all
port = serialportlist;
A = []
s1 = serialport(port(2), 115200,"Timeout",10);
s1.Timeout = 2;
flush(s1);

val=str2num(convertStringsToChars(readline(s1)));

while val~= 5000
val=str2num(convertStringsToChars(readline(s1)));
end
for c =1:10
val=str2num(convertStringsToChars(readline(s1)));
while val~= 5000


   A=[A,val];
    val=str2num(convertStringsToChars(readline(s1)));
     
    

end

figure
plot(A)
pause(0.1)
clear A
A = []
end


clear s1