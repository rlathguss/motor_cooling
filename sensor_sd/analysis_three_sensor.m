clear all
close all
clc
%%이건 아두이노 자체에서 필터링한 값을 매트랩으로 출력만

aa=fopen('DATALOG.TXT','r');
data = fscanf(aa,'%f', [4,1000]);
data = data';
time = data(:,1);
sensor1 = data(:,2);
sensor2 = data(:,3);
sensor3 = data(:,4);
plot(time, sensor1, time, sensor2, time, sensor3);
xlabel('Time (T)');ylabel("Temperature (C)");
legend('sensor1','sensor2','sensor3'); grid on
fclose(aa);
