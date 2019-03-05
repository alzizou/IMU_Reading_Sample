clear 
clc

S = serial('COM3','BaudRate',115200);
fopen(S);
pause('on');

Tf = 1000;

for i=1:Tf
    for j=1:26
        Data{j,i} = fscanf(S);
        b = Data{j,i};
        if j<=3
            Val(j,i) = str2double(b(9:end));               
        elseif j<=6
            if strcmp(b(10),'-') 
                Val(j,i) = str2double(b(10:14));   
            else
                Val(j,i) = str2double(b(10:13));   
            end
        elseif j<=9
            if strcmp(b(14),'-')
                Val(j,i) = str2double(b(14:18));
            else
                Val(j,i) = str2double(b(14:17));
            end
        elseif j==10
            Val(j,i) = str2double(b(15:19));               
        elseif j<=13
            Val(j,i) = str2double(b(9:end));               
        elseif j<=16
            if strcmp(b(10),'-')
                Val(j,i) = str2double(b(10:14));               
            else
                Val(j,i) = str2double(b(10:13));
            end
        elseif j<=19
            if strcmp(b(14),'-')
                Val(j,i) = str2double(b(14:18));
            else
                Val(j,i) = str2double(b(14:17));
            end
        elseif j<=22
            Val(j,i) = str2double(b(9:end));               
        elseif j<=24
            Val(j,i) = str2double(b(14:end));               
        else
            Val(j,i) = str2double(b(18:end));
        end                
    end
    %pause(0.1);
    i
end

fclose(S);

save('Data','Data');
save('Val','Val');

figure
subplot(3,3,1);
plot(Val(1,:));
title('acc_x');
subplot(3,3,4);
plot(Val(2,:));
title('acc_y');
subplot(3,3,7);
plot(Val(3,:));
title('acc_z');
subplot(3,3,2);
plot(Val(4,:));
title('acc_xg (g)');
subplot(3,3,5);
plot(Val(5,:));
title('acc_yg (g)');
subplot(3,3,8);
plot(Val(6,:));
title('acc_zg (g)');
subplot(3,3,3);
plot(Val(7,:));
title('acc_{xg}_{AVG} (g)');
subplot(3,3,6);
plot(Val(8,:));
title('acc_{yg}_{AVG} (g)');
subplot(3,3,9);
plot(Val(9,:));
title('acc_{zg}_{AVG} (g)');


figure 
subplot(3,3,1);
plot(Val(11,:));
title('gyr_x');
subplot(3,3,4);
plot(Val(12,:));
title('gyr_y');
subplot(3,3,7);
plot(Val(13,:));
title('gyr_z');
subplot(3,3,2);
plot(Val(14,:));
title('gyr_{phi} (rad/sec)');
subplot(3,3,5);
plot(Val(15,:));
title('gyr_{theta} (rad/sec)');
subplot(3,3,8);
plot(Val(16,:));
title('gyr_{psi} (rad/sec)');
subplot(3,3,3);
plot(Val(17,:));
title('gyr_{phi}_{AVG} (rad/sec)');
subplot(3,3,6);
plot(Val(18,:));
title('gyr_{theta}_{AVG} (rad/sec)');
subplot(3,3,9);
plot(Val(19,:));
title('gyr_{psi}_{AVG} (rad/sec)');

figure
subplot(3,3,1)
plot(Val(20,:));
title('Magn_X');
subplot(3,3,4)
plot(Val(21,:));
title('Magn_Y');
subplot(3,3,7)
plot(Val(22,:));
title('Magn_Z');
subplot(3,3,2)
plot(Val(23,:));
title('Magn_{YX}');
subplot(3,3,5)
plot(Val(24,:));
title('Magn_{ZX}');
subplot(3,3,3)
plot(Val(25,:));
title('Magn_{YX}_{AVG}');
subplot(3,3,6)
plot(Val(26,:));
title('Magn_{ZX}_{AVG}');

figure
plot(Val(10,:))
title('Temperature (C)')
