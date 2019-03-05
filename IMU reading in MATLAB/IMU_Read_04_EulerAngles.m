clear 
clc

S = serial('COM3','BaudRate',115200);
fopen(S);
pause('on');

Tf = 1500;
NUM_AVG = 1000;
k = 0;

deltat = zeros(Tf,1);

for i=1:Tf
    deltatf = fscanf(S);
    deltat(i,1) = str2double(deltatf);
    NUMf = fscanf(S);
    NUM = str2double(NUMf)
    if NUM >= NUM_AVG
        k = k+1;
        for j=1:37
            Data{j,k} = fscanf(S);
            b = Data{j,k};
            if j<=9
                Val(j,k) = str2double(b(9:end));               
            elseif j<=18
                Val(j,k) = str2double(b(14:end));
            elseif j<=27
                Val(j,k) = str2double(b(12:end));
            elseif j<=30
                Val(j,k) = str2double(b(12:19));               
            elseif j<=34
                Val(j,k) = str2double(b(10:end));                                        
            else
                Val(j,k) = str2double(b(7:end));
            end                
        end
        k
    end
    %pause(0.5);
    %i
end

fclose(S);

save('Data','Data');
save('Val','Val');

figure
subplot(3,3,1);
plot(Val(1,:));
title('acc_x');
grid
subplot(3,3,4);
plot(Val(2,:));
title('acc_y');
grid
subplot(3,3,7);
plot(Val(3,:));
title('acc_z');
grid
subplot(3,3,2);
plot(Val(10,:));
title('acc_{xFilt}');
grid
subplot(3,3,5);
plot(Val(11,:));
title('acc_{yFilt}');
grid
subplot(3,3,8);
plot(Val(12,:));
title('acc_{zFilt}');
grid
subplot(3,3,3);
plot(Val(19,:));
title('acc_{xOK}');
grid
subplot(3,3,6);
plot(Val(20,:));
title('acc_{yOK}');
grid
subplot(3,3,9);
plot(Val(21,:));
title('acc_{zOK}');
grid


figure
subplot(3,3,1);
plot(Val(4,:));
title('gyr_x');
grid
subplot(3,3,4);
plot(Val(5,:));
title('gyr_y');
grid
subplot(3,3,7);
plot(Val(6,:));
title('gyr_z');
grid
subplot(3,3,2);
plot(Val(13,:));
title('gyr_{xFilt}');
grid
subplot(3,3,5);
plot(Val(14,:));
title('gyr_{yFilt}');
grid
subplot(3,3,8);
plot(Val(15,:));
title('gyr_{zFilt}');
grid
subplot(3,3,3);
plot(Val(22,:));
title('gyr_{xOK}');
grid
subplot(3,3,6);
plot(Val(23,:));
title('gyr_{yOK}');
grid
subplot(3,3,9);
plot(Val(24,:));
title('gyr_{zOK}');
grid


figure
subplot(3,3,1);
plot(Val(7,:));
title('mag_x');
grid
subplot(3,3,4);
plot(Val(8,:));
title('mag_y');
grid
subplot(3,3,7);
plot(Val(9,:));
title('mag_z');
grid
subplot(3,3,2);
plot(Val(16,:));
title('mag_{xFilt}');
grid
subplot(3,3,5);
plot(Val(17,:));
title('mag_{yFilt}');
grid
subplot(3,3,8);
plot(Val(18,:));
title('mag_{zFilt}');
grid
subplot(3,3,3);
plot(Val(25,:));
title('mag_{xOK}');
grid
subplot(3,3,6);
plot(Val(26,:));
title('mag_{yOK}');
grid
subplot(3,3,9);
plot(Val(27,:));
title('mag_{zOK}');
grid


figure
subplot(3,1,1)
plot(Val(28,:))
title('Omega_x')
grid
subplot(3,1,2)
plot(Val(29,:))
title('Omega_y')
grid
subplot(3,1,3)
plot(Val(30,:))
title('Omega_z')
grid


figure
subplot(4,2,1)
plot(Val(31,:))
title('Quat_1')
grid
subplot(4,2,3)
plot(Val(32,:))
title('Quat_2')
grid
subplot(4,2,5)
plot(Val(33,:))
title('Quat_3')
grid
subplot(4,2,7)
plot(Val(34,:))
title('Quat_4')
grid
subplot(4,2,2)
plot(Val(35,:))
title('Phi')
grid
subplot(4,2,4)
plot(Val(36,:))
title('Theta')
grid
subplot(4,2,6)
plot(Val(37,:))
title('Psi')
grid
