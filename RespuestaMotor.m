% Borra datos que se encuentren previos y vuelve a declarar el puerto y la
%velocidad de transmisión
%close all;
clear;
clc;
delete(instrfind({'port'},{'COM3'})); 
puerto=serial('COM3');
puerto.BaudRate=9600;
puerto.StopBits=1;
puerto.DataBits=8;
fopen(puerto);%abre el puerto a utilizar
aux_modo=0;
comp=0;

while(1)
    datos = fscanf(puerto,'%d,%d,%d,%d,%d')';
    modo=datos(5);
    if (comp~=modo)
        aux_modo=modo;
        comp=modo;
    else
        aux_modo=0;   
    end
    if aux_modo~=0
        if aux_modo==9 % AQUISITION RATE

            % Record and plot 1 seconds of current data

            ii = 0;
            Current = zeros(1e4,1);
            pwm = zeros(1e4,1);
            t = zeros(1e4,1);
            tic

            while toc < 1
                ii = ii + 1;
                % Read current data value
                datos = fscanf(puerto,'%d,%d,%d,%d,%d')';
                disp(datos); % Only for check
                Current(ii) = datos(1);
                pwm(ii)=datos(2);
                % Get time since starting
                t(ii) = toc;
            end

            % Post-process and plot the data. First remove any excess zeros on the
            % logging variables.
            Current = Current(1:ii);
            pwm = pwm(1:ii);
            t = t(1:ii);
            % Compute acquisition rate
            timeBetweenDataPoints = diff(t);
            averageTimePerDataPoint = mean(timeBetweenDataPoints);
            dataRateHz = 1/averageTimePerDataPoint;

            % Plot current versus time
            figure
            plot(t,Current,'-o')
            xlabel('Elapsed time (sec)')
            ylabel('Corriente (mA)')
            title('Ten Seconds of Current Data')
            hold on
            plot(t,pwm)
            hold off

            Variable=num2str(dataRateHz,4);
            Texto_Leyenda1='Frequency of sampling: ';
            Leyenda1=strcat(Texto_Leyenda1,Variable);
            Texto_Leyenda2=' Hz';
            Leyenda2=strcat(Leyenda1,Texto_Leyenda2);
            legend(Leyenda2)
            set(gca,'xlim',[t(1) t(ii)])

        else % WAVES

            % Acquire and display live data
            figure
            h1 = animatedline;
            h2 = animatedline('Color','b');
            h3 = animatedline('Color','r');
            ax = gca;
            ax.YGrid = 'on';
            ax.YLim = [-7000 7000];

            startTime = datetime('now');
            ii = 0;

            while ii<1000
                ii=ii+1;
                % Read current data value
                datos = fscanf(puerto,'%d,%d,%d,%d,%d')';
                disp(datos);
                % Calculate variable from data
                Current = datos(1);
                Speed = datos(3);
                Acceleration = datos(4);
                % Get data time
                t =  datetime('now') - startTime;
                % Add points to animation
                addpoints(h1,datenum(t),Current)
                addpoints(h2,datenum(t),Speed)
                addpoints(h3,datenum(t),Acceleration)
                % Update axes
                ax.XLim = datenum([t-seconds(15) t]);
                datetick('x','keeplimits')
                drawnow
            end


            % Plot the recorded data
            %  Smooth out readings with gaussian filter

            [~,tempLogsC] = getpoints(h1);
            [~,tempLogsS] = getpoints(h2);
            [timeLogs,tempLogsA] = getpoints(h3);
            timeSecs = (timeLogs-timeLogs(1))*24*3600;

            smoothedTemp1 = smoothdata(tempLogsC,'gaussian',50);
            smoothedTemp2 = smoothdata(tempLogsS,'gaussian',50);
            smoothedTemp3 = smoothdata(tempLogsA,'gaussian',50);

            figure

            subplot(3,1,1)
            plot(timeSecs,tempLogsC)
            hold on
            plot(timeSecs,smoothedTemp1,'r')
            xlabel('Elapsed time (sec)')
            ylabel('Current (mA)')
            I=mean(tempLogsC);
            Variable=num2str(I,3);
            Texto_Leyenda1='Mean current: ';
            Leyenda1=strcat(Texto_Leyenda1,Variable);
            Texto_Leyenda2=' mA';
            Leyenda2=strcat(Leyenda1,Texto_Leyenda2);
            legend(Leyenda2)

            subplot(3,1,2)
            plot(timeSecs,tempLogsS)
            hold on
            plot(timeSecs,smoothedTemp2,'r')
            xlabel('Elapsed time (sec)')
            ylabel('Speed (rad/s)')

            subplot(3,1,3)
            plot(timeSecs,tempLogsA)
            hold on
            plot(timeSecs,smoothedTemp3,'r')
            xlabel('Elapsed time (sec)')
            ylabel('Acceleration (rad/s^2)')

            % Save results to a file

            T = table(timeSecs',tempLogsC',tempLogsS',tempLogsA','VariableNames',{'Time_sec','Current_mA','Speed','Acceleration'});
            filename = 'Data.xlsx';
            % Write table to file 
            writetable(T,filename)
            % Print confirmation to command line
            fprintf('Results table with tcurrent measurements saved to file %s\n',...
            length(timeSecs),filename)
        end
    end   

end
