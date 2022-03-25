function olfactoryArena_LED_control(s1, token, param, docheckstatus)

    dispstatus = nargout == 0;

    if nargin < 4,
        docheckstatus = true;
    end

    switch upper(strtrim(token))
        case 'CONNECT'
            s1 = serial(COMPort, 'BaudRate', 115200, 'Terminator', 'CR');

            try
                fopen(s1);
            catch
                fclose(s1);
                delete(s1);
                fopen(s1);
            end

        case 'IR'
            Ir_int_val = round(param);

            %send command to controller
            fprintf(s1, ['IR ', num2str(Ir_int_val)]);
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'RED'
            Chr_int_val = round(param); % this is done so only one dec place

            %send command to controller
            fprintf(s1, ['RED ', num2str(Chr_int_val)]);
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'RED_QUAD'
            Chr_int_val = round(param.intensity); % this is done so only one dec place

            %send command to controller
            fprintf(s1, ['RED ', num2str(Chr_int_val), ' ', num2str(param.panel), ' ', param.quadrants]);
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'BLUE'
            Chr_int_val = round(param); % this is done so only one dec place

            %send command to controller
            fprintf(s1, ['BLUE ', num2str(Chr_int_val)]);
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end
            
        case 'BLUE_QUAD'
            Chr_int_val = round(param.intensity); % this is done so only one dec place

            %send command to controller
            fprintf(s1, ['BLUE ', num2str(Chr_int_val), ' ', num2str(param.panel), ' ', param.quadrants]);
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'GREEN'
            Chr_int_val = round(param); % this is done so only one dec place

            %send command to controller
            fprintf(s1, ['GREEN ', num2str(Chr_int_val)]);
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end
        
        case 'GREEN_QUAD'
            Chr_int_val = round(param.intensity); % this is done so only one dec place

            %send command to controller
            fprintf(s1, ['GREEN ', num2str(Chr_int_val), ' ', num2str(param.panel), ' ', param.quadrants]);
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end
        
        case 'LOG'
            fprintf(s1, 'LOG');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'LIN'
            fprintf(s1, 'LIN');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'PULSE'
            fprintf(s1, ['PULSE ', num2str(param.pulse_width), ', ', num2str(param.pulse_period), ', ', num2str(param.number), ', ', num2str(param.off), ', ', num2str(param.wait), ', ', num2str(param.iterations), ', ', param.color]);
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'PATT'
            fprintf(s1, ['PATT, ' param]);
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'RUN'
            fprintf(s1, 'RUN');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'STOP'
            fprintf(s1, 'STOP');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'ON'
            fprintf(s1, 'ON');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'OFF'
            fprintf(s1, 'OFF');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'Y0_OFF'
            fprintf(s1, 'OFF 1, 0001');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'Y0_ON'
            fprintf(s1, 'ON 1, 0001');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'Y1_OFF'
            fprintf(s1, 'OFF 1, 0010');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'Y1_ON'
            fprintf(s1, 'ON 1, 0010');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'Y2_OFF'
            fprintf(s1, 'OFF 1, 0100');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'Y2_ON'
            fprintf(s1, 'ON 1, 0100');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'Y3_OFF'
            fprintf(s1, 'OFF 1, 1000');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'Y3_ON'
            fprintf(s1, 'ON 1, 1000');
            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        case 'DISCONNECT'
            fclose(s1);
            delete(s1);

        case 'RESET'
            fprintf(s1, 'CHR 0');
            fprintf(s1, 'OFF 0,0');
            fprintf(s1, 'STOP');

            if docheckstatus, [~, status] = checkControllerStatus(s1, dispstatus); end

        otherwise
            disp('Unknown command for the LED control.')
    end

    function [checReslt, status] = checkControllerStatus(s1, dispstatus)
        %pause(0.1);
        starttime = tic;
        maxwaittime = .5;
        waittime = 0;

        while s1.BytesAvailable <= 1,
            waittime = toc(starttime);

            if waittime >= maxwaittime,
                break;
            end

        end

        fprintf('Waited %f seconds for controller status\n', waittime);
        status = {};

        while s1.BytesAvailable > 1
            s = strtrim(fscanf(s1));

            if isempty(s),
                continue;
            end

            if dispstatus,
                fprintf([s, '\n']);
            else
                status{end + 1} = s; %#ok<AGROW>
            end

        end

        checReslt = 1;
    end

end
