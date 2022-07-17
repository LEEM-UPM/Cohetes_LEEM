clc
clear

fileID = fopen("DATOS_XBEE2.TXT",'r');
h = 0;
h2 = 0;
paquetes_perdidos = 0;

while (~feof(fileID))
    check = fread(fileID,2,'char');

    if size(check, 1) ~= 2
        break;
    end

    % Apertura
    if (check(1) == 'A' && check(2) == 'A')
        apertura = h;
        t_apertura = fread(fileID,1,'uint32');
        continue;
    end


    % Rapidos
    if (check(1) == 'E' && check(2) == 'E')
        h = h+1;
        Data(h,1) = fread(fileID,1,'uint32');

        for i=2:13
            Data(h,i) = fread(fileID,1,'single');
        end
        Data(h,14) = fread(fileID,1,'int16');
        Data(h,15) = fread(fileID,1,'int16');
        Data(h,16) = fread(fileID,1,'int16');

        continue;
    end


    % Lentos
    if (check(1) == 'F' && check(2) == 'F')
        h2 = h2 +1;
        Data2(h2,1) = fread(fileID,1,'uint32');

        for i=2:15
            Data2(h2,i) = fread(fileID,1,'single');
        end

        Data2(h2,16) = fread(fileID,1,'uint8');
        Data2(h2,17) = fread(fileID,1,'uint8');
        Data2(h2,18) = fread(fileID,1,'uint8');
        Data2(h2,19) = fread(fileID,1,'uint8');

        continue;
    end



    paquetes_perdidos = paquetes_perdidos + 1;
    disp("Paquetes perdido");


end
fclose(fileID);


