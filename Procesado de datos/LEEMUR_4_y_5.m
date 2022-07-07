clc
clear

fileID = fopen("DATOS.TXT",'r');
h = 0;
paquetes_perdidos = 0;

while (~feof(fileID))
    h = h+1;
    check = fread(fileID,2,'char');

       if (check(1) == 'A' && check(2) == 'A')
         apertura = h;
         t_apertura = fread(fileID,1,'uint32');
         continue;
       end

       if (check(1) ~= 'E' || check(2) ~= 'E')
           paquetes_perdidos = paquetes_perdidos + 1;
           disp("Paquetes perdido");
           continue;
       end
            
       Data(h,1) = fread(fileID,1,'uint32');

        for i=2:13
            Data(h,i) = fread(fileID,1,'single');
        end 

       Data(h,14) = fread(fileID,1,'uint8');
       Data(h,15) = fread(fileID,1,'uint8');
       Data(h,16) = fread(fileID,1,'uint8');

end
fclose(fileID);
