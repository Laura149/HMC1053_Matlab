frecuencia=1000; %FRECUENCIA=1000Hz(muestras/segundo)

%%CalcularVelocidadAngular

for i=1: 1 : numero_muestras
L1(i)=sqrt((HB(i)*HB(i))+(HC(i)*HC(i))); %%compruebo longitud del vector para si es proxima a cero descartar la medida
L2(i)=sqrt((HC(i)*HC(i))+(HA(i)*HA(i)));
L3(i)=sqrt((HA(i)*HA(i))+(HB(i)*HB(i)));
    if L1(i)>0.3
     if L2(i)>0.3
         if L3(i)>0.3
             x=HB(i)/HC(i);
             x_ant=HB(i-1)/HC(i-1);
             ax1= atan (x);
             ax2= atan (x_ant);
             wx_radianes(i)=(ax1)-(ax2); %rango entre -2pi y 2pi-> grados cada muestra
             if wx_radianes(i) >= pi
                 wx_radianes(i)=-2*pi+wx_radianes(i); %range -pi..+pi
             elseif wx_radianes(i) >= -pi
                  wx_radianes(i)=2*pi+wx_radianes(i); %range -pi..+pi
             end
             wx(i)=wx_radianes(i)*frecuencia;%rad/S-> FRECUENCIA=1000Hz(muestras/segundo)
             
             
             y=HC(i)/HA(i);
             y_ant=HC(i-1)/HA(i-1);
             ay1= atan (y);
             ay2= atan (y_ant);
             wy_radianes(i)=(ay1)-(ay2); %rango entre -2pi y 2pi-> grados cada muestra
             if wy_radianes(i) >= pi
                 wy_radianes(i)=-2*pi+wy_radianes(i); %range -pi..+pi
             elseif wy_radianes(i) >= -pi
                  wy_radianes(i)=2*pi+wy_radianes(i); %range -pi..+pi
             end
             wy(i)=wy_radianes(i)*frecuencia;%rad/S-> FRECUENCIA=1000Hz(muestras/segundo)

             
             
             z=HA(i)/HB(i);
             z_ant=HA(i-1)/HB(i-1);
             az1= atan (z);
             az2= atan (z_ant);
             wz_radianes(i)=(az1)-(az2); %rango entre -2pi y 2pi-> grados cada muestra
             if wz_radianes(i) >= pi
                 wz_radianes(i)=-2*pi+wz_radianes(i); %range -pi..+pi
             elseif wz_radianes(i) >= -pi
                  wz_radianes(i)=2*pi+wz_radianes(i); %range -pi..+pi
             end
             wz(i)=wz_radianes(i)*frecuencia;%rad/S-> FRECUENCIA=1000Hz(muestras/segundo)
           
         end 
     end
    end
end

figure(1)
      subplot(3,1,1);
      plot( wx, 'kx');
      title('Subplot 1: Velocidad angular en eje x')
      ylabel('Velocidad angular (rad/s)');
      subplot(3,1,2);
      plot( wy, 'bx');
      title('Subplot 2: Velocidad angular en eje y')
      ylabel('Velocidad angular (rad/s)');
      subplot(3,1,3);
      plot(wz, 'gx');
      title('Subplot 3: Velocidad angular en eje z')
      ylabel('Velocidad angular (rad/s)');
      xlabel('Número de muestra');

