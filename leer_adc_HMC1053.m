clear all;
close all;
clc;
numero_muestras=10000; %Número de muestras que se desean leer
muestra=0;

i=1;

GananciaAmpli=100;  %Ganancia del circuito de amplificación
VCC=3.3;            %tensión de alimentación
Sensibilidad=0.001; %Sensibilidad del sensor

HA=zeros(numero_muestras,1);            %Variable para almacenar los datos del campo en el eje A
HB=zeros(numero_muestras,1);            %Variable para almacenar los datos del campo en el eje B
HC=zeros(numero_muestras,1);            %Variable para almacenar los datos del campo en el eje C
milisegundos=zeros(numero_muestras,1);  %Variable para almacenar los datos del tiempo de la toma de datos
magnitud=zeros(numero_muestras,1);


%Borrar previos
delete(instrfind({'Port'},{'COM6'})); 
%Crear objeto serie
mipuerto = serial('COM6','BaudRate',2000000,'Terminator','CR/LF');
warning('off','MATLAB:serial:fscanf:unsuccessfulRead');
%Abrir puerto
fopen(mipuerto); 

tic %Inicio cuenta de tiempo en Matlab
while (muestra<numero_muestras)
   
    a = fscanf(mipuerto,'%d, %d, %d, %d')'; %Leo el puerto serie
    sensorValueA(i)=a(1)*3.3/4096; %lectura del ADC a voltaje
    sensorValueB(i)=a(2)*3.3/4096;
    sensorValueC(i)=a(3)*3.3/4096;
    milisegundos(i)=a(4); % milisegundo en el que se ha tomado la muestra
    
   HA(i)=((sensorValueA(i)-(VCC/2))/(0.33));%Resultado del campo magnético leido en gauss por el sensor HMC1053
   HB(i)=((sensorValueB(i)-(VCC/2))/(0.33));
   HC(i)=((sensorValueC(i)-(VCC/2))/(0.33));
   magnitud(i)=sqrt((HA(i)*HA(i))+(HB(i)*HB(i))+(HC(i)*HC(i))); %Cálculo del campo magnético total 
    i = i+1
    muestra=muestra+1;
end
t=toc;                %Tiempo de lectura. Fin de cuenta iniciada con tic y finalizada con toc
fclose(mipuerto);     %Cierro puerto serie
delete(mipuerto);     %Elimino el puerto
clear mipuerto;       %Limpio el puerto serie
%%
VA_media=mean(sensorValueA)     %Cálculo de la media del voltaje leido en el eje A
VB_media=mean(sensorValueB)     %Cálculo de la media del voltaje leido en el eje A
VC_media=mean(sensorValueC)     %Cálculo de la media del voltaje leido en el eje A


%%
%Representación del voltaje leido por el ADC del sensor HMC1053
figure(1)
      subplot(3,1,1);
      plot( sensorValueA, 'bx'); %voltaje leido eje A
      title('Subplot 1: VA')
      ylabel('Voltaje eje A (V)');%voltaje leido eje B
      subplot(3,1,2);
      plot( sensorValueB, 'gx');
      title('Subplot 2: VB')
      ylabel('Voltaje eje B (V)');
      subplot(3,1,3);
      plot(sensorValueC, 'rx');%voltaje leido eje C
      title('Subplot 3: VC')
      ylabel('Voltaje eje C (V)');
      xlabel('Número de muestra');
      
%%
%Representación del campo magnético leido por el ADC del sensor HMC1053
figure(2)
      subplot(4,1,1);
      plot( magnitud, 'kx');
      title('Subplot 1: Magnitud') %campo magnético total
      ylabel('Campo magnético (G)');
      subplot(4,1,2);
      plot( HA, 'bx');
      title('Subplot 2: HA') %campo magnético eje A
      ylabel('Campo magnético (G)');
      subplot(4,1,3);
      plot(HB, 'gx');
      title('Subplot 3: HB') %campo magnético eje B
      ylabel('Campo magnético (G)');
      subplot(4,1,4);
      plot(HC, 'rx');
      title('Subplot 4: HC') %campo magnético eje C
      ylabel('Campo magnético (G)');
      xlabel('Número de muestra');


%%
%valores estadísticos de las medidas media varianza y desviación estandar
magnitud_media=mean(magnitud)
magnitud_varianza=var(magnitud)
magnitud_deviacionestandar=std(magnitud)

HA_media=mean(HA)
HA_varianza=var(HA)
HA_deviacionestandar=std(HA)

HB_media=mean(HB)
HB_varianza=var(HB)
HB_deviacionestandar=std(HB)

HC_media=mean(HC)
HC_varianza=var(HC)
HC_deviacionestandar=std(HC)

%%
%Representación del campo magnético leido por el ADC del sensor HMC1053
  figure(3)
   plot(HA, 'bx'); %campo magnético eje A
   hold on 
   plot(HB, 'gx');%campo magnético eje B
   plot(HC, 'rx');%campo magnético eje C
   legend('HA', 'HB', 'HC')
   ylabel('Campo magnético (G)');
   xlabel('Milisegundo medido');
 
   
 %%
 %Representación en tres dimensiones del campo magnético leido por el ADC del sensor HMC1053
 figure(4)
 scatter3(HA,HB,HC)
 axis equal
 xlabel('Eje X: Campo magnético HA(G)');
 ylabel('Eje Y: Campo magnético HB(G)');
 zlabel('Eje Z: Campo magnético HC(G)');
 title('Campo magnético en los tres ejes')
 
 
 %% 
 %Representación de la proyección sobre plano A-B del campo magnético leido por el ADC del sensor HMC1053
 figure(5)
 plot(HA,HB,  'bx');
 legend('HA-HB')
 axis equal
 xlabel('Campo magnético eje A (G)');
 ylabel('Campo magnético eje B (G)');
 title('Proyeción del campo magnético en los ejes A y B')
 
 %% 
 %Representación de la proyección sobre plano C-B del campo magnético leido por el ADC del sensor HMC1053
 figure(6)
 plot(HC,HB,  'rx');
 legend( 'HC-HB')
 axis equal
 xlabel('Campo magnético eje C (G)');
 ylabel('Campo magnético eje B (G)');
 title('Proyeción del campo magnético en los ejes C y B')
  
 %% 
 %Representación de la proyección sobre plano A-C del campo magnético leido por el ADC del sensor HMC1053
 figure(7)
 plot(HA,HC,  'gx');
 legend( 'HA-HC')
 axis equal
 xlabel('Campo magnético eje A (G)');
 ylabel('Campo magnético eje C (G)');
 title('Proyeción del campo magnético en los ejes A y C')
 
 
 
 %% 
 %Representación de laS proyecciones sobre los planos del campo magnético leido por el ADC del sensor HMC1053
 figure(8)
 plot(HA,HB,  'bx');
 hold on
 plot(HA,HC,  'gx');
 plot(HC,HB,  'rx');
 legend('HA-HB', 'HA-HC', 'HB-HC')
 axis equal
 xlabel('Campo magnético (G)');
 ylabel('Campo magnético (G)');
 title('Proyeciones en los planos')
%% calibracion 
%Calibración de los datos campo magnético leido por el ADC del sensor HMC1053
figure(9)
 Matriz_campo=[HA HB HC];
scatter3(Matriz_campo(:,1),Matriz_campo(:,2),Matriz_campo(:,3));%puntos sin relleno
 %scatter3(Matriz_campo(:,1),Matriz_campo(:,2),Matriz_campo(:,3),'filled');puntos con relleno
 hold all
[A, b, expMFS]= magcal(Matriz_campo, 'auto');%devuelve los los coeficientes necesarios para corregir los datos no calibrados del magnetómetro
Matriz_campo_Corregida=(Matriz_campo- b)*A;
scatter3(Matriz_campo_Corregida(:,1),Matriz_campo_Corregida(:,2),Matriz_campo_Corregida(:,3));%puntos sin relleno
%scatter3(Matriz_campo_Corregida(:,1),Matriz_campo_Corregida(:,2),Matriz_campo_Corregida(:,3),'filled');puntos con relleno
legend('Sin calibrar', 'Calibrada')
display(A);%soft iron correction matrix
display(b);%hard iron correction bias
xlabel('Campo magnético eje A (G)');
ylabel('Campo magnético eje B (G)');
zlabel('Campo magnético eje C (G)');
title(' Calibración de Hard Iron y Soft Iron')
axis equal

%%
%Representación de la proyección calibrada sobre los planos del campo magnético leido por el ADC del sensor HMC1053
figure(10)

plot(Matriz_campo_Corregida(:,1),Matriz_campo_Corregida(:,2),'bx')%campo magnético eje A
hold on
plot(Matriz_campo_Corregida(:,1),Matriz_campo_Corregida(:,3),  'gx')%campo magnético eje B
plot(Matriz_campo_Corregida(:,2),Matriz_campo_Corregida(:,3),  'rx')%campo magnético eje C
 legend('HA-HB', 'HA-HC', 'HB-HC')
 axis equal
 xlabel('Campo magnético (G)');
 ylabel('Campo magnético (G)');
 title('Proyeciones calibradas en los planos')