clear all
close all
clc

arduino = serial('COM6', 'BaudRate', 9600);   
set(arduino,'DataBits', 8);
set(arduino,'StopBits', 1);
fopen(arduino);
s.ReadAsyncMode = 'continuous';
t=0;
readasync(arduino);
ss=0;

t=0;
i=1;
while (1)
    
    bb=fscanf(arduino,'%f');
       
    t=0;
    ax(i)=[i];
    Gx(i)=bb(1,1);
    Gy(i)=bb(2,1);
    Gz(i)=bb(3,1);
    Ax(i)=bb(4,1);
    Ay(i)=bb(5,1);
    Az(i)=bb(6,1);
    Mx(i)=bb(7,1);
    My(i)=bb(8,1);
    Mz(i)=bb(9,1);
    i=i+1;
 
    ax1=subplot(3,3,1);
    ax2=subplot(3,3,2);
    ax3=subplot(3,3,3);
    ax4=subplot(3,3,4);
    ax5=subplot(3,3,5);
    ax6=subplot(3,3,6);
    ax7=subplot(3,3,7);
    ax8=subplot(3,3,8);
    ax9=subplot(3,3,9);
    
    plot(ax1,ax',Gx');
    title(ax1,'Time');
    ylabel(ax1,'GX1');
    
    plot(ax2,ax',Gy');
    title(ax2,'Time');
    ylabel(ax2,'GX2');
    
    plot(ax3,ax',Gz');
    title(ax3,'Time');
    ylabel(ax3,'GX3');
    
    plot(ax4,ax',Ax');
    title(ax4,'Time');
    ylabel(ax4,'AX1');
    
    plot(ax5,ax',Ay');
    title(ax5,'Time');
    ylabel(ax5,'AX2');
    
    plot(ax6,ax',Az');
    title(ax6,'Time');
    ylabel(ax6,'AX3');
    
    plot(ax7,ax',Mx');
    title(ax7,'Time');
    ylabel(ax7,'MX1');
    
    plot(ax8,ax',My');
    title(ax8,'Time');
    ylabel(ax8,'MX2');
    
    plot(ax9,ax',Mz');
    title(ax9,'Time');
    ylabel(ax9,'MX3');
    
   
   
   
   
    drawnow     
    ss=ss+1;
    
end

fclose(arduino);
delete(arduino);
clear(arduino);
    
    
    
    
    
    
    
    
    
    
    


    
        
    




    
    


    
    
    








