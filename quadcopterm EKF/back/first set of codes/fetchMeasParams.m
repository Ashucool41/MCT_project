function(C,Y) = fetchMeasParams(T0,Ts,k,Cib,vx,vy,vz)
   t=Ts*k+T0;
   %kalman runs at 10ms
   index1=sortedData(:,2)<=t;  %take out past 10 ms measurements
   index2=sortedData(:,2)>t-Ts;
   index=bitand(index1,index2);
   
   data = sortedData(index,:);%past 10ms meas
   C=[];
   Y=[];
   for i=1:size(data,1)
       if(data(i,1)==1)
           [Ctemp,Ytemp] = getIMU(data(i,:),Cib,[vx;vy;vz]);
           C = [C;Ctemp];
           Y = [Y;Ytemp];
       end
       if(data(i,1)==2)
           [Ctemp,Ytemp] = getGPS(data(i,:));
           C = [C;Ctemp];
           Y = [Y;Ytemp];
       end
       if(data(i,1)==3)
           [Ctemp,Ytemp] = getBaro(data(i,:));
           C = [C;Ctemp];
           Y = [Y;Ytemp];
       end
       if(data(i,1)==5)
           [Ctemp,Ytemp] = getSonar(data(i,:));
           C = [C;Ctemp];
           Y = [Y;Ytemp];
       end
       C=[];
       Y=[];
   end
   
end

