function [R,C,Y,uk] = fetchMeasurements(T0,timeStep_k,Xk)
   global  Ts    Cbi_function
   global sortedData
   t=Ts*(10^6)*timeStep_k+T0;%Ts in secs, T0 in micro secs, t is in micros
   %Cib_numerical=transpose(subs(Cbi,[phi;theta;psi],Xk(10:12)));
   Cib_numerical=transpose(Cbi_function(Xk(10),Xk(11),Xk(12)));
   vx_num=Xk(4);
   vy_num=Xk(5);
   vz_num=Xk(6);
   %kalman runs at 10ms

   index1=sortedData(:,2)<t;  %take out past 10 ms measurements
   index2=sortedData(:,2)>=(t-Ts*10^6);
   index_=bitand(index1,index2);
   %disp(t)
   %disp(index_)
   
   data = sortedData(index_,:);%past 10ms meas
   C=[];
   Y=[];
   Rvec=[];
   uk=[];
   
   for i=1:size(data,1)
       if(data(i,1)==1)
           [Ctemp,Ytemp,Rvec_temp] = getIMU(data(i,:),Cib_numerical,[vx_num;vy_num;vz_num]);
           C = [C;Ctemp];
           Y = [Y;Ytemp];
           Rvec = [Rvec;Rvec_temp];
       end
%        if(data(i,1)==2)
%            [Ctemp,Ytemp,Rvec_temp] = getGPS(data(i,:));
%            C = [C;Ctemp];
%            Y = [Y;Ytemp];
%            Rvec = [Rvec;Rvec_temp];
%        end
       if(data(i,1)==3)
           [Ctemp,Ytemp,Rvec_temp] = getBaro(data(i,:));
           C = [C;Ctemp];
           Y = [Y;Ytemp];
           Rvec = [Rvec;Rvec_temp];
       end
       if(data(i,1)==5 && data(i,3)<6.5)%sonar has a max range of 7m 6.5m with some safety factor :((Otherwise crap the data
           [Ctemp,Ytemp,Rvec_temp] = getSonar(data(i,:));
           C = [C;Ctemp];
           Y = [Y;Ytemp];
           Rvec = [Rvec;Rvec_temp];
           %disp('got sonar data')
       end
       if(data(i,1)==6)
           uk = transpose(data(i,3:6)-1000)/1000;%scaled from 0-1.0 for convenience
           %disp('input came')
           
       end

   end
   R=diag(Rvec); %this will be the measurement covariance matrix.
   %Since multiples sensors are involved and their data rates are not the
   %same. This needs to be assembled every time based on what all data we
   %have.
   
end

