
% THis is the function that is used to fnd pacf and acf.
% In this assigment is called by q2Caller.m and q4.m to do 2 and 4
% questions
function [acf,phi]=acfPacf(vk)
    %every index should be added with 1
    %clear
    %load series.mat
    
    vMean = mean(vk);
    L = 500;
    N = length(vk);

    sigmaL = zeros(1,L+1); %Autocovariance


     for l = 0:L
         for k = l:N-1
             sigmaL(l+1) = sigmaL(l+1)+(1/N)*( vk(k+1)-vMean )*( vk(k-l+1)-vMean  );
         end
     end
    %sigmaL

    %ACF DONE
    acf=sigmaL/sigmaL(0+1);

   
    %PACF PART phi
    phi = zeros(L+1,L+1);
    
    phi(1+0,1+0)=1;
    phi(1+1,1+1)=acf(1+1);

    sumnum=0;
    densum=0;

    for n=1:L-1
            for j=1:n
                sumnum=sumnum + acf(1+n+1-j)*phi(1+n,1+j);
                densum=densum+phi(1+n,1+j)*acf(1+j);
            end

        phi(1+(n+1),1+(n+1) ) = ( acf(1+(n+1) )- sumnum )/(1-densum);
        sumnum=0;
        densum=0;
        
        for j=1:n
            phi(1+(n+1),1+j) = phi(1+n,1+j)-phi( 1+(n+1),1+(n+1) )*phi( 1+n,1+(n-j+1) );
            
        end
    end

    %Adjusted phiDiag PLOT
    figure
  
    subplot(2,1,1)
    phiDiag = diag(phi);
    plot( 1:L,phiDiag(2:L+1) )
    title('PACF vs lag')
   phiDiag(1+1);
   phi(1+1,1+1);
    acf(1+1);
    %Adjusted ACF PLOT
    subplot(2,1,2)
    plot(0:L,acf);
    title('ACF vs lag')

end
