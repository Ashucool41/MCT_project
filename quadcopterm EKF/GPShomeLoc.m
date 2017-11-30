function [home] = GPShomeLoc()
   
   for i=1:size(sortedData,1)
       if(sortedData(i,1)==2)
           home = [sortedData(i,8),sortedData(i,9),sortedData(i,10)];
           break     
       end
   end
   