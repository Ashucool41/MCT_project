%used to estimate moment of inertia of quadrotor
motorMass=0.1
L=0.45/2 %half of quad diag length
batteryMass=.2
escMass=.03
frameTotalMass=1.5-escMass*4-motorMass*4-batteryMass
%assuming frame as a X cross rods
frameRodMass=frameTotalMass/2
Izz=1/12*frameRodMass*(2*L)^2+4*(motorMass*L^2)+4*(escMass*(L/2)^2)
Ixx=Izz/2%assuming planar quad :))