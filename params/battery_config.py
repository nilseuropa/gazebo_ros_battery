#!/usr/bin/env python3
# coding=UTF-8
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import sys
import yaml

if (len(sys.argv)==2):
  with open(sys.argv[1]) as file:
    parameter = yaml.load(file, Loader=yaml.FullLoader)

  K = parameter['polarization_constant']
  Q = parameter['design_capacity']
  E0 = parameter['full_charge_voltage']
  A = parameter['exponential_voltage']
  B = parameter['exponential_capacity']
  Vcutoff = parameter['cut_off_voltage']
  Ctime = parameter['characteristic_time']
  Tshift = parameter['reversible_voltage_temp']
  Kalpha = parameter['arrhenius_rate_polarization']
  dQdT = parameter['capacity_temp_coeff']
  T0 = parameter['design_temperature']
  print('Parameters loaded.')

  def V(it,i,T):
    t = T+273.15
    t0 = T0+273.15
    QT = Q + dQdT*(t-t0)
    KT = K * np.exp(Kalpha*(1/t-1/t0))
    E0T = E0 + Tshift*(t-t0)
    return E0T - KT*QT/(QT-it)*(i*Ctime/3600.0+it) + A*np.exp(-B*it) 

  def Vc(it):
    return Vcutoff + 0*it
    
  figure(num=None, figsize=(16, 8), dpi=80, facecolor='w', edgecolor='k')

  plt.subplot(1, 2, 1)
  xx = np.arange(start = 0, stop = Q, step = 0.05)
  plt.plot(xx, V(xx,0.1*Q,T0  ), 'g-')
  xx = np.arange(start = 0, stop = Q+dQdT*(T0/2-T0), step = 0.05)
  plt.plot(xx, V(xx,0.1*Q,T0/2), 'c-')
  xx = np.arange(start = 0, stop = Q+dQdT*(0-T0), step = 0.05)
  plt.plot(xx, V(xx,0.1*Q,0   ), 'b-')
  xx = np.arange(start = 0, stop = Q+dQdT*(-10-T0), step = 0.05)
  plt.plot(xx, V(xx,0.1*Q,-10 ), 'm-')
  xx = np.arange(start = 0, stop = Q+dQdT*(-20-T0), step = 0.05)
  plt.plot(xx, V(xx,0.1*Q,-20 ), 'r-')
  xx = np.arange(start = 0, stop = Q, step = 0.05)
  plt.plot(xx, Vc(xx),   'k-')
  plt.axis([0, Q, 0, E0+A+1])
  plt.xlabel('SOC [Ah]')
  plt.ylabel('Voltage [V]')
  plt.legend([ 'T = T_0 °C', 'T = T_0/2 °C', 'T = 0 °C', 'T = -10 °C', 'T = -20 °C'])
  plt.title('Discharge characteristics versus Temperature (at 0.1C)');
  
  xx = np.arange(start = 0, stop = Q, step = 0.05)
  plt.subplot(1, 2, 2)
  plt.plot(xx, V(xx,0.1*Q,T0), 'g-')
  plt.plot(xx, V(xx,0.5*Q,T0), 'c-')
  plt.plot(xx, V(xx,1*Q,T0  ), 'b-')
  plt.plot(xx, V(xx,5*Q,T0  ), 'm-')
  plt.plot(xx, V(xx,10*Q,T0 ), 'r-')
  plt.plot(xx, Vc(xx),   'k-')
  plt.axis([0, Q, 0, E0+A+1])
  plt.xlabel('SOC [Ah]')
  plt.ylabel('Voltage [V]')
  plt.legend([ '0.1C', '0.5C', '1C', '5C', '10C'])
  plt.title('Discharge characteristics versus discharge rate (at T=T0)');
  
  plt.tight_layout()
  plt.show()

else:
    print('Please provide the name of the parameter file.')
