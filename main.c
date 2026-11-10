import numpy as np
import matplotlib.pyplot as plt

from re import U
W_desired= 50 #(rad/s)
W_start=0

#define as constantes do motor
min_V= 0 #tensão minima
max_V= 5 #tensão maxima

R= 54.55   # Resistência de armadura (Ohms)
Kt= 1.005e-3  # Constante de torque (Nm/A)
Ke= 0.098/20  # Constante de força contra-eletromotriz (V/rad/s)

B= 1.600e-6   # Coeficiente de atrito viscoso (Nm/rad/s)
J= 243.10e-9   #Momento de inércia do rotor (kg.m²)

simulation_total_time=60
num_steps = simulation_total_time*1000


def next_omega(u, W, dt):
    #Calculo proxima velocidade
    b= u* max_V
    K1 = ((R*J)/(Kt*Ke))  #tau
    K2 = (Kt/((R*B)+(Kt*Ke)))  #ganho K
    a= ((K2*b) - W)/K1
    next_W = (W + (a*dt))
    return next_W

def plotOmega(time,num_steps,W_list):
    fig1,plt1 = plt.subplots()
    plt1.plot(time, np.repeat(W_desired, num_steps+1), ls="--")
    plt1.plot(time, W_list)
    plt1.set_xlabel("Tempo"); plt1.set_ylabel("Velocidade (rads/s)");

def plotSaida(time,U_list):
    fig2,plt2 = plt.subplots()
    plt2.plot(time, U_list)
    plt2.set_xlabel("Tempo"); plt2.set_ylabel("Saida");

def simulate_temp(controller, simulation_total_time ,num_steps):
    dt = simulation_total_time/num_steps
    W=W_start
    U_list=[0]
    W_list=[W]

    for k in range(num_steps):
        # Calcula a saida do controlador

        u = controller.get_control(W,dt)
        u = np.clip(u, 0, 1)

        U_list.append(u)

        # Simula o proximo Omega
        W = next_omega(u, W, dt)
        W_list.append(W)

    time = dt*np.arange(num_steps+1)
    plotOmega(time,num_steps,W_list)
    plotSaida(time,U_list)
    return W_list
//Calculo de Perfomece:
from logging import Handler
def performace(W_list):
  # Calculo Tempo de pico, sobressinal e Razão de pico
  W_anterior = 0
  for i, W in enumerate(W_list):
    if (W>W_anterior):
      Temp1 = i*0.001
      W_anterior = W
      sobressinal = W_anterior - W_desired
      razao_sobressinal = (sobressinal / W_desired)*100

    #Calculo offset e tempo de acomodação
  for i,W in enumerate (W_list):
    if abs(W-W_list[i-1])<0.001:
      Temp_acomod = i*0.001
      W1=W
      erro = W_desired - W1
      print(f"offset:{erro}")
      print(f"Tempo de acomodação: {Temp_acomod}s")
      break

    #Calculo tempo de subida
  for i,W in enumerate (W_list):
    if (W-W_desired)<0.005:
      Temp_subida = i*0.001
      print(f"tempo de subida: {Temp_subida}s")
      break
  print(f"Tempo de pico: {Temp1}s")
  print(f"Sobressinal: {sobressinal}")
  print("Razão de Sobresinal: ",razao_sobressinal,"%")
  pass
//Controle PID:
class Controller:
    def __init__(self, Kp, Ki, Kd, set_point):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.int_term = 0
        self.derivative_term = 0
        self.last_error = None

    def get_control(self, measurement, dt):
        error = self.set_point - measurement
        self.int_term += error*self.Ki*dt
        if self.last_error is not None:
            self.derivative_term = (error-self.last_error)/dt*self.Kd
        self.last_error = error
        return self.Kp * error + self.int_term + self.derivative_term

//Kp=64.94140625; Ki=831.2067079839594; Kd=0.005073283129784907

//tempo de subida: 0.0s
//Tempo de pico: 0.074s
//Sobressinal: 0.9120179958492054
//Razão de Sobresinal:  26.05765702426301 %

controller = Controller(Kp=64.94140625,Ki=831.2067079839594,Kd=0.005073283129784907, set_point=W_desired)
W_list=simulate_temp(controller,simulation_total_time, num_steps)
performace(W_list
