#!/usr/bin/env python

import pylab as pl
from matplotlib import pyplot as pp
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# 2nd-order Butter
class Butter:
    def __init__(self, fc, dt):
        ita = 1./pl.tan(pl.pi*fc*dt)
        q = pl.sqrt(2)
        self.b = [1./(1+q*ita+ita*ita)]
        self.b.append(2*self.b[0])
        self.b.append(self.b[0])
        self.a = [2*(ita*ita - 1)*self.b[0], -(1-q*ita+ita*ita)*self.b[0]]
        self.reset()
        
    def reset(self):
        self.x = [0,0,0]
        self.y = [0,0]        
        
    def feed(self, v):
        self.x[2] = self.x[1]
        self.x[1] = self.x[0]
        self.x[0] = v
        
        v = self.b[2]*self.x[2] + self.b[1]*self.x[1] + self.a[1]*self.y[1] + self.b[0]*self.x[0] + self.a[0]*self.y[0]

        self.y[1] = self.y[0];
        self.y[0] = v;  
        return v

class PID:
    def __init__(self):
        self.reset()
        
    def set_gains(self, Kp, Ki, Kd, dt, umax):
        self.Kp = Kp
        self.Ki = Ki*dt
        self.Kd = Kd/dt
        self.umax = umax        
        
    def reset(self):
        self.u = 0
        self.x = 0
        self.i = 0
        
    def command(self, x, xd):
        e = xd - x
        if abs(self.u) < self.umax:
            self.i += e
        if self.x:
            cmd = self.Kp*(e + self.Ki*self.i + self.Kd*(self.x - x))
        else:
            cmd = self.Kp*(e + self.Ki*self.i)  
        self.x = x
        return min(max(cmd,-self.umax),self.umax)
    

class Sim:
    def __init__(self, mode, dt = 0.01):
        
        self.fig = pp.figure()
        self.mode = mode
        
        self.dt = dt
        
        # setpoint vs output
        self.ax = self.fig.add_subplot(111)        
        self.lxd, = self.ax.plot([], [], 'r--', lw=2, label='setpoint')
        self.lx, = self.ax.plot([], [], 'r', lw=2, label='output')        

        if mode == 'p':
            self.ax.set_ylabel('Setpoint / output [m - rad]')
        else:
            self.ax.set_ylabel('Setpoint / output [m/s - rad/s]')
        
        # command
        self.ax2 = self.ax.twinx()        
        self.lu, = self.ax2.plot([], [], 'g', lw=2, label='command')
        self.l0, = self.ax2.plot([], [], 'k--')
        self.ax2.set_ylabel('Command (N)')
        self.fig.tight_layout()
                        
        self.canvas = FigureCanvas(self.fig)
        self.butter = Butter(5, self.dt)
        
    def run(self, gains, xd, m, k, fmax, tmax):
        
        if xd == 0:
            xd = 0.1
        
        u = [0]
        x = [0]
        v = 0
        
        t = pl.arange(0, tmax+self.dt, self.dt)
        
        self.butter.reset()
        
        pid = PID()
        pid.set_gains(gains['p'], gains['i'], gains['d'], self.dt, fmax)
        
        for ti in t[1:]:
            # control
            u.append(pid.command(self.butter.feed(x[-1]), xd))
            
            # model
            v += self.dt*(u[-1] - k*v)/m
            if self.mode == 'p':
                x.append(x[-1] + v*self.dt)
            else:
                x.append(v)         
     
        self.lxd.set_data([t[0],t[-1]], [xd, xd])
        self.lx.set_data(t, x)
        self.lu.set_data(t, u)
        self.l0.set_data([t[0],t[-1]], [0,0])
               
        self.ax.set_xlim(0, tmax)        
        self.ax.set_ylim(0, 1.2*xd)
        self.ax2.set_xlim(0, tmax)
        self.ax2.set_ylim(-1.3*fmax, 1.3*fmax)
                
        self.canvas.draw()            
