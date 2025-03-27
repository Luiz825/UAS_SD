import time
import asyncio
import multiprocessing

class PID:
    def __init__(self, Ki, Kd, Kp, setpt = 0):
        self.Ki = Ki
        self.Kd = Kd
        self.Kp = Kp
        self.setpt = setpt
        self.last_err = 0
        self.integral = 0
        self.last_tm = time.time()
# mostly from https://medium.com/@aleksej.gudkov/python-pid-controller-example-a-complete-guide-5f35589eec86
    def compute(self, curr, t):
        # Calculate error
        error = self.setpt - curr
        
        # Proportional term
        P_out = self.Kp * error
        dt = t - self.last_tm

        # Integral term
        self.integral += error * dt
        I_out = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.last_err) / dt
        D_out = self.Kd * derivative
        
        # Compute total output
        output = P_out + I_out + D_out
        
        # Update previous error
        self.previous_error = error
        self.last_tm = t
        
        return output
    
    