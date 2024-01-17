# controllers.py
# class implementation of controllers
# dew.ninja  Sep 2023


class BaseController():
    def __init__(self,umin,umax,T):
        self.umin = umin
        self.umax = umax
        self.umid = (umax - umin)/2
        self.T = T  # sampling period
        self.e0 = 0.0
        self.e1 = 0.0
        self.u0 = 0.0
        self.u1 = 0.0
        self.ulim = 0.0
        
        # integrator states. Shared by LLIC and SFIC
        self.u0_int = 0.0 # output
        self.u1_int = 0.0 # past-output
        self.e0_int = 0.0 # input
        self.e1_int = 0.0 # past-input
        self.a_int = 0.5*self.T
    def gett(self):
        return self.T
    def sett(self,T):
        self.T = T
        self.a_int = 0.5*self.T        
    def integrator(self,inp):  # integrator function
        self.e1_int = self.e0_int
        self.e0_int = inp
        self.u1_int = self.u0_int
        self.u0_int = self.u1_int + self.a_int*(self.e0_int + self.e1_int)
        return self.u0_int
        
    def limit_and_offset(self,u0):
        ulim = u0
        if u0 > self.umid:
            ulim = self.umid         # limit u to self.umid
        elif u0 < -self.umid:
            ulim = -self.umid         # limit u to -self.umid
        ulim += self.umid
        return ulim
        
class PIDControl(BaseController): # PID controller class
    def __init__(self,umin, umax,T,kp,ki,kd,kt,N,wp,wd):
        super().__init__(umin,umax,T)
        # PID parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kt = kt
        self.N = N
        self.wp = wp
        self.wd = wd

        # controller states
        #self.e1 = 0  # in base class
        #self.e0 = 0
        self.ed1 = 0
        self.ed0 = 0
        self.eus1 = 0
        self.eus0 = 0
        self.up0 = 0
        self.ui1 = 0
        self.ui0 = 0
        self.ud1 = 0
        self.ud0 = 0
        #self.u = 0  # in base class
        #self.ulim = 0
        self.update()
    # parameter getters & setters 
    def getkp(self):
        return self.kp
    def getki(self):
        return self.ki
    def getkd(self):
        return self.kd
    def getkt(self):
        return self.kt
    def getn(self):
        return self.N
    def getwp(self):
        return self.wp
    def getwd(self):
        return self.wd

    def setkp(self,kp):
        self.kp = kp
    def setki(self,ki):
        self.ki = ki
    def setkd(self,kd):
        self.kd = kd
    def setkt(self,kt):
        self.kt = kt
    def setn(self,N):
        self.N = N
    def setwp(self,wp):
        self.wp = wp
    def setwd(self,wd):
        self.wd = wd
    def setparms(self,parms):
        self.kp = parms[0]
        self.ki = parms[1]
        self.kd = parms[2]
        self.kt = parms[3]
        self.N = parms[4]
        self.wp = parms[5]
        self.wd = parms[6]
        self.T = parms[7]
   
    # coefficient update
    def update(self):
        self.bi = 0.5*self.T*self.ki
        self.bt = 0.5*self.T*self.kt
        ad1 = 1+0.5*self.N*self.T
        ad2 = 0.5*self.N*self.T - 1
        self.ad = -ad2/ad1
        self.bd = self.kd*self.N/ad1
        
    # reset controller states
    def reset(self):
        self.e1 = 0
        self.e0 = 0
        self.ed1 = 0
        self.ed0 = 0
        self.eus1 = 0
        self.eus0 = 0
        self.up0 = 0
        self.ui1 = 0
        self.ui0 = 0
        self.ud1 = 0
        self.ud0 = 0
        self.u0 = 0
        self.ulim = 0

    def out(self,r,y):
        # state transfer
        self.e1 = self.e0
        self.ed1 = self.ed0
        self.eus1 = self.eus0
        
        self.ui1 = self.ui0
        self.ud1 = self.ud0
        # compute errors for each term
        self.e0 = r - y
        self.ep0 = self.wp*r - y # weighted proportional error
        self.ed0 = self.wd*r - y # weighted derivative error
        
        self.up0 = self.kp*self.ep0 # output of P term
        self.ui0 = self.ui1 +self.bi*(self.e0+self.e1) + self.bt*(self.eus0+self.eus1) # output of I term
        self.ud0 = self.ad*self.ud1 +self.bd*(self.ed0 - self.ed1) # output of D term
        self.u0 = self.up0 + self.ui0 + self.ud0
        self.ulim = self.limit_and_offset(self.u0)                
        return self.ulim      



class LLIControl(BaseController): # Lead-lag + Integrator class
    def __init__(self,umin, umax,T,llk,llz,llp):
        super().__init__(umin,umax,T)
        self.llk = llk # lead-lag gain
        self.llz = llz # lead-lag zero
        self.llp = llp # lead-lag pole
        self.g1,self.g2,self.g3 = self.ll_coeff_compute()

    def ll_coeff_compute(self): 
        alpha1 = 2+self.llz*self.T
        alpha2 = self.llz*self.T - 2
        beta1 = 2+self.llp*self.T
        beta2 = self.llp*self.T-2
        gamma1 = -beta2/beta1
        gamma2 = alpha1/beta1
        gamma3 = alpha2/beta1
        return gamma1, gamma2, gamma3

    def update(self):
        self.g1,self.g2,self.g3 = self.ll_coeff_compute()
        
    def out(self,r,y):
        self.e1 = self.e0
        self.e0 = r-y
        self.u1 = self.u0
        self.u0 = self.g1*self.u1 + self.g2*self.e0 + self.g3*self.e1
        u0_int = self.llk*self.integrator(self.u0)
        self.ulim = self.limit_and_offset(u0_int)                
        return self.ulim     

class SFIControl(BaseController): # State Feedback + Integrator class
    def __init__(self,umin, umax,T,sfi_k, sfi_ki):
        super().__init__(umin,umax,T)
        self.sfi_k = sfi_k  # vector of state-feedback gain
        self.sfi_ki = sfi_ki  # integrator gain
    def out(self,r,x1,x2,x3):
        self.u0 = (self.sfi_ki*self.integrator(r-x3) -self.sfi_k[0]*x1
                   -self.sfi_k[1]*x2 - self.sfi_k[2]*x3)
        self.ulim = self.limit_and_offset(self.u0)
        return self.ulim