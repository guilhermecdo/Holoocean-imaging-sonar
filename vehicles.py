import numpy as np

class ROV:
    def __init__(self, id:int, controlScheme="keyboard")->None:
        self.id=id
        self.controlScheme=controlScheme

        self.position={
            "x0":0.0,
            "y0":0.0,
            "z0":0.0,
            "theta0":0.0,
            "phi0":0.0,
            "psi0":0.0
        }

        self.setPoint={
            "xf":0.0,
            "yf":0.0,
            "zf":0.0,
            "thetaf":0.0,
            "phif":0.0,
            "psif":0.0
        }

        self.thruster={
            "t1":0.0,
            "t2":0.0,
            "t3":0.0,
            "t4":0.0,
            "t5":0.0,
            "t6":0.0,
            "t7":0.0,
            "t8":0.0
        }

    def setControlScheme(self,scheme:str)->None:
        possibilites=["joystick","keyboard","autonumos"]
        
        if scheme in possibilites:
            self.controlScheme=scheme

        else: print("invalid control scheme")

    def setSetPoint(self,xf:float, yf:float,zf:float, thetaf:float,phif:float,psif:float)->None:
        self.setPoint={
            "xf":xf,
            "yf":yf,
            "zf":zf,
            "thetaf":thetaf,
            "phif":phif,
            "psif":psif
        }

    def setThrusterVelocity(self)->bool:
        delta={
            "dx":self.setPoint["xf"]-self.position["x0"],
            "dy":self.setPoint["yf"]-self.position["y0"],
            "dz":self.setPoint["zf"]-self.position["z0"],
            "dtheta":self.setPoint["thetaf"]-self.position["theta0"],
            "dphi":self.setPoint["phif"]-self.position["phi0"],
            "dpsi":self.setPoint["psif"]-self.position["psi0"]
        }

        self.thruster={
            "t1":-delta["dx"] - delta["dy"] + delta["dz"] - delta["dtheta"] - delta["dphi"] + delta["dpsi"],
            "t2":-delta["dx"] - delta["dy"] + delta["dz"] - delta["dtheta"] - delta["dphi"] + delta["dpsi"],
            "t3":-delta["dx"] + delta["dy"] + delta["dz"] - delta["dtheta"] - delta["dphi"] - delta["dpsi"],
            "t4":-delta["dx"] + delta["dy"] + delta["dz"] - delta["dtheta"] - delta["dphi"] - delta["dpsi"],
            "t5": delta["dx"] + delta["dy"] + delta["dz"] + delta["dtheta"] + delta["dphi"] + delta["dpsi"],
            "t6": delta["dx"] + delta["dy"] + delta["dz"] + delta["dtheta"] + delta["dphi"] + delta["dpsi"],
            "t7": delta["dx"] - delta["dy"] + delta["dz"] + delta["dtheta"] - delta["dphi"] + delta["dpsi"],
            "t8": delta["dx"] - delta["dy"] + delta["dz"] + delta["dtheta"] - delta["dphi"] + delta["dpsi"]
        }
    
    def updatePosition(self,x,y,z,theta,phi,psi):
        
        pass


    def move(self)->None:
        if self.controlScheme=="joystick":
            pass
        elif self.controlScheme=="keyboard":
            pass
        elif self.controlScheme=="autonumos":
            pass