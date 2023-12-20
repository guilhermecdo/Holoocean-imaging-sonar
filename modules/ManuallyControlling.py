import evdev

class keyboard:
    def __init__(self) -> None:
        self.device=evdev.InputDevice()
        self.pressed_keys=list()
        self.thrust=0.0
        

class joystick:
    def __init__(self)->None:
        self.device=evdev.InputDevice('/dev/input/event12')
        self.buttons={
            "A":False,
            "B":False,
            "X":False,
            "Y":False,
            "UP":False,
            "DOWN":False,
            "LEFT":False,
            "RIGHT":False,
            "RB":False,
            "RT":False,
            "RC":False,
            "LB":False,
            "LT":False,
            "LC":False,
            "START":False,
            "BACK":False
        }
        self.axis={
           "vy0":0.0,
            "vx0":0.0,
            "vy1":0.0,
            "vx1":0.0 
        }    

    def read(self)->None:

        if 304 in self.device.active_keys():   self.buttons["A"]=True
        else:  self.buttons["A"]=False

        if 305 in self.device.active_keys(): self.buttons["B"]=True
        else:self.buttons["B"]=False

        if 307 in self.device.active_keys(): self.buttons["X"]=True
        else:self.buttons["X"]=False

        if 308 in self.device.active_keys(): self.buttons["Y"]=True
        else:self.buttons["Y"]=False

        if 310 in self.device.active_keys(): self.buttons["LB"]=True
        else:self.buttons["LB"]=False
        
        if 311 in self.device.active_keys(): self.buttons["RB"]=True
        else:self.buttons["RB"]=False
        
        if 312 in self.device.active_keys(): self.buttons["LT"]=True
        else:self.buttons["LT"]=False
        
        if 313 in self.device.active_keys(): self.buttons["RT"]=True
        else:self.buttons["RT"]=False
        
        if 314 in self.device.active_keys(): self.buttons["BACK"]=True
        else:self.buttons["BACK"]=False
        
        if 315 in self.device.active_keys(): self.buttons["START"]=True
        else:self.buttons["START"]=False
        
        if 317 in self.device.active_keys(): self.buttons["LC"]=True
        else:self.buttons["LC"]=False
        
        if 318 in self.device.active_keys(): self.buttons["RC"]=True
        else:self.buttons["RC"]=False
        
        if 704 in self.device.active_keys(): self.buttons["LEFT"]=True
        else:self.buttons["LEFT"]=False
        
        if 705 in self.device.active_keys(): self.buttons["RIGHT"]=True
        else:self.buttons["RIGHT"]=False
        
        if 706 in self.device.active_keys(): self.buttons["UP"]=True
        else:self.buttons["UP"]=False
        
        if 707 in self.device.active_keys(): self.buttons["DOWN"]=True
        else:self.buttons["DOWN"]=False

        self.axis={
            "thrust":round(self.device.absinfo(1)[0]*-1/32767,2),
            "yaw":round(self.device.absinfo(0)[0]/32767,2),
            "roll":round(self.device.absinfo(4)[0]*-1/32767,2),
            "pitch":round(self.device.absinfo(3)[0]/32767,2),
        }
