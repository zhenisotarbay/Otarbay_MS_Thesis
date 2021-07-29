try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
import time
import numpy as np




class list_holder():
    
    def __init__(self, enc_zero_cnt, frs_zero_cnt, init_deg, enc_val, rt_val, frs_val, rt_arr, n_cnt):
        self.enc_zero_cnt = enc_zero_cnt
        self.frs_zero_cnt = frs_zero_cnt
        self.init_deg = init_deg
        self.enc_val = enc_val
        self.rt_val = rt_val
        self.frs_val = frs_val
        self.rt_arr = rt_arr
        self.n_cnt = n_cnt

class list_coord():
    
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class simProc(QObject):

    finished = pyqtSignal(list_coord)

    def __init__(self, client_id):
        QThread.__init__(self)
        active = True
        self.clientID = client_id

    def __del__(self):
        self.wait()

    def _sim_mag(self, angMag):

        robotMode = 1
        sim.simxSetIntegerSignal(self.clientID, "robotMode", robotMode, sim.simx_opmode_blocking)

        self.mot = [0, 0, 0, 0, 0]
        current_x=0
        current_y=0
        current_z=0
        print("simulation started, robotMode = 1")
        

        sim.simxSetIntegerSignal(self.clientID, "angMag", 0, sim.simx_opmode_blocking)
        time.sleep(2)
        print("initialized")

        # Sending signal to the Robot Model in V-REP
        sim.simxSetIntegerSignal(self.clientID, "angMag", angMag, sim.simx_opmode_blocking)

        xa_list=[]
        ya_list=[]
        za_list=[]


        for n in range(angMag*4):
            if (self.active == False):
                break;
            n = n//4
            sim.simxSetIntegerSignal(self.clientID, "angMag", n, sim.simx_opmode_blocking)
            [r,h]=sim.simxGetObjectHandle(self.clientID,'platform_center', sim.simx_opmode_blocking)
            positions=sim.simxGetObjectPosition(self.clientID,h,-1,sim.simx_opmode_blocking)
            xa=np.linspace(current_x,positions[1][0])
            current_x=positions[1][0]
            xa_list.append(current_x)
            ya=np.linspace(current_y,positions[1][1])
            current_y=positions[1][1]
            ya_list.append(current_y)
            za=np.linspace(current_z,positions[1][2])
            current_z=positions[1][2]
            za_list.append(current_z)
            print(n, positions)
            time.sleep(0.1)

        print("end mag")
        
        ret_obj = list_coord(xa_list,ya_list,za_list)
        return ret_obj

    def run(self, angMag):
        self.active = True
        obj_lists = self._sim_mag(angMag)
        self.finished.emit(obj_lists)

    def stop(self):
        self.active = False


class fetchData(QObject):

    finished = pyqtSignal(list_holder)
    
    def __init__(self):
        QThread.__init__(self)
        active = True

    def __del__(self):
        self.wait()

    def _fetch(self):
        n_cnt = 0

        enc_list = []
        init_deg = [0, 0, 0, 0, 0]
        
        enc_val = [[], [], [], [], []]
        rt_val = [[], [], [], [], []]
        rt_arr = [[], [], [], [], []]
        
        frs_list = []
        frs_val = [[], [], [], []]

        enc_zero_cnt = [0, 0, 0, 0, 0]
        frs_zero_cnt = [0, 0, 0, 0, 0]
        
        ser=serial.Serial('COM3',9600)
        
        print("Start reading...")
        
        while(n_cnt<=700):
            line = ser.readline()
            line_arr = str(line).split("&")

            frs = line_arr[0]
            if (len(frs)<2 or not ("fs" in frs)):
                print("input not valid")
                break
            print(frs)
            if ("fs" in frs):
                frs_list = frs.split("|", 5)
                for i in range(1, 4):
                    if float(frs_list[i])==0.0:
                        frs_zero_cnt[i] += 1
                    frs_val[i].append(float(frs_list[i]))

            enc = line_arr[1]
            print(enc)            
            if ("enc" in enc):
                enc_list = enc.split("|", 10)
                if (n_cnt == 0):
                    init_deg[1:5] = enc_list[1:5]

                for i in range(1, 5):
                    if int(enc_list[i])==0:
                        enc_zero_cnt[i] += 1
                    enc_val[i].append(int(enc_list[i]))
                
                    if ":" in enc_list[i+4]:
                        val = enc_list[i+4]
                        rt_arr[i] = val.split(":",2)
                        rt_val[i].append(int(rt_arr[i][1]))

            if (self.active != True or enc_list[9]=="off"):
                self.enc_last = enc_list
                if enc_list[9]=="off":
                    print("Limit reached. Please rotate in different direction")
                break;
     
            n_cnt = n_cnt + 1

        ser.close()
        print("End")

        self.ret_lists = list_holder(enc_zero_cnt, frs_zero_cnt, init_deg, enc_val, rt_val, frs_val, rt_arr, n_cnt)
        return self.ret_lists
        
    def run(self):
        self.active = True
        obj_lists = self._fetch()
        self.finished.emit(obj_lists)

    def stop(self):
        self.active = False
