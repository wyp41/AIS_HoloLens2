import socket
import time
import threading
import pybullet as p
from time import sleep
import sys
import pybullet_data
import math

host, port, i_port = "192.168.1.132", 25001, 25002
# host, port, i_port = "127.0.0.1", 25001, 25002

# host, port = "127.0.0.1", 25001


class UnityCtrlThread(threading.Thread):
    def __init__(self, host, port, i_port, name):
        threading.Thread.__init__(self,name=name)
        print("start!")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.i_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.i_sock.connect((host, i_port))
        self.cmd = "get"
        self.key = "get"
        self.mesh = ""
        self.touch = False
        self.new_cmd = False
        self.start_sim = False
        self.force = [0, 0, 0]
        self.force_pos = [0, 0, 0]
        self.f = 0.1

    def load_force_apply_client(self, basePos):
        cylinderOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
        sphereId = p.loadURDF("sphere.urdf", basePos, cylinderOrn)
        return sphereId
    
    def pos_modify(self, force, r, pos):
        import math
        for i in range(3):
            if abs(force[i]) < 0.1:
                force[i] = 0
        f = math.sqrt(force[0]**2 + force[1]**2 + force[2]**2)
        if f == 0:
            return pos
        pos[0] += r*force[0] / f
        pos[1] += r*force[1] / f
        pos[2] += r*force[2] / f
        return pos

    def comm(self):
        # time.sleep(0.5)
        while True:
            if self.new_cmd:
                self.cmd = self.key
                self.new_cmd = False
            else:
                self.cmd = "get"
            if self.cmd == "q":
                break

            if len(self.mesh) < 10:
                continue
            self.sock.sendall(self.mesh.encode("UTF-8")) #Converting string to Byte, and sending it to C#
            receivedPos = self.sock.recv(1024).decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String
            # print(receivedPos)
            time.sleep(0.02)
    
    def i_comm(self):
        # time.sleep(0.5)
        while True:
            if self.cmd == "q":
                self.i_sock.sendall(self.cmd.encode("UTF-8"))
                receivedPos = self.i_sock.recv(1024).decode("UTF-8")
                break
            self.i_sock.sendall(self.cmd.encode("UTF-8")) #Converting string to Byte, and sending it to C#
            receivedPos = self.i_sock.recv(1024).decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String
            # print(receivedPos)
            a = receivedPos.replace("(",",")
            a = a.replace(")", "")
            a = a.split(",")
            a = list(map(float, a))
            self.touch = a[0]
            self.force[0] = a[1]
            self.force[1] = a[3]
            self.force[2] = a[2]
            # self.force[0] = 0
            # self.force[1] = 0
            # self.force[2] = self.f
            self.force_pos[0] = -a[4]*10
            self.force_pos[1] = a[5]*10
            self.force_pos[2] = a[6]*10
            print("get", self.touch, self.force, self.force_pos)
            # time.sleep(0.02)
    
    def keyboard_read(self):
        while True:
            self.key = str(input("ctrl:"))
            self.new_cmd = True
            if self.key == "s":
                self.i_sock.connect((host, i_port))
                t4 = threading.Thread(target=self.i_comm)
                t4.start()
            if self.key == "a":
                self.f += 0.1
            if self.key == "q":
                break


    def get_dis(self, pos1, pos2):
        dis = 0
        for i in range(3):
            dis += (pos2[i] - pos1[i])**2
        dis = math.sqrt(dis)
        return dis

    def find_index(self, pos, mesh):
        dis = 100
        ind = 0
        for i in range(len(mesh)):
            d = self.get_dis(pos, mesh[i])
            if d < dis:
                dis = d
                ind = i
        return ind

    def pybullet_connect(self):
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

        gravZ=-10
        p.setGravity(0, 0, gravZ)

        planeId = p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1])

        clothId = p.loadSoftBody("tissue.obj", basePosition = [0,0,0], scale = 1., mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)

        p.changeVisualShape(clothId, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

        # p.createSoftBodyAnchor(clothId ,24,-1,-1)
        # p.createSoftBodyAnchor(clothId ,20,-1,-1)


        data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
        self.mesh = str(list(data)[1])
        print(self.mesh)
        mesh = list(data)[1]
        for i in range(len(mesh)):
            pos = list(mesh[i])
            if pos[0] == -1.0 and pos[1] == 1.0:
                p.createSoftBodyAnchor(clothId ,i,-1,-1)

        p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
        # p.setGravity(0,0,gravZ)
        # p.stepSimulation()
        reset_ind = True
        while p.isConnected():
            if self.cmd == "q":
                p.disconnect()
                break

            if self.touch:
                p.getCameraImage(320,200)
                data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
                self.mesh = str(list(data)[1])
                m = list(data)[1]
                if reset_ind:
                    ind = self.find_index(self.force_pos, list(data)[1])
                    reset_ind = False
                # pos = self.force_pos
                pos = list(m[ind])
                force = self.force
                print("update", force, pos)
                sphereId = self.load_force_apply_client(pos)  
                constraint_id =  p.createSoftBodyAnchor(clothId, ind, sphereId,-1, pos)

                # if reset_ind:
                #     pos = self.pos_modify(force, 0.1, pos)
                #     reset_ind = False

                pos = self.pos_modify(force, 0.1, pos)
                p.setGravity(0,0,gravZ)
                p.resetBasePositionAndOrientation(sphereId, pos, [0,0,0,1])
                p.stepSimulation()
                p.removeConstraint(constraint_id)
                p.removeBody(sphereId)
            else:
                p.getCameraImage(320,200)
                data = p.getMeshData(clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
                self.mesh = str(list(data)[1])
                reset_ind = True
                # print(self.mesh)
                p.setGravity(0,0,gravZ)
                p.stepSimulation()
            

    def run(self):
        t1 = threading.Thread(target=self.keyboard_read)
        t2 = threading.Thread(target=self.pybullet_connect)
        t3 = threading.Thread(target=self.comm)
        # t4 = threading.Thread(target=self.i_comm)
        t1.start()
        t2.start()
        t3.start()
        # t4.start()

if __name__ == "__main__":
    unity_ctrl = UnityCtrlThread(host, port, i_port, "unity_ctrl")
    unity_ctrl.start()
    unity_ctrl.join()
