
from time import sleep
from pySerialTransfer import pySerialTransfer as txfer
import time
import logging
from delta import Delta



# =============================================
# Set logging formats
logging.basicConfig(
    level=logging.INFO,
    format=("[%(filename)8s] [%(levelname)4s] :  %(funcName)s - %(message)s"),
)
# =============================================


delta = Delta(e=0.138,f=0.474,re=0.384,rf=0.18)


x0 = delta.calcForward([40.0,40.0,40.0])

valid, thetaW1 = delta.inverse(*[-0.1, 0.0, -0.4194879595738985+0.1])
valid, thetaW2 = delta.inverse(*[+0.1, 0.0, -0.4194879595738985+0.0])
valid, thetaW3 = delta.inverse(*[0.0, -0.1, -0.4194879595738985+0.1])
valid, thetaW4 = delta.inverse(*[0.0, +0.0, -0.2])

thetaW4 = [0,0,0]
print(thetaW1)
print(thetaW2)
print(thetaW3)
print(thetaW4)

# =========================
class ControllStruct(object):
    theta1 = 0
    theta2 = 0
    theta3 = 0
    enabled = 0
    home = 0


class StateStruct(object):
    theta1 = 0
    theta2 = 0
    theta3 = 0
    enabled = 0
    home = 0
    status = 200


controllStruct = ControllStruct
stateStruct = StateStruct


# =========================
class SerialCommunication(object):
    def __init__(self, comPort='COM4'):
        self.link = txfer.SerialTransfer(comPort)
        self.link.open()
        sleep(5)
        logging.info("starting serial....")


    def close(self):
        self.link.close()

    def send2robot(self, theta1, theta2, theta3, home):
        sendSize = 0
        sendSize += self.link.tx_obj([theta1,theta2,theta3,home], start_pos=sendSize)
        self.link.send(sendSize)


    def wait4response(self):
        while not self.link.available():
            if self.link.status < 0:
                if self.link.status == txfer.CRC_ERROR:
                    logging.error('ERROR: CRC_ERROR')
                elif self.link.status == txfer.PAYLOAD_ERROR:
                    logging.error('ERROR: PAYLOAD_ERROR')
                elif self.link.status == txfer.STOP_BYTE_ERROR:
                    logging.error('ERROR: STOP_BYTE_ERROR')
                else:
                    logging.error('ERROR: {}'.format(self.link.status))


    def recieveFromRobot(self):

        recSize = 0

        stateStruct.theta1 = self.link.rx_obj(obj_type='f', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
        
        stateStruct.theta2 = self.link.rx_obj(obj_type='f', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
        
        stateStruct.theta3 = self.link.rx_obj(obj_type='f', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

        stateStruct.enabled = self.link.rx_obj(obj_type='i', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['i']
        
        stateStruct.home = self.link.rx_obj(obj_type='i', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['i']
        
        stateStruct.status = self.link.rx_obj(obj_type='i', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['i']

        logging.info(f"{stateStruct.theta1}, {stateStruct.theta2}, {stateStruct.theta3}, {stateStruct.enabled}, {stateStruct.home}, {stateStruct.status}")

        return stateStruct.status

# =========================

serialComm = SerialCommunication()
commandAvailable = True

waypoints = [thetaW1, thetaW2, thetaW3, thetaW4]
idx = 0
a = True

try:

    while True:

        # Awaiting commands
        serialComm.wait4response()
        status = serialComm.recieveFromRobot()


        if status == 200 and commandAvailable:

                # Ready to take orders
                theta1 = int(waypoints[idx][0])
                theta2 = int(waypoints[idx][1])
                theta3 = int(waypoints[idx][2])

                print(theta1,theta2, theta3)
                serialComm.send2robot(theta1, theta2, theta3, False)
                idx += 1
                a = False
           

        if idx >= len(waypoints):
            idx = 0




   
except KeyboardInterrupt:
    serialComm.close()


