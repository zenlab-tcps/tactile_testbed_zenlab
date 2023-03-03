# import the opencv library
import cv2

import socket
import struct
import pickle
import numpy as np

HOST = '127.0.0.1'
PORT = 12001

MAX_DGRAM = 2**16

def dump_buffer(s):
    """ Emptying buffer frame """
    while True:
        msg, addr = s.recvfrom(MAX_DGRAM)
        print(msg[0])
        if struct.unpack('B', msg[0:1])[0] == 1:
            print("finish emptying buffer")
            break

def VideoFeedback():
    """ Getting image udp frame &
    concate before decode and output image """

    # Create receive socket
    UDPVideoRecv = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    # bind receive port
    UDPVideoRecv.bind((HOST, PORT))

    dat = b''
    dump_buffer(UDPVideoRecv)

    while True:
        msg, addr = UDPVideoRecv.recvfrom(MAX_DGRAM)
        if struct.unpack('B', msg[0:1])[0] > 1:
            dat += msg[1:]
        else:
            dat += msg[1:]
            img = cv2.imdecode(np.fromstring(dat, dtype=np.uint8), 1)

            cv2.imshow('Video Feedback', img)
            dat = b''
        # msg = pickle.loads(msg)
        # img = cv2.imdecode(msg, 1)
        #
        # print (msg)
        # cv2.imshow('frame',img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    UDPVideoRecv.close()

if __name__ == "__main__":
    VideoFeedback()
