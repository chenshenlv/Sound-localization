import socket
import crcmod
from threading import Thread
import numpy as np
import collections
import sys
from time import sleep
from time import perf_counter
import struct

class BeaconUDP(Thread):
    _payload_offset = 5
    _request_header = bytearray(b'\x47\x01\x00\x04\x00\x00\x10')
#    _request_header = bytearray(b'\x47\x05\x04\x00\x00\x2a')
    def __init__(self, ip, port, beacon_id):
        self.ip = ip
        self.port = port
        self.beacon_id = beacon_id
        self._request_header = beacon_id.to_bytes(1, byteorder='little') + self._request_header
        self.terminate = False
        maxvaluescount=3
        self.coord_arr = collections.deque([[0]*3]*maxvaluescount, maxlen=maxvaluescount) # ultrasound position buffer
        crc16 = crcmod.predefined.Crc('modbus')
        crc16.update(self._request_header)

        # generate packet CRC
        # seems like correct CRC is not necessary for request packet
        self.request_packet = self._request_header + crc16.digest()
        Thread.__init__(self)
        # create UDP socket
        
#        self.udp_socket.bind((self.ip, 49100))
        # print("Source socket binded to port:",49100)
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#            self.udp_socket.connect((self.ip, self.port))
            print("Connection established!")
        except socket.timeout:
            print('Connection timeout, check IP address and port number of the target server')
        except OSError:
            print('\nCannot build connection to the Server/Dashboard')

    def __del__(self):
        if self.udp_socket is not None:
            self.udp_socket.close()

    def close(self):
        self.terminate = True
        self.udp_socket.close()
    def request_position(self):
        return self.coord_arr[-1]

    def run(self):
        print(self._request_header)
        try:
            while (not self.terminate ):
                self.udp_socket.sendto(self.request_packet,(self.ip, self.port))
                data, address = self.udp_socket.recvfrom(1024)
#                print (data)
                # parse the packet, '<' for little endian; 'L' for unsigned long(4);
                # 'h' for short integer(2); 'B06' for unsigned char(1); 'x' for pad byte(1); 'H' for unsigned short(2)
                # details of String Format: https://docs.python.org/3.5/library/struct.html
                timestamp, coord_x, coord_y, coord_z, data_crc16 = struct.unpack_from('<LhhhxxxxxxH',
                                                                                      data, self._payload_offset)
#                x, y, z, qw, qx, qy, qz, vx, vy, vz, ax, ay, az, HedgeAdr, timestamp, usnCRC16 = struct.unpack_from ('<lllhhhhhhhhhhBxLxxxxH', data, self._payload_offset)
                self.coord_arr.append(np.array([coord_x*0.01,coord_y*0.01,coord_z*0.01]))
#                self.coord_arr.append(np.array([x, y, z, vx, vy, vz, ax, ay, az]))
                sleep(0.005)
        except KeyboardInterrupt:
            self.close()
            sys.exit()
            

#def udp_factory(ip, port, beacon_add):
#    udp = BeaconUDP(ip, port, beacon_add)
#    return udp
                        
#from Kalman import Kalman
#k_filter=Kalman() 
#x=np.array([0,2,3,5,8,9,10])
#y=np.array([1,2,5,7,10,11,13])
#for i in range(x.size):
#    x_pre,y_pre=k_filter.K_filter(x[i],y[i])
#    print('x,y:',np.array([x_pre,y_pre]))
