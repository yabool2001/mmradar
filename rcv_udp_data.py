# Script with minimum parsing to send binary data to UDP with period 
# sudo tcpdump -vv -A udp dst port 10005


import datetime
from multiprocessing.dummy import Process
import time
import serial
import serial.tools.list_ports
import struct
# sys.setdefaultencoding('utf-8')
from mmradar_ops import mmradar_conf
from serial_ops import open_serial_ports, set_serials_cfg , close_serial_ports , open_serial_ports
import socket
from file_ops import write_data_2_local_file


################################################################
######################## DEFINITIONS ###########################
################################################################

src_udp_port                    = 10005
src_udp_ip                     = '192.168.1.17'
dest_udp_ip                     = '192.168.1.30'

hello = "\n\n##########################################\n############# mmradar started ############\n##########################################\n"

################################################################
################ SOCKET Configuration ##########################
################################################################
#src_udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
#dest_udp.sendto ( bytes ( 'Hello' , 'utf-8') , ( dest_udp_ip , dest_udp_port ) )
src_udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
src_udp.bind ( ( dest_udp_ip , src_udp_port ) )

##################### READ DATA #################################
while True :
    data, address = src_udp.recvfrom ( 4096 )
    print("\n\n 2. Server received: ", data.decode('utf-8'), "\n\n")

################# CLOSE DATA COM PORT FILE ######################
src_udp.close ()