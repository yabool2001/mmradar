# Script to save binary data to file with minimum parsing

import struct
import socket
import time

################################################################
######################## DEFINITIONS ###########################
################################################################

#dst_udp_ip                      = '10.0.0.157' # Lipków raspberry pi 3b+
#dst_udp_ip                      = '10.0.0.159' # Lipków raspberry pi 02w
#dst_udp_ip                      = '10.0.0.5' # Lipków GO3
#dst_udp_ip                      = '192.168.1.17' # Meander raspberrypi
#dst_udp_ip                      = '192.168.1.30' # Meander MW50-SV0
#src_udp_ip                      = '10.0.0.5' # Lipków GO3
#src_udp_ip                      = '10.0.0.157' # Lipków raspberry pi 3b+
#src_udp_ip                      = '10.0.0.159' # Lipków raspberry pi 02w
dst_udp_ip                      = '192.168.1.30' # Meander MW50-SV0
ctrl_udp_port                    = 10004
data_udp_port                    = 10005

#saved_raw_data_file_name       = 'save_bin_data/mmradar_gen_1655368399032378700.bin_raw_data
#saved_raw_data_file_name        = 'mmradar_gen-20220612_2.bin_raw_data'

ctrl_struct = 'B'
ctrl_length = struct.calcsize ( ctrl_struct )
ctrl_exit = b'0'
ctrl_received = None

hello = "\n\n##########################################\n######### test_rcv_receive_stop #########\n##########################################\n"

################################################################
################ SCRIPT START ##################################
################################################################
print ( hello )

################################################################
################ SOCKET Configuration ##########################
################################################################
#dst_udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
udp.bind ( ( dst_udp_ip , ctrl_udp_port ) )
udp.settimeout ( 0 )
#udp.setblocking ( 0 )
################ MAIN ##########################################

def listen () :
    udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
    udp.bind ( ( dst_udp_ip , ctrl_udp_port ) )
    #udp.settimeout ( 0 )
    try :
        ctrl , address = udp.recvfrom ( 4666 )
        try :
            ctrl = struct.unpack ( ctrl_struct , ctrl[:ctrl_length] )
            #print ( int.from_bytes ( ctrl , byteorder='big' ) )
            #print ( ctrl[0] )
            if ctrl[0] == int.from_bytes ( ctrl_exit , byteorder='big' ) :
                print ("Ctrl exit received!")
                global ctrl_received
                ctrl_received = 0
        except struct.error as e :
            pass
    except socket.error as e :

while True :
    try :
        ctrl , address = udp.recvfrom ( 4666 )
        try :
            ctrl = struct.unpack ( ctrl_struct , ctrl[:ctrl_length] )
            print ( int.from_bytes ( ctrl , byteorder='big' ) )
            print ( ctrl[0] )
            if ctrl[0] == int.from_bytes ( ctrl_exit , byteorder='big' ) :
                print ("Ctrl exit received!")
                break
        except struct.error as e :
            pass
    except socket.error as e :
        pass
    time.sleep ( 1 )
    print ( 'Go' )
################# CLOSE DATA COM PORT FILE #####################
udp.close ()
