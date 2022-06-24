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
dst_udp_ip                      = '192.168.1.30' # Meander MW50-SV0
#src_udp_ip                      = '10.0.0.5' # Lipków GO3
#src_udp_ip                      = '10.0.0.157' # Lipków raspberry pi 3b+
#src_udp_ip                      = '10.0.0.159' # Lipków raspberry pi 02w
ctrl_udp_port                    = 10004
data_udp_port                    = 10005

#saved_raw_data_file_name       = 'save_bin_data/mmradar_gen_1655368399032378700.bin_raw_data
#saved_raw_data_file_name        = 'mmradar_gen-20220612_2.bin_raw_data'

ctrl_struct = 'B'
ctrl_length = struct.calcsize ( ctrl_struct )
ctrl_exit = b'0'

hello = "\n\n##########################################\n######### test_send_udp_ctrl_stop ########\n##########################################\n"

################################################################
################ SCRIPT START ##################################
################################################################
print ( hello )

################################################################
################ SOCKET Configuration ##########################
################################################################
#dst_udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM ) #, socket.IPPROTO_UDP )
#udp.bind ( ( src_udp_ip , ctrl_udp_port ) )
################ MAIN ##########################################
while True :
    udp.sendto ( ctrl_exit , ( dst_udp_ip , ctrl_udp_port ) )
    print ( 'ctrl_exit sent' )
    time.sleep ( 5 )
################# CLOSE DATA COM PORT FILE #####################
udp.close ()
