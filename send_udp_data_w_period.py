# Script with minimum parsing to send binary data to UDP with period 
# sudo tcpdump -vv -A udp dst port 10005
# do odbioru służy skrypt C:\Users\mzeml\python\mmradar\rcv_udp_data.py


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

com_source                      = 1
chirp_conf                      = 2
period_send_delta_seconds       = 1

control                         = 506660481457717506
frame                           = bytes (1)

#dst_udp_ip                     = '10.0.0.157' # Lipków raspberry pi 3b+
#dst_udp_ip                      = '10.0.0.159' # Lipków raspberry pi 02w
dst_udp_ip                      = '10.0.0.5' # Lipków GO3
#dst_udp_ip                     = '192.168.1.17' # Meander raspberrypi
#dst_udp_ip                     = '192.168.1.30' # Meander MW50-SV0
#src_udp_ip                      = '10.0.0.157' # Lipków raspberry pi 3b+
src_udp_ip                      = '10.0.0.159' # Lipków raspberry pi 02w
dst_udp_port                   = 10005

#saved_raw_data_file_name       = 'save_bin_data/mmradar_gen_1655368399032378700.bin_raw_data
saved_raw_data_file_name        = 'mmradar_gen-20220612_2.bin_raw_data'

mmradar_cfg_file_name           = 'chirp_cfg/ISK_6m_default-mmwvt-v14.11.0.cfg'
mmradar_stop_cfg_file_name      = 'chirp_cfg/sensor_stop.cfg'
mmradar_start_cfg_file_name     = 'chirp_cfg/sensor_start.cfg'

frame_header_struct = 'Q9I2H'
frame_header_length = struct.calcsize ( frame_header_struct )

tlv_header_struct = '2I'
tlv_header_length = struct.calcsize ( tlv_header_struct )

hello = "\n\n##########################################\n############# mmradar started ############\n##########################################\n"

################################################################
################ SERIALS COMM CONFiguration ####################
################################################################
if com_source :
    conf_com = serial.Serial ()
    data_com = serial.Serial ()
    set_serials_cfg ( conf_com , data_com )
    open_serial_ports ( conf_com , data_com )
    conf_com.reset_input_buffer()
    conf_com.reset_output_buffer()
    if chirp_conf :
        if chirp_conf == 1 :
            mmradar_conf ( mmradar_start_cfg_file_name , conf_com )
        if chirp_conf == 2 :
            mmradar_conf ( mmradar_cfg_file_name , conf_com )
        print ("Chirp configured !")
    data_com.reset_output_buffer()
    data_com.reset_input_buffer ()

def setup_udp_timer () :
    return datetime.datetime.utcnow () + datetime.timedelta ( seconds = period_send_delta_seconds )

################################################################
################ SOCKET Configuration ##########################
################################################################
dst_udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
#dst_udp.sendto ( bytes ( 'Hello' , 'utf-8') , ( dst_udp_ip , dst_udp_port ) )

##################### READ DATA #################################
if com_source :
    data_com.reset_output_buffer ()
    data_com.reset_input_buffer ()
else :
    saved_raw_frames = open ( saved_raw_data_file_name , 'r' ) .readlines ()
    saved_raw_frames_number = len ( saved_raw_frames )
    saved_raw_frame_counter = 0
send_udp_time_up = setup_udp_timer ()
while True :
    if com_source :
        frame = data_com.read ( 4666 )
    else :
        if saved_raw_frame_counter >= saved_raw_frames_number :
            break
        frame = eval ( saved_raw_frames[saved_raw_frame_counter] )
        saved_raw_frame_counter += 1
    try:
        sync , version , total_packet_length , platform , frame_number , subframe_number , chirp_processing_margin , frame_processing_margin , track_process_time , uart_sent_time , num_tlvs , checksum = struct.unpack ( frame_header_struct , frame[:frame_header_length] )
        if sync == control :
            frame = frame[frame_header_length:]
            for i in range ( num_tlvs ) :
                try:
                    tlv_type, tlv_length = struct.unpack ( tlv_header_struct , frame[:tlv_header_length] )
                except struct.error as e :
                    frame = b''
                    break # porzucam ramkę, bo nie wiem o ile uciąć ramkę, żeby dobrać się do następnego TLV
                if tlv_type == 7 or tlv_type == 11 :
                    dst_udp.sendto ( frame[:tlv_length] , ( dst_udp_ip , dst_udp_port ) )
                    tlv_type = None
                    tlv_length = None
                    break # porzucam ramkę, bo dostałem wszystko co chciałem z tej ramki
                frame = frame[tlv_length:]
        else :
            sync = 0
            frame = b''
            continue
    except struct.error as e :
        pass
    frame = b''

################# CLOSE DATA COM PORT FILE ######################
if com_source :
    close_serial_ports ( conf_com , data_com )
dst_udp.close ()