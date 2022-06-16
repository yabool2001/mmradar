# Script to save binary data to file with minimum parsing

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
chirp_conf                      = 0
data_com_delta_seconds          = 60

control                         = 506660481457717506
frame                           = bytes (1)

dest_udp_ip                     = '10.0.0.157'
dest_udp_port                   = 10005

#saved_raw_data_file_name       = 'save_bin_data/mmradar_gen_1655368399032378700.bin_raw_data
saved_raw_data_file_name        = 'mmradar_gen-20220612_2.bin_raw_data'

mmradar_cfg_file_name           = 'chirp_cfg/ISK_6m_default-mmwvt-v14.11.0.cfg'
mmradar_stop_cfg_file_name      = 'chirp_cfg/sensor_stop.cfg'
mmradar_start_cfg_file_name     = 'chirp_cfg/sensor_start.cfg'

sync_header_struct = 'Q'
sync_header_length = struct.calcsize ( sync_header_struct )

hello = "\n\n##########################################\n############# mmradar started ############\n##########################################\n"

################################################################
################ SERIALS COMM CONFiguration ####################
################################################################
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

################################################################
################ SOCKET Configuration ##########################
################################################################
dest_udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )

##################### READ DATA #################################
data_com.reset_output_buffer ()
data_com.reset_input_buffer ()
saved_raw_frames = open ( saved_raw_data_file_name , 'r' ) .readlines ()
saved_raw_frames_number = len ( saved_raw_frames )
saved_raw_frame_counter = 0
frame_read_time_up = datetime.datetime.utcnow () + datetime.timedelta ( seconds = data_com_delta_seconds )
while datetime.datetime.utcnow () < frame_read_time_up and saved_raw_frame_counter < saved_raw_frames_number :
    if com_source :
        frame = data_com.read ( 4666 )
    else :
        frame = eval ( saved_raw_frames[saved_raw_frame_counter] )
        saved_raw_frame_counter += 1
    try :
        sync = struct.unpack ( sync_header_struct , frame[:sync_header_length] )
        if sync[0] == control :
            print ( 1 )
            dest_udp.sendto ( frame , ( dest_udp_ip , dest_udp_port ) )
        else :
            print ( 0 )
    except struct.error as e :
        pass

################# CLOSE DATA COM PORT FILE ######################
close_serial_ports ( conf_com , data_com )
dest_udp.close ()