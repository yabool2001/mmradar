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
from file_ops import write_data_2_local_file


################################################################
######################## DEFINITIONS ###########################
################################################################

chirp_conf                      = 0

control                         = 506660481457717506
data_com_delta_seconds          = 120
raw_data                        = bytes (1)
raw_data_bin_file_name          = f'mmradar_gen_{datetime.datetime.utcnow ()}.bin_raw_data'
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

##################### READ DATA #################################
data_com.reset_output_buffer ()
data_com.reset_input_buffer ()
f = open ( raw_data_bin_file_name , "a")
frame_read_time_up = datetime.datetime.utcnow () + datetime.timedelta ( seconds = data_com_delta_seconds )
while datetime.datetime.utcnow () < frame_read_time_up :
    raw_data = data_com.read ( 4666 )
    try:
        sync = struct.unpack ( sync_header_struct , raw_data[:sync_header_length] )
        if sync == control :
            f.write ( f'{raw_data}\n' )
    except struct.error as e :
        pass

################# CLOSE DATA COM PORT FILE ######################
close_serial_ports ( conf_com , data_com )
f.close()