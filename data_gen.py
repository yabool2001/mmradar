# Aplikacja do odczytu danych z sensora i zapisu raw data do pliku i sparsowanych danych do odpowiedniego pliku

# Wdrożyć checksum bo nie wiem skąd się biorą błędy w ramkach tlv

import datetime
import multiprocessing
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

people_counting_mode            = 'pc3d'
control                         = 506660481457717506
chirp_conf                      = 0
data_com_delta_seconds          = 0.2
raws                            = bytes(1)
parsed_data_file_name           = 'mmradar_gen.parsed_data'
raw_data_file_name              = 'mmradar_gen.raw_data'
raw_data_bin_file_name          = 'mmradar_gen.bin_raw_data'
hvac_cfg_file_name              = 'chirp_cfg/sense_and_direct_68xx-mzo1.cfg'
pc3d_aop_cfg_file_name          = 'chirp_cfg/AOP_6m_default.cfg'
mmradar_stop_conf_file_name     = 'chirp_cfg/sensor_stop.cfg'
mmradar_start_conf_file_name    = 'chirp_cfg/sensor_start.cfg'

frame_header_struct = 'Q9I2H'
frame_header_length = struct.calcsize ( frame_header_struct )
frame_header_dict = dict ()
frame_header_json = None
tlv_header_struct = '2I'
tlv_header_length = struct.calcsize ( tlv_header_struct )
tlv_header_dict = dict ()
tlv_header_json = ""


################################################################
################ SERIALS COMM CONFiguration ####################
################################################################

conf_com                        = serial.Serial ()
data_com                        = serial.Serial ()


hello = "\n\n##########################################\n############# mmradar started ############\n##########################################\n"

################################################################
############ OPEN LOG, DATA AND CHIRP CONF FILE ################
################################################################
match people_counting_mode:
    case 'pc3d':
        chirp_conf_file_name = pc3d_aop_cfg_file_name
    case 'hvac':
        chirp_conf_file_name = hvac_cfg_file_name
    case _:
        print ( f'{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} Error: no chirp cfg file!' )


################################################################
################ START PROGRAM #################################
################################################################

print ( hello )

################ OPEN CONF AND DATA COM PORTS ##################
set_serials_cfg ( conf_com , data_com )
open_serial_ports ( conf_com , data_com )

################ CHIRP CONF ####################################
conf_com.reset_input_buffer()
conf_com.reset_output_buffer()
mmradar_conf ( chirp_conf_file_name , conf_com )

##################### READ DATA #################################
data_com.reset_output_buffer()
data_com.reset_input_buffer ()
frame_json_2_azure = ''
frame_json_2_file = ''
frame_read_time_up = datetime.datetime.utcnow () + datetime.timedelta ( seconds = data_com_delta_seconds )
while datetime.datetime.utcnow () < frame_read_time_up :
    # print ( datetime.datetime.utcnow ().second )
    raw_data = data_com.read ( 4666 )
    tlv_header_json = ""
    try:
        sync , version , total_packet_length , platform , frame_number , subframe_number , chirp_processing_margin , frame_processing_margin , track_process_time , uart_sent_time , num_tlvs , checksum = struct.unpack ( frame_header_struct , raw_data[:frame_header_length] )
        if sync == control :
            # frame_header_dict = { 'frame_number' : frame_number , 'num_tlvs' : num_tlvs , 'sync' : sync , 'version' : version , 'total_packet_length' : total_packet_length , 'platform' : platform , 'subframe_number' : subframe_number , 'chirp_processing_margin' : chirp_processing_margin , 'frame_processing_margin' : frame_processing_margin , 'track_process_time' : track_process_time , 'uart_sent_time' : uart_sent_time , 'checksum' : checksum }
            frame_header_dict = { 'frame_number' : frame_number , 'num_tlvs' : num_tlvs , 'total_packet_length' : total_packet_length }
        else :
            frame_header_dict = { 'error' : 'control != {sync}' }
    except struct.error as e :
        frame_header_dict = { 'error' : {e} }
    frame_header_json = f"{{'frame_header':{frame_header_dict}}}"
    if not frame_header_dict.get ( 'error' ) :
        #raw_data_s = raw_data.decode("utf-8") # need to test it and encode in reader
        with open ( raw_data_bin_file_name , 'ab' ) as f :
            f.write ( f'{raw_data}\n' )
        raw_data = raw_data[frame_header_length:]
        for i in range ( frame_header_dict.get ( 'num_tlvs' ) ) :
            try:
                tlv_type, tlv_length = struct.unpack ( tlv_header_struct , raw_data[:tlv_header_length] )
                tlv_header_dict = { 'tlv_type' : tlv_type , 'tlv_length' : tlv_length }
                if tlv_type == 7 or tlv_type == 8 :
                    print ( tlv_type )
            except struct.error as e :
                tlv_header_dict = { 'error' : {e} }
            tlv_header_json += f"'tlv_header':{tlv_header_dict}"
            raw_data = raw_data[tlv_length:]
        with open ( parsed_data_file_name , 'a' , encoding='utf-8' ) as f :
            f.write ( frame_header_json + tlv_header_json + '\n' )


##################### CLOSE DATA COM PORT ######################
close_serial_ports ( conf_com , data_com )

