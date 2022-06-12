# Aplikacja do odczytu danych z sensora i zapisu raw data do pliku i sparsowanych danych do odpowiedniego pliku

# Znaleźć błąd w target index albo zadać pytanie na formu o same 255
# Wdrożyć checksum bo nie wiem skąd się biorą błędy w ramkach tlv

import datetime
#import multiprocessing
from multiprocessing.dummy import Process
from pprint import pprint
import time
from numpy import append
import serial
import serial.tools.list_ports
import struct
# sys.setdefaultencoding('utf-8')
from mmradar_ops import mmradar_conf
from serial_ops import open_serial_ports, set_serials_cfg , close_serial_ports , open_serial_ports
from file_ops import write_data_2_local_file
import tkinter as tk

import PointCloud
import Presence
import TargetIndex
import TargetList

################################################################
######################## DEFINITIONS ###########################
################################################################
com_source                      = 1
chirp_conf                      = 0
data_com_delta_seconds          = 120

control                         = 506660481457717506
raws                            = bytes(1)
frame                           = bytes(1)
raw_data_bin_file_name          = 'mmradar_gen.bin_raw_data'
saved_raw_data_file_name        = 'mmradar_gen_good2.bin_raw_data'
parsed_data_file_name           = 'mmradar_gen.parsed_data'
mmradar_cfg_file_name           = 'chirp_cfg/ISK_6m_default-mmwvt-v14.11.0.cfg'
mmradar_stop_cfg_file_name     = 'chirp_cfg/sensor_stop.cfg'
mmradar_start_cfg_file_name    = 'chirp_cfg/sensor_start.cfg'

frames_list = []

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

################################################################
################ START PROGRAM #################################
################################################################

print ( hello )

################ OPEN FILE WITH SAVED RAW DATA #################
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
    frame_dict = { 'id' : time.time_ns() }
    try:
        sync , version , total_packet_length , platform , frame_number , subframe_number , chirp_processing_margin , frame_processing_margin , track_process_time , uart_sent_time , num_tlvs , checksum = struct.unpack ( frame_header_struct , frame[:frame_header_length] )
        if frame_number == 69721 :
            pass
        if sync == control :
            #frame_dict.update ( { 'frame_number' : frame_number , 'num_tlvs' : num_tlvs , 'sync' : sync , 'version' : version , 'total_packet_length' : total_packet_length , 'platform' : platform , 'subframe_number' : subframe_number , 'chirp_processing_margin' : chirp_processing_margin , 'frame_processing_margin' : frame_processing_margin , 'track_process_time' : track_process_time , 'uart_sent_time' : uart_sent_time , 'checksum' : checksum } )
            frame_dict.update ( { 'frame_number' : frame_number , 'num_tlvs' : num_tlvs } )
        else :
            frame_dict.update ( { 'frame header error' : 'control != {sync}' } )
    except struct.error as e :
        frame_dict.update ( { 'frame header error' : {e} } )
    if not frame_dict.get ( 'frame header error' ) :
        #print ( frame_number )
        if com_source :
            with open ( raw_data_bin_file_name , 'a' ) as f :
                f.write ( f'{frame}\n' )
        frame = frame[frame_header_length:]
        tlv_type_list = []
        for i in range ( frame_dict.get ( 'num_tlvs' ) ) :
            try:
                tlv_type, tlv_length = struct.unpack ( tlv_header_struct , frame[:tlv_header_length] )
                tlv_type_list.append ( tlv_type )
            except struct.error as e :
                frame_dict.update ( { 'tlv header error' : {e} } )
            match tlv_type :
                case 6 :
                    point_cloud = PointCloud.PointCloud ( tlv_length - tlv_header_length , frame[tlv_header_length:][:(tlv_length - tlv_header_length )] )
                    frame_dict.update ( point_cloud_unit = point_cloud.get_point_unit_dict () )
                    frame_dict.update ( points = point_cloud.get_points_list () )
                case 7 :
                    target_list = TargetList.TargetList ( tlv_length - tlv_header_length , frame[tlv_header_length:][:( tlv_length - tlv_header_length )] )
                    frame_dict.update ( target_list = target_list.get_target_list () )
                case 8 :
                    target_index_list = TargetIndex.TargetIndex ( tlv_length - tlv_header_length , frame[tlv_header_length:][:( tlv_length - tlv_header_length )] )
                    frame_dict.update ( target_index_list = target_index_list.get_target_index_list () )
                    #for target_id in frame_dict['target_index_list'] :
                    #    if target_id < 253 :
                    #        print ( f"{target_id} {frame_dict['frame_number']}" )
                case 11 :
                    presence = Presence.Presence ( tlv_length - tlv_header_length , frame[tlv_header_length:][:( tlv_length - tlv_header_length )] )
                    frame_dict.update ( presence.get_presence_dict () )
            frame = frame[tlv_length:]
            tlv_type = None
            tlv_length = None
        frame_dict.update ( tlv_type_list = tlv_type_list )
    frames_list.append ( frame_dict )
    del ( frame_dict )
with open ( parsed_data_file_name , 'a' , encoding='utf-8' ) as f :
    f.write ( f'{frames_list}' + '\n\r' )
#pprint ( frames_list )
frames_list.clear ()

if com_source :
    close_serial_ports ( conf_com , data_com )
