# Aplikacja do odczytu danych z sensora i zapisu raw data do pliku i sparsowanych danych do odpowiedniego pliku

# Znaleźć błąd w target index albo zadać pytanie na formu o same 255
# Wdrożyć checksum bo nie wiem skąd się biorą błędy w ramkach tlv

from asyncio.windows_events import NULL
import datetime
import multiprocessing
from multiprocessing.dummy import Process
from pprint import pprint
import time
from xml.etree.ElementTree import tostring
from numpy import append
import serial
import serial.tools.list_ports
import struct
import PointCloud
# sys.setdefaultencoding('utf-8')
from mmradar_ops import mmradar_conf
from serial_ops import open_serial_ports, set_serials_cfg , close_serial_ports , open_serial_ports
from file_ops import write_data_2_local_file
import tkinter as tk
import Presence
import TargetIndex
import TargetList

################################################################
######################## DEFINITIONS ###########################
################################################################

people_counting_mode            = 'pc3d'
control                         = 506660481457717506
chirp_conf                      = 0
data_com_delta_seconds          = 60
raws                            = bytes(1)
frame                           = bytes(1)
saved_raw_data_file_name        = 'mmradar_gen_good.bin_raw_data'
parsed_data_file_name           = 'mmradar_gen.parsed_data'
hvac_cfg_file_name              = 'chirp_cfg/sense_and_direct_68xx-mzo1.cfg'
pc3d_cfg_file_name              = 'chirp_cfg/ISK_6m_default-mmwvt-v14.11.0.cfg'
mmradar_stop_conf_file_name     = 'chirp_cfg/sensor_stop.cfg'
mmradar_start_conf_file_name    = 'chirp_cfg/sensor_start.cfg'

frames_list = []
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
        chirp_conf_file_name = pc3d_cfg_file_name
    case 'hvac':
        chirp_conf_file_name = hvac_cfg_file_name
    case _:
        print ( f'{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} Error: no chirp cfg file!' )


################################################################
################ START PROGRAM #################################
################################################################

print ( hello )

################ OPEN FILE WITH SAVED RAW DATA #################
saved_raw_frames = open ( saved_raw_data_file_name , 'r' ) .readlines ()

frame_read_time_up = datetime.datetime.utcnow () + datetime.timedelta ( seconds = data_com_delta_seconds )
for saved_raw_frame in saved_raw_frames :
    frame = eval ( saved_raw_frame )
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
        frame_header_dict = { 'error' : {e} }
    if not frame_header_dict.get ( 'error' ) :
        frame = frame[frame_header_length:]
        tlv_type_list = []
        for i in range ( frame_dict.get ( 'num_tlvs' ) ) :
            try:
                tlv_type, tlv_length = struct.unpack ( tlv_header_struct , frame[:tlv_header_length] )
                tlv_type_list.append ( tlv_type )
            except struct.error as e :
                tlv_header_dict = { 'error' : {e} }
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
                    # 3 linie poniżej do wywalenia po nagraniu nowych plików z targetami
                    for target_id in frame_dict['target_index_list'] :
                        if target_id['target_id'] < 253 :
                            print ( f"{target_id} {frame_dict['frame_number']}" )
                case 11 :
                    presence = Presence.Presence ( tlv_length - tlv_header_length , frame[tlv_header_length:][:( tlv_length - tlv_header_length )] )
                    frame_dict.update ( { 'presence' : presence.get_presence_dict () } )
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

