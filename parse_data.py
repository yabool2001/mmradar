# Aplikacja do odczytu danych z sensora i zapisu raw data do pliku i sparsowanych danych do odpowiedniego pliku

# Znaleźć błąd w target index albo zadać pytanie na formu o same 255
# Wdrożyć checksum bo nie wiem skąd się biorą błędy w ramkach tlv

# To decode messages on linux machine use command: sudo tcpdump -vv -A udp dst port 10005

import datetime
from matplotlib import projections
#import multiprocessing
import matplotlib.pyplot as plt
from multiprocessing.dummy import Process
from pprint import pprint
import time
#from numpy import append
import serial
import serial.tools.list_ports
import socket
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
data_source                     = 1 # 0: COM Port, 1: UDP Port, 2: File 
chirp_conf                      = 2 # 0: Do nothing, 1: Start radar, 2: Stop, Config and Start radar
data_com_delta_seconds          = 3600

control                         = 506660481457717506
raws                            = bytes(1)
frame                           = bytes(1)
raw_data_bin_file_name          = f'save_bin_data/mmradar_gen_{time.time_ns()}.bin_raw_data'
#saved_raw_data_file_name       = 'save_bin_data/mmradar_gen_1655368399032378700.bin_raw_data
saved_raw_data_file_name        = 'mmradar_gen-20220612_2.bin_raw_data'
parsed_data_file_name           = f'save_parsed_data/mmradar_gen_{time.time_ns()}.parsed_data'
mmradar_cfg_file_name           = 'chirp_cfg/ISK_6m_default-mmwvt-v14.11.0.cfg'
mmradar_stop_cfg_file_name      = 'chirp_cfg/sensor_stop.cfg'
mmradar_start_cfg_file_name     = 'chirp_cfg/sensor_start.cfg'

#dst_udp_ip                      = '10.0.0.157' # Lipków raspberry pi 3b+
dst_udp_ip                      = '10.0.0.159' # Lipków raspberry pi 02w
#dst_udp_ip                      = '10.0.0.2' # Lipków MW50-SV0
#dst_udp_ip                      = '10.0.0.5' # Lipków GO3
# dst_udp_ip                      = '192.168.1.14' # Meander raspberrypi 3b
#dst_udp_ip                      = '192.168.1.17' # Meander raspberrypi 02w
#dst_udp_ip                      = '192.168.1.30' # Meander MW50-SV0
#src_udp_ip                      = '10.0.0.5' # Lipków GO3
src_udp_ip                      = '10.0.0.2' # Lipków MW50-SV0
#src_udp_ip                      = '192.168.1.30' # Meander MW50-SV0
#src_udp_ip                      = '10.0.0.157' # Lipków raspberry pi 3b
#src_udp_ip                      = '10.0.0.159' # Lipków raspberry pi 02w
ctrl_udp_port                   = 10004
data_udp_port                   = 10005

frames_list = []

frame_header_struct = 'Q9I2H'
frame_header_length = struct.calcsize ( frame_header_struct )

tlv_header_struct = '2I'
tlv_header_length = struct.calcsize ( tlv_header_struct )

hello = "\n\n##########################################\n############# mmradar started ############\n##########################################\n"

################################################################
################ SERIALS COMM CONFiguration ####################
################################################################
if data_source == 0 : # COM Port
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
if data_source == 1 : # UDP Port
    src_udp = socket.socket ( socket.AF_INET , socket.SOCK_DGRAM , socket.IPPROTO_UDP )
    src_udp.bind ( ( src_udp_ip , data_udp_port ) )

################################################################
################ START PROGRAM #################################
################################################################
print ( hello )

################################################################
################ MATPLOTLIB SETUP ##############################
################################################################
plt.ion ()
plt.axis ( [ -3 , 3 , -1 , 10] )
x_list = []
y_list = []
aplot = plt.plot ( x_list , y_list , 'ro' )[0]
plt.draw ()
#plt.style.use ( 'seaborn' )

################ OPEN FILE WITH SAVED RAW DATA #################
if data_source == 2 : # File
    saved_raw_frames = open ( saved_raw_data_file_name , 'r' ) .readlines ()
    saved_raw_frames_number = len ( saved_raw_frames )
    saved_raw_frame_counter = 0
else :
    saved_raw_frame_counter = 1
    saved_raw_frames_number = 2
frame_read_time_up = datetime.datetime.utcnow () + datetime.timedelta ( seconds = data_com_delta_seconds )
while datetime.datetime.utcnow () < frame_read_time_up and saved_raw_frame_counter < saved_raw_frames_number :
    #time.sleep (1)
    if data_source == 0 :
        frame = data_com.read ( 4666 )
    if data_source == 1 :
        frame , address = src_udp.recvfrom ( 4666 )
    if data_source == 2 :
        frame = eval ( saved_raw_frames[saved_raw_frame_counter] )
        saved_raw_frame_counter += 1
    frame_dict = { 'id' : time.time_ns() }
    try:
        sync , version , total_packet_length , platform , frame_number , subframe_number , chirp_processing_margin , frame_processing_margin , track_process_time , uart_sent_time , num_tlvs , checksum = struct.unpack ( frame_header_struct , frame[:frame_header_length] )
        #if frame_number == 8961 :
        #    pass
        if sync == control :
            #frame_dict.update ( { 'frame_number' : frame_number , 'num_tlvs' : num_tlvs , 'sync' : sync , 'version' : version , 'total_packet_length' : total_packet_length , 'platform' : platform , 'subframe_number' : subframe_number , 'chirp_processing_margin' : chirp_processing_margin , 'frame_processing_margin' : frame_processing_margin , 'track_process_time' : track_process_time , 'uart_sent_time' : uart_sent_time , 'checksum' : checksum } )
            frame_dict.update ( { 'frame_number' : frame_number , 'num_tlvs' : num_tlvs } )
        else :
            frame_dict.update ( { 'frame header error' : 'control != {sync}' } )
    except struct.error as e :
        frame_dict.update ( { 'frame header error' : {e} } )
    sync = 0
    if not frame_dict.get ( 'frame header error' ) :
        #if data_source == 0:
        #    with open ( raw_data_bin_file_name , 'a' ) as f :
        #        f.write ( f'{frame}\n' )
        frame = frame[frame_header_length:]
        tlv_type_list = []
        for i in range ( frame_dict.get ( 'num_tlvs' ) ) :
            try:
                tlv_type, tlv_length = struct.unpack ( tlv_header_struct , frame[:tlv_header_length] )
                tlv_type_list.append ( tlv_type )
            except struct.error as e :
                frame_dict.update ( { 'tlv header error' : {e} } )
                break
            match tlv_type :
                case 6 :
                    point_cloud = PointCloud.PointCloud ( tlv_length - tlv_header_length , frame[tlv_header_length:][:(tlv_length - tlv_header_length )] )
                    frame_dict.update ( point_cloud_unit = point_cloud.get_point_unit_dict () )
                    frame_dict.update ( points = point_cloud.get_points_list () )
                    if not point_cloud.point_unit_dict.get ( 'error' ) :
                        ### matplotlib
                        #print ( f'{frame_number}, tlv_type: {tlv_type}, len: {len ( point_cloud.points_list)}' )
                        for p in point_cloud.points_list :
                            if not p.get ( 'error' ) :
                                x_list.append ( round ( p['azimuth'] * point_cloud.point_unit_dict['azimuth_unit'] , 2 ) )
                                y_list.append ( round ( p['range'] * point_cloud.point_unit_dict['range_unit'] , 2 ) )
                        aplot.set_xdata ( x_list )
                        aplot.set_ydata ( y_list )
                        #plt.draw ()
                        plt.pause (0.1)
                        #time.sleep (0.1)
                        x_list = []
                        y_list = []
                        ### matplotlib
                case 7 :
                    target_list = TargetList.TargetList ( tlv_length - tlv_header_length , frame[tlv_header_length:][:( tlv_length - tlv_header_length )] )
                    frame_dict.update ( target_list = target_list.get_target_list () )
                    ### matplotlib
                    #print ( f'{frame_number}, tlv_type: {tlv_type}, len: {len ( target_list.targets_list)}' )
                    #for t in target_list.targets_list :
                    #    if not t.get ( 'error' ) :
                    #        x_list.append ( round ( t['pos_x'] , 2 ) )
                    #        y_list.append ( round ( t['pos_y'] , 2 ) )
                    #aplot.set_xdata ( x_list )
                    #aplot.set_ydata ( y_list )
                    #plt.draw ()
                    #plt.pause (0.1)
                    #time.sleep (0.1)
                    #x_list = []
                    #y_list = []
                    #### matplotlib
                case 8 :
                    target_index_list = TargetIndex.TargetIndex ( tlv_length - tlv_header_length , frame[tlv_header_length:][:( tlv_length - tlv_header_length )] )
                    frame_dict.update ( target_index_list = target_index_list.get_target_index_list () )
                case 11 :
                    presence = Presence.Presence ( tlv_length - tlv_header_length , frame[tlv_header_length:][:( tlv_length - tlv_header_length )] )
                    frame_dict.update ( presence.get_presence_dict () )
            frame = frame[tlv_length:]
            tlv_type = None
            tlv_length = None
        frame_dict.update ( tlv_type_list = tlv_type_list )
        tlv_type_list = None
        ### matplotlib action:
        #if target_list :
        #for tl in target_list :
        #    print ( tl['pos_x'] )
        #ax.scatter ( target_list['pos_x'] , target_list['pos_y'] , target_list['pos_z'] , marker = 'v' , alpha = 0.1 )
        #plt.show ()
    #frames_list.append ( frame_dict )
    #del ( frame_dict )
#with open ( parsed_data_file_name , 'a' , encoding='utf-8' ) as f :
    #f.write ( f'{frames_list}' + '\n\r' )
#pprint ( frames_list )
frames_list.clear ()

if data_source == 0 :
    close_serial_ports ( conf_com , data_com )

if data_source == 1 :
    src_udp.close ()