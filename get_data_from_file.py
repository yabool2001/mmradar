# Aplikacja do odczytu danych z sensora i zapisu raw data do pliku i sparsowanych danych do odpowiedniego pliku

# Wdrożyć checksum bo nie wiem skąd się biorą błędy w ramkach tlv

import datetime
import struct
# sys.setdefaultencoding('utf-8')
from mmradar_ops import mmradar_conf
from serial_ops import open_serial_ports, set_serials_cfg , close_serial_ports , open_serial_ports
from file_ops import write_data_2_local_file


################################################################
######################## DEFINITIONS ###########################
################################################################

people_counting_mode            = 'pc3d_isk'
control                         = 506660481457717506
chirp_conf                      = 0
data_com_delta_seconds          = 1
raw_data                        = bytes (1)
parsed_data_file_name           = 'mmradar_gen.parsed_data'
hvac_cfg_file_name              = 'chirp_cfg/sense_and_direct_68xx-mzo1.cfg'
pc3d_aop_cfg_file_name          = 'chirp_cfg/AOP_6m_default.cfg'
pc3d_isk_cfg_file_name          = 'chirp_cfg/ISK_6m_default.cfg'
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


hello = "\n\n##########################################\n############# mmradar started ############\n##########################################\n"

################################################################
############ OPEN LOG, DATA AND CHIRP CONF FILE ################
################################################################
match people_counting_mode:
    case 'pc3d_aop':
        chirp_conf_file_name = pc3d_aop_cfg_file_name
    case 'pc3d_isk':
        chirp_conf_file_name = pc3d_isk_cfg_file_name
    case 'hvac':
        chirp_conf_file_name = hvac_cfg_file_name
    case _:
        print ( f'Error: no chirp cfg file!' )


################################################################
################ START PROGRAM #################################
################################################################

print ( hello )

################ OPEN FILE #####################################
raw_frames_file = open ( 'mmradar_gen_good_2lines.bin_raw_data' , 'r' )
raw_frames = raw_frames_file.readlines ()

##################### READ DATA ################################

frame_json_2_azure = ''
frame_json_2_file = ''
for raw_frame in raw_frames :
    print ( raw_frame[:10] )
    raw_frame.strip ("b")
    print ( raw_frame[:10] )
    raw_data = raw_frame.lstrip().removeprefix('b\'').removesuffix('\'').encode()
    print ( raw_data[:10] )
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

##################### CLOSE FILES ######################
raw_frames_file.close ()
