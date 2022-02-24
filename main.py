# Wdrożyć checksum bo nie wiem skąd się biorą błędy w ramkach tlv

#import azure_iot_hub_mzemlopl as aih
from azure.iot.device import IoTHubDeviceClient
#from contextlib import nullcontext
import datetime
import logging
import multiprocessing
from multiprocessing.dummy import Process
import time
import serial
import serial.tools.list_ports
# sys.setdefaultencoding('utf-8')
from mmradar_ops import mmradar_conf
from serial_ops import open_serial_ports, set_serials_cfg , close_serial_ports , open_serial_ports
from file_ops import write_data_2_local_file
import PC3D


################################################################
######################## DEFINITIONS ###########################
################################################################

p_list                          = []
raws                            = bytes(1)
log_file_name                   = 'mmradar.log'
data_file_name                  = 'mmradar.data'
hvac_cfg_file_name              = 'chirp_cfg/sense_and_direct_68xx-mzo1.cfg'
pc3d_cfg_file_name              = 'chirp_cfg/ISK_6m_default-mzo-v.1.cfg'
mmradar_stop_conf_file_name     = 'chirp_cfg/sensor_stop.cfg'
mmradar_start_conf_file_name    = 'chirp_cfg/sensor_start.cfg'
data_com_delta_seconds          = 0.2

################################################################
###################### LOGGING CONFIG ##########################
################################################################

LOG_FORMAT = '%(asctime)s;%(message)s;%(funcName)s;line:%(lineno)d;%(levelname)s'
logging.basicConfig ( filename = log_file_name , level = logging.INFO , format = LOG_FORMAT )
logging.info (f"\n")
logging.info ( f"########## Hello mmradar app! ##############" )
logging.info ( f"########## 'main.py' has started! ##########" )

################################################################
################ SERIALS COMM CONFiguration ####################
################################################################

conf_com                = serial.Serial ()
data_com                = serial.Serial ()
set_serials_cfg ( conf_com , data_com )

# people_counting_mode: 'hvac' - Sense And Direct Hvac Control; 'pc3d' - 3d People Counting
people_counting_mode = 'pc3d'

hello = "\n\n##########################################\n############# mmradar started ############\n##########################################\n"

################################################################
############ OPEN LOG, DATA AND CHIRP CONF FILE ################
################################################################

# Open Chirp configuration file and read configuration to chirp_cfg

# Jak będę miał na raspberry pi python wersja 3.10 to zastąpić to
#match people_counting_mode:
#    case 'pc3d':
#        conf_file_name = pc3d_cfg_file_name
#    case 'hvac':
#        conf_file_name = hvac_cfg_file_name
#    case _:
#        print ( f'{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} Error: no chirp cfg file!' )
# na to 
if people_counting_mode == 'pc3d':
    chirp_conf_file_name = pc3d_cfg_file_name
elif people_counting_mode == 'hvac':
    chirp_conf_file_name = hvac_cfg_file_name
else:
    logging.info ( f"Error: no chirp cfg file!" )

################ OPEN CONF AND DATA COM PORTS ##################
open_serial_ports ( conf_com , data_com )

####################### AZURE CONNECTION #######################
azure_connection_string = "HostName=mmradariothub.azure-devices.net;DeviceId=iwr6843;SharedAccessKey=k8yx5ft6yrSJ8Xsti3FViAuXWxDRtBMPbI5Hvr1DfI0=" # wersja dla konta mzemlo.pl@gmail.com
#azure_connection_string = "HostName=mmradar.azure-devices.net;DeviceId=iwr6843isk001;SharedAccessKey=ujS/p+N9cUhUw//fhQW6tbomVxcPajR2vkUjfwRqCsY=" # wersja dla konta mzemlo@netemera.comb
azure_client = IoTHubDeviceClient.create_from_connection_string ( azure_connection_string )
azure_client.connect ()
def send_2_azure_iothub ( s ) :
    if azure_client.connected :
        try :
            azure_client.send_message ( f'{s}' )
            logging.info ( "Azure message sent")
        except :
            logging.info ( "Azure error sending message!")

################################################################
####################### START PROGRAM ##########################
################################################################

print ( hello )

##################### CHIRP CONF ################################
conf_com.reset_input_buffer()
conf_com.reset_output_buffer()
#mmradar_conf ( chirp_conf_file_name , conf_com )
#mmradar_conf ( mmradar_start_conf_file_name , conf_com )

##################### READ DATA #################################
data_com.reset_output_buffer()
data_com.reset_input_buffer ()
frame_read_time_up = datetime.datetime.utcnow () + datetime.timedelta ( seconds = data_com_delta_seconds )
while datetime.datetime.utcnow () < frame_read_time_up :
    raw_data = data_com.read ( 4666 )
    tsp = time.process_time_ns ()
    pc3d_object = PC3D.PC3D ( raw_data )
    pc3d_object.get_json_data ()
    if __name__ == '__main__' :
        p = Process ( target = write_data_2_local_file , args = ( pc3d_object.frame_json_2_file , data_file_name , ) , name = f"File-{pc3d_object.frame_header_dict.get('frame_number')}" )
        p.daemon = True
        p.start ()
    if __name__ == '__main__' :
        p = Process ( target = send_2_azure_iothub , args = ( pc3d_object.frame_json_2_azure , ) , name = f"Azure-{pc3d_object.frame_header_dict.get('frame_number')}" )
        p.daemon = True
        p.start ()
    tp = ( time.process_time_ns () - tsp ) / 1000000
    if tp > 1 :
        logging.info ( f"WARNING: Frame {pc3d_object.frame_header_dict.get('frame_number')} process time [ms]: {tp}" )
    del pc3d_object

# Read data
#mmradar_conf ( mmradar_stop_conf_file_name , conf_com )


##################### CLOSE DATA COM PORT ######################
close_serial_ports ( conf_com , data_com )

##################### CLOSE AZURE ##############################
azure_client.shutdown()