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

real_data                       = 0
chirp_conf                      = 1
data_com_delta_seconds          = 10
plsa                            = []
plwf                            = []
frame_json_2_file               = ''
frame_json_2_azure              = ''
raws                            = bytes(1)
log_file_name                   = 'mmradar.log'
data_file_name                  = 'mmradar.data'
saved_data_file_name            = 'mmradar_gen_good_2lines.bin_raw_data'
hvac_cfg_file_name              = 'chirp_cfg/sense_and_direct_68xx-mzo1.cfg'
pc3d_cfg_file_name              = 'chirp_cfg/ISK_6m_default-mmwvt-v14.11.0.cfg'
mmradar_stop_conf_file_name     = 'chirp_cfg/sensor_stop.cfg'
mmradar_start_conf_file_name    = 'chirp_cfg/sensor_start.cfg'

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

if real_data :
    conf_com = serial.Serial ()
    data_com = serial.Serial ()
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
#        chirp_conf_file_name = pc3d_cfg_file_name
#    case 'hvac':
#        chirp_conf_file_name = hvac_cfg_file_name
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
if real_data :
    open_serial_ports ( conf_com , data_com )

####################### AZURE CONNECTION #######################
azure_connection_string = "HostName=mmradariothub.azure-devices.net;DeviceId=iwr6843;SharedAccessKey=k8yx5ft6yrSJ8Xsti3FViAuXWxDRtBMPbI5Hvr1DfI0=" # wersja dla konta mzemlo.pl@gmail.com
#azure_connection_string = "HostName=mmradar.azure-devices.net;DeviceId=iwr6843isk001;SharedAccessKey=ujS/p+N9cUhUw//fhQW6tbomVxcPajR2vkUjfwRqCsY=" # wersja dla konta mzemlo@netemera.comb
azure_client = IoTHubDeviceClient.create_from_connection_string ( azure_connection_string )
azure_client.connect ()
def send_2_azure_iothub ( s ) :
    if not azure_client.connected :
        azure_client.connect ()
    else :
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
if real_data :
    conf_com.reset_input_buffer()
    conf_com.reset_output_buffer()
    if chirp_conf :
        if chirp_conf == 1 :
            mmradar_conf ( mmradar_start_conf_file_name , conf_com )
        if chirp_conf == 2 :
            mmradar_conf ( chirp_conf_file_name , conf_com )

##################### READ DATA #################################
if real_data :
    data_com.reset_output_buffer()
    data_com.reset_input_buffer ()
else :
    saved_raw_frames = open ( saved_data_file_name , 'r' ) .readlines ()
    saved_raw_frames_numbers = len(saved_raw_frames)

frame_json_2_azure = ''
frame_json_2_file = ''
frame_read_time_up = datetime.datetime.utcnow () + datetime.timedelta ( seconds = data_com_delta_seconds )
i = 0
while datetime.datetime.utcnow () < frame_read_time_up :
    if real_data :
        frame = data_com.read ( 4666 )
    else :
        if i >= saved_raw_frames_numbers :
            break
        frame = eval ( saved_raw_frames[i] )
        i += 1
    tsp = time.process_time_ns ()
    pc3d_object = PC3D.PC3D ( frame )
    pc3d_object.get_json_data ()
    frame_json_2_file += pc3d_object.frame_json_2_file
    frame_json_2_azure += pc3d_object.frame_json_2_azure
    del pc3d_object
    tp = ( time.process_time_ns () - tsp ) / 1000000
    if tp > 1 :
        logging.info ( f"WARNING: Process time [ms]: {tp}" )
        # logging.info ( f"WARNING: Frame {pc3d_object.frame_header_dict.get('frame_number')} process time [ms]: {tp}" )


if __name__ == '__main__' :
    p = Process ( target = write_data_2_local_file , args = ( frame_json_2_file , data_file_name , ) , name = f"File-{tsp}" )
    p.daemon = True
    p.start ()
    plwf.append ( p )
if __name__ == '__main__' :
    #p = Process ( target = send_2_azure_iothub , args = ( pc3d_object.frame_json_2_azure , ) , name = f"Azure-{i}" )
    plsa.append ( Process ( target = send_2_azure_iothub , args = ( frame_json_2_azure , ) , name = f"{tsp}" ) )
    plsa[-1].daemon = True
    plsa[-1].start ()
    #plsa.append ( p )

print ( f"plsa: {len ( plsa )}" )
plsa_print = ''
for j in plsa :
    if not j.is_alive () :
        #plsa.remove ( j )
        #plsa.remove ( plsa.index ( j ) )
        del plsa[plsa.index ( j )]
    plsa_print += f"{j.name},"
print (f"{plsa_print}")
#for j in plwf :
#    print ( f"plwf c: {len ( plwf )}" )
#    if not j.is_alive () :
#        print (f"plwf {j} is not alive")
#        plwf.remove (j)
#    else :
#        print (f"plwf {j} is alive")

# Read data
#mmradar_conf ( mmradar_stop_conf_file_name , conf_com )


##################### CLOSE DATA COM PORT ######################
if real_data :
    close_serial_ports ( conf_com , data_com )

##################### CLOSE AZURE ##############################
#azure_client.disconnect ()
azure_client.shutdown ()