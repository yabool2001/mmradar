import azure_iot_hub_mzemlopl as aih
from contextlib import nullcontext
import datetime
import json
from os import error, system
import platform
import time
import serial
import serial.tools.list_ports
import struct
import sys
# sys.setdefaultencoding('utf-8')

################################################################
######################## DEFINITIONS ###########################
################################################################

global                  chirp_cfg
global                  log_file , data_file
global                  conf_com , data_com
raws                    = bytes(1)
log_file_name           = 'log.txt'
data_file_name          = 'data.txt'
hvac_cfg_file_name      = 'chirp_cfg/sense_and_direct_68xx-mzo1.cfg'
pc3d_cfg_file_name      = 'chirp_cfg/ISK_6m_default-mzo-v.1.cfg'
conf_com                = serial.Serial ()
data_com                = serial.Serial ()
serial_ports =  serial.tools.list_ports.comports()
for s_p in serial_ports:
    if 'CP2105'.lower() in s_p.description.lower() and 'Enhanced'.lower() in s_p.description.lower():
        if platform.system() == 'Linux':
            conf_com.port = '/dev/' + s_p.name
        elif platform.system() == 'Windows':
            conf_com.port = s_p.name
        else:
            print ("No compatible  os!")
    if 'CP2105'.lower() in s_p.description.lower() and 'Standard'.lower() in s_p.description.lower():
        if platform.system() == 'Linux':
            data_com.port = '/dev/' + s_p.name
        elif platform.system() == 'Windows':
            data_com.port = s_p.name
        else:
            print ("No compatible  os!")
conf_com.baudrate       = 115200
data_com.baudrate       = 921600*1
conf_com.bytesize       = serial.EIGHTBITS
data_com.bytesize       = serial.EIGHTBITS
conf_com.parity         = serial.PARITY_NONE
data_com.parity         = serial.PARITY_NONE
conf_com.stopbits       = serial.STOPBITS_ONE
data_com.stopbits       = serial.STOPBITS_ONE
conf_com.timeout        = 0.3
data_com.timeout        = 0.025
conf_com.write_timeout  = 1

data_com_delta_seconds = 2

#pprint (conf_com)
#pprint (data_com)

# people_counting_mode: 'hvac' - Sense And Direct Hvac Control; 'pc3d' - 3d People Counting
people_counting_mode = 'pc3d'

hello = "\n\n#########################################\n########## serials3.py started ##########\n#########################################\n"

################################################################
############ OPEN LOG, DATA AND CHIRP CONF FILE ################
################################################################

# Open log file
try:
    log_file = open ( log_file_name , 'a' , encoding='utf-8' )
    if log_file.writable() :
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {hello}' )
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {log_file.name} file is writable.' )
except IOError as e :
    print ( f'{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {log_file.name} file opening problem... {str(e)}' )

# Open data file
try:
    data_file = open ( data_file_name , 'a' , encoding='utf-8' )
    if data_file.writable() :
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {data_file.name} file is writable.' )
except IOError as e :
    print ( f'{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {data_file.name} file opening problem... {str(e)}' )

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
    conf_file_name = pc3d_cfg_file_name
elif people_counting_mode == 'hvac':
    conf_file_name = hvac_cfg_file_name
else:
    print ( f'{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} Error: no chirp cfg file!' )
# do tąd
try:
    with open ( f'{conf_file_name}' , 'r' , encoding='utf-8' ) as conf_file:
        if conf_file.readable () :
            log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_file.name} file is readable.' )
        chirp_cfg = conf_file.readlines()
except IOError as e :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_file.name} file opening problem... {str(e)}' )

################################################################
################ OPEN CONF AND DATA COM PORTS ##################
################################################################

# Open CONF COM port
try: 
    conf_com.open ()
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port opened.' )
except serial.SerialException as e :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port error opening: {str(e)}' )

# Open DATA COM port
try: 
    data_com.open ()
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {data_com.name} port opened' )
except serial.SerialException as e:
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {data_com.name} port error opening: {str(e)}' )

################################################################
##################### CHIRP CONFIGURATION ######################
################################################################
def chirp_conf () :
    for line in chirp_cfg :
        time.sleep(.1)
        conf_com.write ( line.encode () )
        ack = conf_com.readline ()
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port ack: {ack}' )
        ack = conf_com.readline ()
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port ack: {ack}' )
        time.sleep ( 3 )
        conf_com.reset_input_buffer ()

################################################################
####################### AZURE CONNECTION #######################
################################################################
# Version for IoT Hub connection
#open_client = aih.open_azure ()

class PC3D :
    def __init__ ( self , raw_data ) :
        self.raw_data = raw_data
        self.control = 506660481457717506
        self.tlv_type_pointcloud_2d = 6
        self.tlv_type_target_list = 7
        self.tlv_type_target_index = 8
        self.tlv_type_presence_indication = 11
        self.frame_header_struct = 'Q9I2H'
        self.frame_header_length = struct.calcsize ( self.frame_header_struct )
        self.tlv_header_struct = '2I'
        self.tlv_header_length = struct.calcsize ( self.tlv_header_struct )
        self.pointcloud_unit_struct = '4f'
        self.pointcloud_unit_length = struct.calcsize ( self.pointcloud_unit_struct )
        self.point_struct = '2B2h'
        self.point_length = struct.calcsize ( self.point_struct )
        self.target_list_struct = 'I27f'
        self.target_list_length = struct.calcsize ( self.target_list_struct )
        self.presence_indication_struct = '1I'
        self.presence_indication_length = struct.calcsize ( self.presence_indication_struct )
        self.frame_header = None
        self.tlvs = None
        self.num_tlvs = None
        self.frame_header = None
        self.tlv_header = None
        self.pointcloud_unit = None
        self.presence_indication = None
        self.azimuth_unit = None
        self.doppler_unit = None
        self.range_unit = None
        self.snr_unit = None
        self.points = None
        self.tlv_list = []
        self.point_list = []
        #self.frame_header_data = dict ()

    def write_data ( self , file ) :
        file.write ( f"\n\n{{frame:{self.frame_header},{self.tlvs}}}" )
        # Azure part
        #try :
        #    azure_client.send_message ( f"\n\n{{frame:{self.frame_header},{self.tlvs}}}" )
        #except :
        #    print ( "Azure error connecting or sending message")

    # Zdekodowanie wszystkich punktów z ramki zaczynającej się od Punktów
    # Zapisanie punktów do dict, zapisanie słownika do pliku i skasowanie słownika
    # Usunięcie Punktu z ramki w każdej iteracji
    def get_points ( self , tlv_length ) :
        points_number = int ( ( tlv_length - self.tlv_header_length - self.pointcloud_unit_length ) / self.point_length )
        for i in range ( points_number ) :
            try :
                azimuth_point , doppler_point , range_point , snr_point = struct.unpack (self. point_struct , self.raw_data[(self.tlv_header_length + self.pointcloud_unit_length ) + ( i * self.point_length ):][:self.point_length] )
                # Zapisz punkt
                if doppler_point :
                    self.point_list.append ( f"{{'azimuth_point':{azimuth_point},'doppler_point':{doppler_point}, 'range_point':{range_point},'snr_point':{snr_point}}}" )
            except struct.error as e :
                self.point_list.append ( f"{{'error':'{e}'}}" )
        l = len ( self.point_list )
        self.points = f"'num_points':{points_number},'points':["
        for i in range ( len ( self.point_list ) ) :
            self.points += str ( self.point_list[i] ) #self.points = self.points + str ( self.point_list[i] )
            if i < ( l - 1 ) :
                self.points = self.points + ","
        self.points = self.points + "]"
        self.point_list.clear ()


    # Zdekodowanie chmury punktów z ramki zaczynającej się od chmury punktów
    # Zapisanie chmury punktów do dict, zapisanie słownika do pliku i skasowanie słownika
    # Usunięcie chmury punktów z ramki
    def get_pointcloud2d_unit ( self ) :
        try :
            self.azimuth_unit , self.doppler_unit , self.range_unit , self.snr_unit = struct.unpack ( self.pointcloud_unit_struct , self.raw_data[self.tlv_header_length:][:self.pointcloud_unit_length] )
            self.pointcloud_unit = f"'point_cloud_unit':{{'azimuth_unit':{self.azimuth_unit},'doppler_unit':{self.doppler_unit},'range_unit':{self.range_unit},'snr_unit':{self.snr_unit}}}"
            return True
        except struct.error as e :
            self.pointcloud_unit = f"{{'point_cloud_unit':{{'error':'{e}'}}}}"
            return False

    def get_presence_indication (self ) :
        try :
            presence_indication = struct.unpack ( self.presence_indication_struct , self.raw_data[self.tlv_header_length:][:self.presence_indication_length] )
            self.presence_indication = f"{{'presence_indication':{presence_indication[0]}}}"
            print ( f"{presence_indication[0]}" )
            return True
        except struct.error as e :
            self.presence_indication = f"{{'presence_indication':{{'error':'{e}'}}}}"
            return False

    def get_tlv ( self ) :
        try:
            tlv_type, tlv_length = struct.unpack ( self.tlv_header_struct , self.raw_data[:self.tlv_header_length] )
            self.tlv_header = f"'tlv_header':{{'tlv_type':{tlv_type},'tlv_length':{tlv_length}}}"
            if tlv_type == self.tlv_type_pointcloud_2d :
                if self.get_pointcloud2d_unit () :
                    self.get_points ( tlv_length )
                    self.tlv_list.append ( f"{{tlv:{{{self.tlv_header},{self.pointcloud_unit},{self.points}}}}}" )
                else :
                    self.tlv_list.append ( f"{{tlv:{{{self.tlv_header},{self.pointcloud_unit}}}}}" )
            elif tlv_type == self.tlv_type_target_list or tlv_type == self.tlv_type_target_index :
                self.tlv_list.append ( f"{{tlv:{{{self.tlv_header}}}}}" )
            elif tlv_type == self.tlv_type_presence_indication :
                if self.get_presence_indication () :
                    self.tlv_list.append ( f"{{tlv:{{{self.tlv_header},{self.presence_indication}}}}}" )
            self.raw_data = self.raw_data[tlv_length:]
            return True
        except struct.error as e :
            self.tlv_header = f"'tlv_header':{{'error':{e}}}"
            return False

    def get_tlvs ( self ) :
        for i in range ( self.num_tlvs ) :
            if not self.get_tlv () :
                break
        l = len ( self.tlv_list )
        self.tlvs = "'tlvs':["
        for i in range ( l ) :
            self.tlvs = self.tlvs + str ( self.tlv_list[i] )
            if i < ( l - 1 ) :
                self.tlvs = self.tlvs + ","
        self.tlvs = self.tlvs + "]"
        self.tlv_list.clear ()
            
    # Rozpakuj i zapisz dane z Frame header
    def get_frame_header ( self ) :
        try:
            sync , version , total_packet_length , platform , frame_number , subframe_number , chirp_processing_margin , frame_processing_margin , track_process_time , uart_sent_time , num_tlvs , checksum = struct.unpack ( self.frame_header_struct , self.raw_data[:self.frame_header_length] )
            self.num_tlvs = num_tlvs
            if sync == self.control :
                self.frame_header = f"{{'frame_header':{{'sync':{sync},'version':{version},'platform':{platform}, 'total_packet_length':{total_packet_length},'frame_number':{frame_number},'subframe_number':{subframe_number},'chirp_processing_margin':{chirp_processing_margin},'frame_processing_margin':{frame_processing_margin},'uart_sent_time':{uart_sent_time},'track_process_time':{track_process_time},'num_tlvs':{num_tlvs},'checksum':{checksum}}}}}"
            else :
                self.frame_header = f"{{'frame_header':{{'error':'control = {sync}'}}}}"
        except struct.error as e :
            self.frame_header = f"{{'frame_header':{{'error':'{e}'}}}}"

################################################################
####################### START PROGRAM ##########################
################################################################

print ( hello )

# Configure chirp 
conf_com.reset_input_buffer()
conf_com.reset_output_buffer()
chirp_conf ()

# Read data
data_com.reset_output_buffer()
data_com.reset_input_buffer ()
frame_read_time_up = datetime.datetime.utcnow () + datetime.timedelta ( seconds = data_com_delta_seconds )
while datetime.datetime.utcnow () < frame_read_time_up :
    raw_data = data_com.read ( 4666 )
    pc3d_object = PC3D ( raw_data )
    pc3d_object.get_frame_header ()
    if pc3d_object.num_tlvs :
        pc3d_object.raw_data = pc3d_object.raw_data[pc3d_object.frame_header_length:]
        pc3d_object.get_tlvs ()
    pc3d_object.write_data ( data_file )

    del pc3d_object

################################################################
##################### CLOSE DATA COM PORT ######################
################################################################
try:
    data_com.close ()
except serial.SerialException as e :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {data_com.name} port error closing: {str(e)}' )
if not data_com.is_open :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {data_com.name} port closed.' )

################################################################
############# STOP SENSOR AND CLOSE CONF COM PORT ##############
################################################################
# Stop sensor (freez until know how to start it properly)
# conf_com.write ( 'sensorStop\n'.encode () )
# ack = conf_com.readline ()
# log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port ack: {ack}' )
# ack = conf_com.readline ()
# log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port ack: {ack}' )
# time.sleep ( 3 )
# conf_com.reset_input_buffer ()
# Close CONF COM Port
if conf_com.is_open :
    try:
        conf_com.close ()
        if not conf_com.is_open :
            log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port closed.' )
    except serial.SerialException as e:
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port error closing: {str(e)}' )
else:
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port closed.' )

################################################################
################## CLOSE LOG AND DATA FILE #####################
################################################################

# Azure part


# Close data file
try:
    data_file.close ()
    if data_file.closed :
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {data_file.name} file is closed.' )
except IOError as e :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {data_file.name} file closing problem... {str(e)}' )
# Close log file (must be at the end of the program)
log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {log_file.name} file closing...' )
try:
    log_file.close ()
    if log_file.closed :
        print ( f'{log_file.name} file is closed.' )
except IOError as e :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {log_file.name} file closing problem... {str(e)}' )
    print ( f'{log_file.name} file closing problem... {str(e)}' )