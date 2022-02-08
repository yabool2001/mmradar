import azure_iot_hub_mzemlopl as aih
from contextlib import nullcontext
import datetime
import json
from os import error, system
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
#conf_com.port           = 'COM10' # Choose: Silicon Labs Dual CP2105 USB to UART Bridge: Enhanced COM Port from Device manager on MS GO3
conf_com.port           = 'COM4'
#data_com.port           = 'COM11' # Choose: Silicon Labs Dual CP2105 USB to UART Bridge: Standard COM Port from Device manager on MS GO3
data_com.port           = 'COM3'
# Chose from Device manager: Silicon Labs Dual CP2105 USB to UART Bridge: Standard COM Port 
conf_com.baudrate       = 115200
data_com.baudrate       = 921600*1
conf_com.bytesize       = serial.EIGHTBITS
data_com.bytesize       = serial.EIGHTBITS
conf_com.parity         = serial.PARITY_NONE
data_com.parity         = serial.PARITY_NONE
conf_com.stopbits       = serial.STOPBITS_ONE
data_com.stopbits       = serial.STOPBITS_ONE


hello = "\n\n#########################################\n########## seel_ports.py started ##########\n#########################################\n"

################################################################
############ OPEN LOG, DATA AND CHIRP CONF FILE ################
################################################################

# Start
print ( hello )
ports =  serial.tools.list_ports.comports()
for p in ports:
    print ( p.description )

# Open log file
try:
    log_file = open ( log_file_name , 'a' , encoding='utf-8' )
    if log_file.writable() :
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {hello}' )
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {log_file.name} file is writable.' )
except IOError as e :
    print ( f'{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {log_file.name} file opening problem... {str(e)}' )

try: 
    conf_com.open ()
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port opened.' )
except serial.SerialException as e :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port error opening: {str(e)}' )

if conf_com.is_open :
    try:
        conf_com.close ()
        if not conf_com.is_open :
            log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port closed.' )
    except serial.SerialException as e:
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port error closing: {str(e)}' )
else:
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port closed.' )

try:
    log_file.close ()
    if log_file.closed :
        print ( f'{log_file.name} file is closed.' )
except IOError as e :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {log_file.name} file closing problem... {str(e)}' )
    print ( f'{log_file.name} file closing problem... {str(e)}' )