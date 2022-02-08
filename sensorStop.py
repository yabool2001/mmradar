import time
import serial

################################################################
######################## DEFINITIONS ###########################
################################################################

global                  chirp_cfg , log_file , conf_com
conf_file_name          = 'chirp_cfg/sensorStop.cfg'
log_file_name           = 'log.txt'
conf_com                = serial.Serial ()
#conf_com.port           = 'COM10' # Choose: Silicon Labs Dual CP2105 USB to UART Bridge: Enhanced COM Port from Device manager on MS GO3
conf_com.port           = 'COM4'
#data_com.port           = 'COM11' # Choose: Silicon Labs Dual CP2105 USB to UART Bridge: Standard COM Port from Device manager on MS GO3
# Chose from Device manager: Silicon Labs Dual CP2105 USB to UART Bridge: Standard COM Port 
conf_com.baudrate       = 115200
conf_com.bytesize       = serial.EIGHTBITS
conf_com.parity         = serial.PARITY_NONE
conf_com.stopbits       = serial.STOPBITS_ONE
conf_com.timeout        = 0.3
conf_com.write_timeout  = 1

hello = "\n\n#########################################\n########## sensorStop.py started ##########\n#########################################\n"

# Open log file
try:
    log_file = open ( log_file_name , 'a' , encoding='utf-8' )
    if log_file.writable() :
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {hello}' )
        log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {log_file.name} file is writable.' )
except IOError as e :
    print ( f'{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {log_file.name} file opening problem... {str(e)}' )

# Open Chirp configuration file and read configuration to chirp_cfg
try:
    with open ( f'{conf_file_name}' , 'r' , encoding='utf-8' ) as conf_file:
        if conf_file.readable () :
            log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_file.name} file is readable.' )
        chirp_cfg = conf_file.readlines()
except IOError as e :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_file.name} file opening problem... {str(e)}' )

# Open CONF COM port
try: 
    conf_com.open ()
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port opened.' )
except serial.SerialException as e :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port error opening: {str(e)}' )

# START PROGRAM
print ( hello )

# Configure chirp 
conf_com.reset_input_buffer()
conf_com.reset_output_buffer()
for line in chirp_cfg :
    time.sleep(.1)
    conf_com.write ( line.encode () )
    ack = conf_com.readline ()
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port ack: {ack}' )
    ack = conf_com.readline ()
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {conf_com.name} port ack: {ack}' )
    time.sleep ( 3 )
    conf_com.reset_input_buffer ()

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

# Close log file
try:
    log_file.close ()
    if log_file.closed :
        print ( f'{log_file.name} file is closed.' )
except IOError as e :
    log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {log_file.name} file closing problem... {str(e)}' )
    print ( f'{log_file.name} file closing problem... {str(e)}' )