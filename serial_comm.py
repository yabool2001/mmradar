import platform
import serial.tools.list_ports
import time

def set_serial_cfg ( conf_com , data_com ) :
    serial_ports =  serial.tools.list_ports.comports()
    for s_p in serial_ports:
        if 'CP2105'.lower () in s_p.description.lower () and 'Enhanced'.lower () in s_p.description.lower ():
            if platform.system () == "Windows":
                conf_com.port = s_p.name
            elif platform.system () == "Linux":
                conf_com.port = '/dev/' + s_p.name
            else:
                print ('Error: No compatible os!')
        if 'CP2105'.lower () in s_p.description.lower () and 'Standard'.lower () in s_p.description.lower ():
            if platform.system () == "Windows":
                data_com.port = s_p.name
            elif platform.system () == "Linux":
                data_com.port = '/dev/' + s_p.name
            else:
                print ('Error: No compatible os!')
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

def close_serial_comm ( sp , log_file ) :
    if sp.is_open :
        try:
            sp.close ()
        except serial.SerialException as e :
            log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {sp.name} port closing error: {str(e)}' )
        if not sp.is_open :
            log_file.write ( f'\n{time.gmtime ().tm_hour}:{time.gmtime ().tm_min}:{time.gmtime ().tm_sec} {sp.name} port closed.' )