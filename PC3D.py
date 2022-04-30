import datetime
import logging
import random
import struct


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
        self.frame_header_dict = dict ()
        self.frame_header_json = None
        self.tlv_header_struct = '2I'
        self.tlv_header_length = struct.calcsize ( self.tlv_header_struct )
        self.tlv_header_dict = dict ()
        self.tlv_header_json = None
        self.tlv_list = []
        self.tlvs_json = None
        self.pointcloud_unit_struct = '4f'
        self.pointcloud_unit_length = struct.calcsize ( self.pointcloud_unit_struct )
        self.point_struct = '2B2h'
        self.point_length = struct.calcsize ( self.point_struct )
        self.point_list = []
        self.points_json = None
        self.pointcloud_unit_dict = dict ()
        self.pointcloud_unit_json = None
        self.target_list_struct = 'I27f'
        self.target_list_length = struct.calcsize ( self.target_list_struct )
        self.presence_indication_struct = '1I'
        self.presence_indication_length = struct.calcsize ( self.presence_indication_struct )
        self.presence_indication_value = None
        self.presence_indication_json = None
        self.frame_json_2_azure = None
        self.frame_json_2_file = None

    def send_data_2_azure ( self ) :
        pass
        # Azure part
        #try :
        #    azure_client.send_message ( f"\n\n{{frame:{self.frame_header_json},{self.tlvs_json}}}" )
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
        self.points_json = f"'num_points':{points_number},'points':["
        for i in range ( len ( self.point_list ) ) :
            self.points_json += str ( self.point_list[i] ) #self.points_json = self.points_json + str ( self.point_list[i] )
            if i < ( l - 1 ) :
                self.points_json = self.points_json + ","
        self.points_json = self.points_json + "]"
        self.point_list.clear ()

    def get_pointcloud2d_unit ( self ) :
        try :
            azimuth_unit , doppler_unit , range_unit , snr_unit = struct.unpack ( self.pointcloud_unit_struct , self.raw_data[self.tlv_header_length:][:self.pointcloud_unit_length] )
            self.pointcloud_unit_dict = { 'azimuth_unit' : azimuth_unit , 'doppler_unit' : doppler_unit , 'range_unit' : range_unit , 'snr_unit' : snr_unit }
        except struct.error as e :
            self.pointcloud_unit_dict = { 'error' : {e} }
        self.pointcloud_unit_json = f"'point_cloud_unit':{self.pointcloud_unit_dict}"
        if self.pointcloud_unit_dict.get ( 'error' ) :
            return False
        else:
            return True


    def get_presence_indication ( self ) :
        try :
            presence_indication = struct.unpack ( self.presence_indication_struct , self.raw_data[self.tlv_header_length:][:self.presence_indication_length] )
            self.presence_indication_value = presence_indication[0] # Dlacze otrzymuję to jako tuple, a nie int 32bit
            self.presence_indication_json = f"'presence_indication':{self.presence_indication_value}"
            #print ( type(presence_indication) )
            #print ( type(self.presence_indication_value) )
            #print ( f"{self.presence_indication_value}" )
            return True
        except struct.error as e :
            self.presence_indication_json = f"{{'presence_indication':{{'error':'{e}'}}}}"
            return False

    def get_tlv_header ( self ) :
        try:
            tlv_type, tlv_length = struct.unpack ( self.tlv_header_struct , self.raw_data[:self.tlv_header_length] )
            self.tlv_header_dict = { 'tlv_type' : tlv_type , 'tlv_length' : tlv_length }
        except struct.error as e :
            self.tlv_header_dict = { 'error' : {e} }
            logging.info ( f"TLV Header unpack error during frame unpack number: {self.frame_header_dict['frame_number']}" )
        self.tlv_header_json = f"'tlv_header':{self.tlv_header_dict}"
        if self.tlv_header_dict.get ( 'error' ) :
            return False
        else:
            return True

    def get_tlv ( self ) :
        if self.get_tlv_header () :
            if self.tlv_header_dict.get ( 'tlv_type' ) == self.tlv_type_pointcloud_2d :
                    if self.get_pointcloud2d_unit () : 
                        self.get_points ( self.tlv_header_dict['tlv_length'] )
                        self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json},{self.pointcloud_unit_json},{self.points_json}}}}}" )
                    else : # tutaj coś jest nie tak, bo jesli jest False to czemu wstawiam to unit? jeśli chodzi o point to trzeba to poprawić
                        self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json},{self.pointcloud_unit_json}}}}}" )
            elif self.tlv_header_dict.get ( 'tlv_type' ) == self.tlv_type_target_list or self.tlv_header_dict.get ( 'tlv_type' ) == self.tlv_type_target_index :
                self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json}}}}}" )
            elif self.tlv_header_dict.get ( 'tlv_type' ) == self.tlv_type_presence_indication :
                if self.get_presence_indication () :
                    self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json},{self.presence_indication_json}}}}}" )
            else :
                self.raw_data = self.raw_data[self.tlv_header_dict['tlv_length']:]
            return True
        else :
            self.tlv_list.append ( f"{{tlv:{{{self.tlv_header_json}}}}}" )
            return False

    def get_tlvs ( self ) :
        for i in range ( self.frame_header_dict.get ( 'num_tlvs' ) ) : # get jest bezpieczne, bo w tym miejscu nie jest jeszcze pewny czy self.frame_header_dict['num_tlvs'] istnieje i mogłoby powodować błąd w programie
            if not self.get_tlv () :
                logging.info ( f"Break 1 in function: with frame number: {self.frame_header_dict['frame_number']}" )
                break
        l = len ( self.tlv_list )
        self.tlvs_json = "'tlvs':["
        for i in range ( l ) :
            self.tlvs_json = self.tlvs_json + str ( self.tlv_list[i] )
            if i < ( l - 1 ) :
                self.tlvs_json = self.tlvs_json + ","
        self.tlvs_json = self.tlvs_json + "]"
        self.tlv_list.clear ()
            
    # Rozpakuj i zapisz dane z Frame header
    def get_frame_header ( self ) :
        try:
            sync , version , total_packet_length , platform , frame_number , subframe_number , chirp_processing_margin , frame_processing_margin , track_process_time , uart_sent_time , num_tlvs , checksum = struct.unpack ( self.frame_header_struct , self.raw_data[:self.frame_header_length] )
            if sync == self.control :
                self.frame_header_dict = { 'frame_number' : frame_number , 'num_tlvs' : num_tlvs , 'sync' : sync , 'version' : version , 'total_packet_length' : total_packet_length , 'platform' : platform , 'subframe_number' : subframe_number , 'chirp_processing_margin' : chirp_processing_margin , 'frame_processing_margin' : frame_processing_margin , 'track_process_time' : track_process_time , 'uart_sent_time' : uart_sent_time , 'checksum' : checksum }
            else :
                self.frame_header_dict = { 'error' : 'control != {sync}' }
        except struct.error as e :
            self.frame_header_dict = { 'error' : {e} }
        self.frame_header_json = f"{{'frame_header':{self.frame_header_dict}}}"
        if self.frame_header_dict.get ( 'error' ) :
            return False
        else:
            return True

    def get_json_data ( self ) :
        if self.get_frame_header () :
            self.raw_data = self.raw_data[self.frame_header_length:]
            self.get_tlvs ()
        self.frame_json_2_file = f"\n\n{{frame:{self.frame_header_json},{self.tlvs_json}}}"
        self.frame_json_2_azure = f"{{'id' : {datetime.datetime.utcnow().strftime('%Y%m%d%H%M%S%f')[:17]},'frame_number':{self.frame_header_dict.get('frame_number')},'presence':{self.presence_indication_value}}}"
        #self.frame_json_2_azure = f"{{'frame':{{'frame_number':{self.frame_header_dict.get('frame_number')},'presence':{random.randint ( 0, 12 )}}}"