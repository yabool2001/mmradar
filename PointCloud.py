import struct

class PointCloud :
    def __init__ ( self , length , v ) :
        
        self.length = length
        self.v = v
        
        self.point_unit_struct = '5f'
        self.point_unit_length = struct.calcsize ( self.point_unit_struct )
        self.targets_index_list = []
        self.point_unit_dict = dict ()
        
        self.point_struct = '2B3h'
        self.point_length = struct.calcsize ( self.point_struct )
        self.points_list = []

    def get_point_unit_dict ( self ) :
        
        try :
            elevation_unit , azimuth_unit , doppler_unit , range_unit , snr_unit = struct.unpack ( self.point_unit_struct , self.v[:self.point_unit_length] )
            self.point_unit_dict = { 'elevation_unit' : elevation_unit , 'azimuth_unit' : azimuth_unit , 'doppler_unit' : doppler_unit , 'range_unit' : range_unit , 'snr_unit' : snr_unit }
        except struct.error as e :
            self.point_unit_dict = { 'error' : {e} }
        
        return self.point_unit_dict
    
    def get_points_list ( self ) :
        
        points_number = int ( ( self.length - self.point_unit_length ) / self.point_length )
        for i in range ( points_number ) :
            try :
                #elevation , azimuth , doppler , range , snr = struct.unpack ( self.point_struct , self.v[(self.tlv_header_length + self.pointcloud_unit_length ) + ( i * self.point_length ):][:self.point_length] )
                elevation , azimuth , doppler , range , snr = struct.unpack ( self.point_struct , self.v[( self.point_unit_length + self.point_length * i ):self.point_length] )
                point_dict = { 'elevation' : elevation , 'azimuth' : azimuth , 'doppler' : doppler , 'range' : range , 'snr' : snr }
                self.points_list.append ( point_dict )
            except struct.error as e :
                self.points_list.append ( f'error: {e}' )
        return self.points_list