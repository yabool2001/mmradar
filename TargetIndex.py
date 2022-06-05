import struct

class TargetIndex :
    def __init__ ( self , length , v ) :
        self.length = length
        self.v = v
        self.target_index_struct = '1B'
        self.target_index_length = struct.calcsize ( self.target_index_struct )
        self.targets_index_list = []

    def get_targets_index_list ( self ) :
        targets_index_number = int ( self.length / self.target_index_length )
        for i in range ( targets_index_number ) :
            try :
                track_id = struct.unpack ( self.target_index_struct , self.v[( i * self.target_index_length ):][:self.target_index_length] )
                # Zapisz punkt
                if track_id :
                    self.targets_index_list.append ( track_id )
            except struct.error as e :
                self.targets_index_list.append ( f"error: {e}" )
        return self.targets_index_list
