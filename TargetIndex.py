import struct

class TargetIndex :
    def __init__ ( self , length , v ) :
        self.length = length
        self.v = v
        self.target_index_struct = '1B'
        self.target_index_length = struct.calcsize ( self.target_index_struct )
        self.targets_index_list = []

    def get_target_index_list ( self ) :
        targets_index_number = int ( self.length / self.target_index_length )
        for i in range ( targets_index_number ) :
            try :
                target_id = struct.unpack ( self.target_index_struct , self.v[( i * self.target_index_length ):][:self.target_index_length] )
                self.targets_index_list.append ( target_id[0] )
                #if target_id[0] < 253 :
                    #print ( f'target_id = {target_id[0]}' )
            except struct.error as e :
                self.targets_index_list.append ( f"error: {e}" )
        return self.targets_index_list
