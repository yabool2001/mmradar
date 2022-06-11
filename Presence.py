import struct

class Presence :
    def __init__ ( self , length , v ) :
        self.length = length
        self.v = v
        self.presence_struct = '1I'
        self.presence_length = struct.calcsize ( self.presence_struct )

    def get_presence_dict ( self ) :
        try :
            presence = struct.unpack ( self.presence_struct , self.v )
            presence_dict = { 'presence' : presence[0] }
        except struct.error as e :
            presence_dict = { 'presence' : f"error: {e}" }
        return presence_dict
