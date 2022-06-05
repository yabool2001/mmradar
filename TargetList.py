#   Arguments:
#   length - length of V (values) as V = TLV - TL
#   v - bytes of V


import struct

class TargetList :
    def __init__ ( self , length , v ) :
        self.length = length
        self.v = v
        self.target_struct = 'I27f'
        self.target_length = struct.calcsize ( self.target_struct )
        self.target_part1_struct = 'I9f'
        self.target_part1_length = struct.calcsize ( self.target_part1_struct )
        self.target_part2_struct = '16f'
        self.target_part2_length = struct.calcsize ( self.target_part2_struct )
        self.target_part3_struct = '2f'
        self.target_part3_length = struct.calcsize ( self.target_part3_struct )
        self.targets_list = []

    def get_targets_list ( self ) :
        targets_number = int ( self.length / self.target_length )
        for i in range ( targets_number ) :
            try :
                track_id , target_pos_x , target_pos_y , target_pos_z , target_vel_x , target_vel_y , target_vel_z , target_acc_x , target_acc_y , target_acc_z = struct.unpack ( self.target_part1_struct , self.v[( i * self.target_length ):][:self.target_part1_length] )
                # Zostawiam err_covariance[16] na później
                err_covariance = struct.unpack ( self.target_part2_struct , self.v[( i * self.target_length ) + self.target_part1_length:][:self.target_part2_length] )
                gain , confidence_level = struct.unpack ( self.target_part3_struct , self.v[( i * self.target_length ) + self.target_part1_length + self.target_part2_length:][:self.target_part3_length] )
                # Zapisz punkt
                if track_id :
                    #self.targets_list.append ( f"{{'target_id':{target_id},'target_pos_x':{target_pos_x}, 'target_pos_y':{target_pos_y},'target_pos_z':{target_pos_z},'target_vel_x':{target_vel_x}, 'target_vel_y':{target_vel_y},'target_vel_z':{target_vel_z},'target_acc_x':{target_acc_x},'target_acc_y':{target_acc_y},'target_acc_z':{target_acc_z},'err_covariance':{err_covariance},'gain':{gain},'confidence_level':{confidence_level}}}" )
                    self.targets_list.append ( track_id )
            except struct.error as e :
                self.targets_list.append ( f"error: {e}" )
        return self.targets_list
