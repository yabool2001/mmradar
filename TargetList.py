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

    def get_target_list ( self ) :
        targets_number = int ( self.length / self.target_length )
        for i in range ( targets_number ) :
            try :
                target_id , pos_x , pos_y , pos_z , vel_x , vel_y , vel_z , acc_x , acc_y , acc_z = struct.unpack ( self.target_part1_struct , self.v[( i * self.target_length ):][:self.target_part1_length] )
                # Zostawiam err_covariance[16] na później
                err_covariance = struct.unpack ( self.target_part2_struct , self.v[( i * self.target_length ) + self.target_part1_length:][:self.target_part2_length] )
                gain , confidence_level = struct.unpack ( self.target_part3_struct , self.v[( i * self.target_length ) + self.target_part1_length + self.target_part2_length:][:self.target_part3_length] )
                #self.targets_list.append ( f"{{'target_id':{target_id},'target_pos_x':{target_pos_x}, 'target_pos_y':{target_pos_y},'target_pos_z':{target_pos_z},'target_vel_x':{target_vel_x}, 'target_vel_y':{target_vel_y},'target_vel_z':{target_vel_z},'target_acc_x':{target_acc_x},'target_acc_y':{target_acc_y},'target_acc_z':{target_acc_z},'err_covariance':{err_covariance},'gain':{gain},'confidence_level':{confidence_level}}}" )
                target = { 'target_id' : target_id , 'pos_x' : pos_x , 'pos_y' : pos_y , 'pos_z' : pos_z , 'vel_x' : vel_x , 'vel_y' : vel_y , 'vel_z' : vel_z , 'acc_x' : acc_x , 'acc_y' : acc_y , 'acc_z' :acc_z }
                self.targets_list.append ( target )
            except struct.error as e :
                #self.targets_list.append ( f"error: {e}" )
                self.targets_list.append ( {'error' : e } )
                break
        return self.targets_list
