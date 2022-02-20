import logging

def write_data_2_local_file ( s , file_name ) :

    with open ( file_name , 'a' , encoding='utf-8' ) as f :
        f.write ( s )
        #logging.info ( f"Data has been written to: {f.name}" )

# Wersja w któej chciałbym raz otworzyć plik i zapisywać do niego do czasu zamknięcia programu:        
#    try:
#        f = open ( file_name , 'a' , encoding='utf-8' )
#    except IOError as e :
#        logging.info ( f"{f.name} file opening problem... {str(e)}" )       
#    if f.writable() :
#        f.write ( f"\n\n{{frame:{self.frame_header_json},{self.tlvs_json}}}" )
#        try:
#            f.close ()
#        except IOError as e :
#            logging.info ( f"{f.name} file closing problem... {str(e)}" )