from azure.iot.device import IoTHubDeviceClient

# Żadne poniższe nowe funkcje nie był testowane.
# Uzupełnić o multiprocessing na podstawie testów z python-sandbox

def open_azure ():
    azure_connection_string = "HostName=mmradariothub.azure-devices.net;DeviceId=iwr6843;SharedAccessKey=k8yx5ft6yrSJ8Xsti3FViAuXWxDRtBMPbI5Hvr1DfI0="
    azure_client = IoTHubDeviceClient.create_from_connection_string ( azure_connection_string )
    azure_client.connect ()
    return azure_client

def send_2_azure_f ( azure_client , **s ):
    azure_connection_string = "HostName=mmradariothub.azure-devices.net;DeviceId=iwr6843;SharedAccessKey=k8yx5ft6yrSJ8Xsti3FViAuXWxDRtBMPbI5Hvr1DfI0="
    azure_client = IoTHubDeviceClient.create_from_connection_string ( azure_connection_string )
    azure_client.connect ()
    try :
        azure_client.send_message ( f'{s}' )
    except :
        print ( "Azure error connecting or sending message")
    azure_client.shutdown()

def close_azure ( azure_client ):
    azure_client.shutdown()