from __future__ import print_function 
import time 
import openadk
from openadk.rest import ApiException
import pprint
from openadk.models.motions_parameter import MotionsParameter
from openadk.models.motions_operation import MotionsOperation

# create an instance of the API
configuration = openadk.Configuration()
configuration.host = 'http://172.26.207.227:9090/v1'
#api_instance = openadk.DevicesApi(openadk.ApiClient(configuration))

api_instance = openadk.MotionsApi(openadk.ApiClient(configuration)) 
timestamp = int(time.time())
name = 'Stop'
motion = MotionsParameter(name=name, direction='both', speed='fast', repeat=5)
body = MotionsOperation(motion=motion, operation='start', timestamp=timestamp)

 try: 
    # Update the motions
    api_response = api_instance.put_motions(body) 
    print(api_response) 
except ApiException as e: 
            print("Exception when calling MotionsApi->put_motions: %s\n" % e)
