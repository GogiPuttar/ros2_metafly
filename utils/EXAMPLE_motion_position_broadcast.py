#3rd party library imports (from Optitrack)
from localization_helper.NatNetClient import NatNetClient

#Native imports
import sys
import time
import numpy as np
import socket  # Import the socket library for networking


#Class used to handle the underlying localization code for the platform.

class Localizer():
    def __init__(self):
        self.reported_id = 0
        self.reported_pos = 0
        self.reported_rot = 0

    def unpack_pos(self, pos):
        self.reported_id = pos[1]
        self.reported_pos = pos[2]
        self.reported_rot = pos[3]

    def begin_process(self):
        
        #initialize options dict which will hold info for the specific connection.
        self.optionsDict = {}
        self.optionsDict["clientAddress"] = "127.0.0.1"#"192.168.18.158" #This is my computer as a client
        #self.optionsDict["clientAddress"] = "192.168.18.142" #this is the raspberry pi as a client.
        self.optionsDict["serverAddress"] = "127.0.0.1"#"192.168.18.189"
        self.optionsDict["use_multicast"] = True
        
        #This will create a new NatNet client
        self.optionsDict = my_parse_args(sys.argv, self.optionsDict)

        #create a new streaming client and initailize its values.
        self.streaming_client = NatNetClient()
        self.streaming_client.set_client_address(self.optionsDict["clientAddress"])
        self.streaming_client.set_server_address(self.optionsDict["serverAddress"])
        self.streaming_client.set_use_multicast(self.optionsDict["use_multicast"])

        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        self.streaming_client.new_frame_listener = self.receive_new_frame
        self.streaming_client.rigid_body_listener = receive_rigid_body_frame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        self.is_running = self.streaming_client.initialize_optitrack_comms()
        if not self.is_running:
            print("ERROR: Could not start streaming client.")
            try:
                self.streaming_client.shutdown()
                sys.exit(1)
            except SystemExit:
                print("...")
            finally:
                self.is_running = False
                print("exiting")

        self.is_looping = True
        time.sleep(1)
        

    #The main function for this class. The one that is the target of a process when a process is intitiated.
    def classifier_behavior(self):

        port = 54321
        ip = "192.168.18.100"

        #set up UDP server.
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_socket.bind((ip,port))
        #allow the socket to send broadcast packets
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        #Define the boradcast address and port 
        broadcast_address = ('<broadcast>', port)

        #begin process by connecting to MOTIVE
        self.begin_process()
        self.streaming_client.set_major(3)
        self.streaming_client.set_minor(0)
        print('process started.')

        #initialize clocks to control when we print data
        time_last_printed = time.time()
        time_between_prints = 0.01 #s
        last_time = time.time()
        last_ten_freq = np.zeros(100)

        avg_pointer = 0

        try:
            i = 0
            while self.is_looping:
                
                #get position of drone(s)
                all_pos = self.streaming_client.get_pos()
                time_stamp = time.time()
                # p2 = self.streaming_client.get_pos()
                # print(pos[2])
                # pos = [True, 1, (1.0, 1.0, 1.0), (1.0, 1.0, 1.0, 1.0)]

                #If there is some data there, unpack it.
                if len(all_pos) > 0:

                    time_new_pos = time.time()
                    if time_new_pos - last_time == 0:
                        # print('DIV 0 WARNING')
                        last_time = time_new_pos        
                    else:
                        freq = 1/(time_new_pos - last_time)

                        last_time = time_new_pos

                        #remove outliers
                        if freq < 500:


                            last_ten_freq[avg_pointer] = freq
                            avg_freq = np.average(last_ten_freq)
                            median_freq = np.median(last_ten_freq)
                            avg_pointer += 1
                            if avg_pointer >= 100:
                                avg_pointer = 0

                    message = ""
                    num_bodies = 0
                    for pos in all_pos:
                        if pos[0]:

                            #if we have more than one body, separate bodies with ', '
                            if num_bodies > 0:
                                message = message + ', '
                            # print(self.streaming_client.get_major())
                            # print(pos)
                            # print(p2)
                            

                        
                            self.unpack_pos(pos)

                            # print(self.reported_id)
                            # print(self.reported_pos)
                            # print(self.reported_rot)

                            #convert quaternion to roll, pitch, and yaw
                            roll, pitch, yaw = quaternion_to_euler(self.reported_rot)

                            
                            msg = str(pos[1]) + ', ' +  str(pos[2][0])[0:] + ', ' +  str(pos[2][1])[0:] + ', ' +  str(pos[2][2])[0:]+ ', ' \
                                +  str(self.reported_rot[0])[0:] + ', ' +  str(self.reported_rot[1])[0:] + ', ' +  str(self.reported_rot[2])[0:] + ', '\
                                    + str(self.reported_rot[3])[0:] + ', ' + str(time_stamp)
                            message += msg

                            num_bodies += 1

                    encoded_message = message.encode('utf-8')
                    # if len(encoded_message) > 1024:
                    #     print('message lenght exceeded maximum!!')
                    #     while True:
                    #         time.sleep(2)

                    # Send the message to the broadcast address
                    server_socket.sendto(encoded_message, broadcast_address)
                    
                    try:
                        if time.time() > time_last_printed + time_between_prints:
                            # print(pos)
                            # print(f"Broadcasting message: {message}")
                            # print(len(encoded_message))
                            # print(num_bodies)
                            print("Loop frequency: %0.2f, %0.2f, %0.2f, %d" %(freq, avg_freq, median_freq, num_bodies))
                            # print(np.max(last_ten_freq))
                            # print(pos[2][0], str(pos[2][0])[0:12])
                            # print(last_ten_freq)
                            # print(roll)
                            time_last_printed = time.time()
                    except:
                        print('error')

                


                if self.streaming_client.connected() is False:
                    print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
                    try:
                        self.streaming_client.shutdown()
                        sys.exit(2)
                    except SystemExit:
                        print("...")
                    finally:
                        self.is_running = False
                        print("exiting")
                        self.is_looping = False
                if not pos[0]:
                    print("ERROR: Position not received. Check that Motive streaming is on.")
                    try:
                        self.streaming_client.shutdown()
                        sys.exit(2)
                    except SystemExit:
                        print("...")
                    finally:
                        self.is_running = False
                        print("exiting")
                        self.is_looping = False

                
        except KeyboardInterrupt:
            print("Server stopped.")
        finally:
            # Close the socket when done
            server_socket.close()
            print('Socket closed.')
            
        


    
    # This is a callback function that gets connected to the NatNet client
    # and called once per mocap frame.
    def receive_new_frame(data_dict):
        order_list=[ "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
                    "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
        dump_args = False
        if dump_args == True:
            out_string = "    "
            for key in data_dict:
                out_string += key + "="
                if key in data_dict :
                    out_string += data_dict[key] + " "
                out_string+="/"
            print(out_string)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame( new_id, position, rotation ):
    pass
    #print( "Received frame for rigid body", new_id )
    #print( "Received frame for rigid body", new_id," ",position," ",rotation )




#from PythonSample.py in optitrack natnet installation file.
def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict


def quaternion_to_euler(q):
    # Extract quaternion components
    w, x, y, z = q

    # Convert quaternion to rotation matrix
    rotation_matrix = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])

    # Extract roll, pitch, and yaw from rotation matrix
    # Roll (x-axis rotation)
    roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

    # Pitch (y-axis rotation)
    sin_pitch = rotation_matrix[2, 0]
    cos_pitch = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
    pitch = np.arctan2(sin_pitch, cos_pitch)

    # Yaw (z-axis rotation)
    sin_yaw = rotation_matrix[1, 0]
    cos_yaw = rotation_matrix[0, 0]
    yaw = np.arctan2(sin_yaw, cos_yaw)

    # # Convert angles from radians to degrees
    # roll = np.degrees(roll)
    # pitch = np.degrees(pitch)
    # yaw = np.degrees(yaw)

    return roll, pitch, yaw


if __name__ == "__main__":

    localizer = Localizer()
    localizer.classifier_behavior()
    

