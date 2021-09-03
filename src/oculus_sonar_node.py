#!/usr/bin/env python
"""
Communication node to talk to the Oculus and send the processed information
to a local network port for Dune to listen.

        The Oculus Message Header is made up from:
            - oculusId (uint16): found inside the UDP config message
                        in our case it is 0x4f53 (53 4f in little endian)
            - sourceId (unint16): inside UDP config message (9565 = 5d 25)
            - destiantionId (uint16): UDP config [off 4](65535 = ff ff)
            - messageId (uint16): messageSimpleFire = 21 (15 00)
                                messageSimplePingResult = 35
                                Check data structures pdf for more...
            - version (uint16): 0?
            - payloadSize (uint32): size of incoming payload.
            - spare (uint16): two spare bytes

        The SimpleFireMessage will contain:
            - Oculus Message Header: described above, mostle from UDP config message.
            - masterMode (uint8): either 1 or 2. (01)
            - pingRate (enum): Let's go normal (00)
            - networkSpeed (uint8): set to 255 (ff)
            - gamma (uint8): tunable, must experiment. Set to 150 (96 00)
            - flags (mask): set to b00011001 (19)
            - range (double): the range in m (let's try 3m = 40 40 00 00)
            - gain (double): set to 50 intially (42480000)
            - vos (double): leave to zero so sonar calculates (00000000)
            - salinity (double): start w 35 ppm (420c0000)

Author: Aldo Teran Espinoza <aldot@kth.se>
"""
import socket
import struct
# import rospy
import numpy as np
# from sensor_msgs.msg import Image
# from std_msgs.msg import Float64MultiArray
# from cv_bridge import CvBridge
# from dynamic_reconfigure.server import Server
# from oculus_sonar.cfg import OculusConfig

class OculusSonar:
    """
    Class to handle Sonar shit.
    """
    # Constant ports for Oculus comms
    UDP_PORT = 52102
    TCP_PORT = 52100
    TCP_IP = '169.254.37.93'
    # Constant port for detected edge
    TCP_OUT_PORT = 12345

    def __init__(self):
        """
        Not done yet.
        """
        self.udp_sock = socket.socket(socket.AF_INET,
                                      socket.SOCK_DGRAM)
        self.tcp_sock = socket.socket(socket.AF_INET,
                                      socket.SOCK_STREAM)
        # self.tcp_sock.bind((self.TCP_IP, self.TCP_PORT))
        self.tcp_ip = '169.254.37.93'

        # Dynamic reconfigure server
        self.config = None
        self.config_change = False

        #TODO: catch exception
        self.udp_sock.bind(('', self.UDP_PORT))
        #TODO: self.get_udp_vars().

        self.fire_message = self.build_simplefire_msg(self.config)

        # Sonar image publisher
        self.count = 0

    def recv_udp_msg(self):
        """
        rcv udp msg.
        """
        data, addr = self.udp_sock.recvfrom(1024)
        return data

    def connect_tcp(self):
        """
        If no message is sent to the sonar in more than
        one second, it'll drop the tcp connection.
        """
        self.tcp_sock.connect((self.tcp_ip, self.TCP_PORT))

    def send_tcp_msg(self, msg):
        """
        Send tcp message to the connected port.
        """
        self.tcp_sock.sendto(msg, (self.tcp_ip, self.TCP_PORT))

    def recv_tcp_msg(self):
        """
        Duh.
        """
        data = self.tcp_sock.recv(1500)
        return data

    def build_simplefire_msg(self, config,
                             masterMode=1, pingRate=0,
                             gamma=150, gain=0, range_m=1,
                             vos=0, salinity=0):
        """
        Fetch the first part from the UDP config message and fill
        in the rest.
        """
        # If dynamic reconfig not running
        if not self.config_change:
            # Constant header for the SimpleFireRequest
            fire_message = '534f0000000015000000000000000000'
            fire_message = fire_message.decode("hex")
            fire_message += struct.pack('B', masterMode)
            fire_message += struct.pack('B', pingRate)
            networkSpeed = 'ff'
            fire_message += networkSpeed.decode("hex")
            fire_message += struct.pack('B', gamma)
            flags = '19'
            fire_message += flags.decode("hex")
            fire_message += struct.pack('d', range_m)
            fire_message += struct.pack('d', gain)
            fire_message += struct.pack('d', vos)
            fire_message += struct.pack('d', salinity)
        else:
        # Constant header for SimpleFireRequest
            fire_message = '534f0000000015000000000000000000'
            fire_message = fire_message.decode("hex")
            fire_message += struct.pack('B', config['masterMode'])
            fire_message += struct.pack('B', config['pingRate'])
            networkSpeed = 'ff' # Keep constant
            fire_message += networkSpeed.decode("hex")
            fire_message += struct.pack('B', config['gamma'])
            flags = '19' # Keep constant
            fire_message += flags.decode("hex")
            fire_message += struct.pack('d', config['range_m'])
            fire_message += struct.pack('d', config['gain'])
            fire_message += struct.pack('d', config['vOfSound'])
            fire_message += struct.pack('d', config['salinity'])
        # pdb.set_trace()

        self.config_change = False

        return fire_message

    def dynamic_cb(self, config, level):
        """
        Callback for the dynamic reconfigure server.
        """
        self.config = config
        self.config_change = True
        return config

    def keep_alive(self):
        """
        Threaded function to keep alive the TCP socket.
        """
        self.send_tcp_msg('00')

    def process_n_publish(self, data):
        """
        Process the recieved sonar message and
        publish the sonar image on a ROS Image
        message.
        """
        self.count += 1
        # Get no.bearings and no.ranges
        dim = struct.unpack('HH', data[106:110])
        # Starting offset for sonar image
        img_offset = struct.unpack('I', data[110:114])[0]
        # Build greyscale image from echo intensity data
        img = np.fromstring(data[img_offset:], dtype='uint8')
        try:
            img = img.reshape(dim)
        except:
            # rospy.logwarn("Message dims {0} don't match ping result info {1}. Dropping frame.".format(dim, len(img)))
            return

        simple_ping = (list(struct.unpack('<ddddIbdHHIII', data[61:122])))

def main():
    """
    Main method for the ROS node.
    """
    sonar = OculusSonar()
    fire_message = sonar.build_simplefire_msg(sonar.config)
    sonar.connect_tcp()

    # Send message to boot sonar
    sonar.send_tcp_msg(fire_message)
    # rospy.loginfo(sonar.recv_tcp_msg())

    while True:

        # Change fire message if dynamic reconfig triggered
        if sonar.config_change:
            fire_message = sonar.build_simplefire_msg(sonar.config)

        # Send fire message
        sonar.send_tcp_msg(fire_message)
        data = sonar.recv_tcp_msg()

        # Disregard if dummy message is sent
        if len(data) < 100:
            continue

        # Fill buffer until message length
        msg_len = struct.unpack('I', data[10:14])[0]
        while len(data) < msg_len:
            data += sonar.recv_tcp_msg()

        # Process, publish, clean, repeat.
        sonar.process_n_publish(data)
        data = ''


if __name__ == "__main__":
    main()

