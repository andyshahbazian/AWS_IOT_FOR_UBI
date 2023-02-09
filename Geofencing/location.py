
"""
Created on Wed Mai 4 07:38:09 2022
gps.py
Copyright (C) 2022 Ralph Grewe
  
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import threading

import serial
from pynmeagps import NMEAReader
import pynmeagps.exceptions as nme

class gps_reader():
    def __init__(self):
        self.gps_running = False
        self.gps_available = False
        self.gps_config = ""
        self.gps_data = {
            "latitude": 0,
            "longitude": 0,
            "altitude": 0,
            "quality": 0,
            "gpsAlive": 0        
        }
        self.lock = threading.Lock()
        self.gui_update_fcn = ''
        self.gps_thread = ''
        self.gps_conntected = False

    def start(self,config):
        self.gps_running = True
        self.gps_config = config['gps']
        self.gps_thread = threading.Thread(target=self.gps_loop)
        self.gps_thread.start()

    def stop(self):
        self.gps_running = False
        if self.gps_thread:
            self.gps_thread.join(5)

    def gps_loop(self):
        try:
            gpsStream = serial.Serial(self.gps_config['interface'], self.gps_config['baudrate'], timeout=3)
        except ( 
            serial.serialutil.SerialException 
        ) as err:
            print(f"Error connecting to GPS {err}")
            return

        nms = NMEAReader(gpsStream)
        print('GPS started')
        self.gps_connected = True
        aliveCounter = 0

        while self.gps_running:
            try:
                (raw_data, parsed_data) = nms.read()
                if parsed_data:
                    if parsed_data.msgID == "GGA":
                        self.lock.acquire()
                        self.gps_data['latitude'] = parsed_data.lat
                        self.gps_data['longitude'] = parsed_data.lon
                        self.gps_data['altitude'] = parsed_data.alt
                        self.gps_data['quality'] = parsed_data.quality
                        aliveCounter += 1
                        self.gps_data['gpsAlive'] = aliveCounter
                        self.gps_available = True
                        self.lock.release()

                        if self.gui_update_fcn:
                            self.gui_update_fcn()
            except (
                nme.NMEAStreamError,
                nme.NMEAMessageError,
                nme.NMEATypeError,
                nme.NMEAParseError,
            ) as err:
                print(f"Something went wrong {err}")
                continue

    def get_data(self):
        self.lock.acquire()
        data = dict(self.gps_data)
        self.lock.release()
        return data
