#from multi_motor_control import *
#from picamera import PiCamera
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import io
#import picamera
import logging
import socketserver
from threading import Condition
from http import server
from lidar import *
import open3d

def main():
    stablePos = [0, 0, 0, 0, 0]

    lidarCam = open3d.camera.PinholeCameraIntrinsic(160, 60, 2.4, 2.4, 79.5, 29.5)

    #camera1 = PiCamera()
    #camera2 = picamera.PiCamera(resolution='640x480', framerate=24)
    #camera2.rotation = 90

    parser = argparse.ArgumentParser(description='Try connecting to any available Tau LiDAR Camera.\nIf no port argument is specified scan all serial ports will be scanned.')
    parser.add_argument('--port', metavar='<serial port>', default=None,
                        help='Specify a serial port instead of trying all available Tau LiDAR Cameras')
    args = parser.parse_args()

    port = args.port

    if port is None:
        lidar = scanPortsAndSetup()
    else:
        print('Attempting to connect to device on port \'%s\''%port)
        lidar = setup(port)
    
    #time.sleep(2)

    while True:
        print("System Ready: ")
        v = input()
        if v == "1": # wave
            print("Saying hi: ")
            #smile()

        elif v == "2": # flourish
            print("Showing range: ")

        elif v == "3": # pick
            print("Pick, Place: ")

        elif v == "4": # document
            print("Scanning: ")

        elif v == "5": # stack
            print("Stacking: ")

        elif v == "6": # CV demo
            print("CV demo: ")
            
        elif v == "7":
            print("tbd: ")
            '''
            elif v == "d1": # show camera output
                camera.capture("/home/pi/Pictures/img.jpg")
                img = mpimg.imread("/home/pi/Pictures/img.jpg")
                imgplot = plt.imshow(img)
                plt.show()

            elif v == "d2": # start camera stream
                output = StreamingOutput()
                camera2.start_recording(output, format='mjpeg')
                address = ('', 8000)
                server = StreamingServer(address, StreamingHandler)
                server.serve_forever()
            
            elif v == "d3": # stop camera stream
                camera2.stop_recording()
            '''
        elif v == "d4": # show lidar output
            if lidar:
                try:
                    run(lidar)
                except KeyboardInterrupt:
                    print("ctrl+c pressed")
                except Exception as e:
                    print(e)
            else:
                print("Lidar not propery setup")
            cv2.destroyAllWindows()

        elif v == "end": # finish
            lidar.close()
            arduino.close()
            break

# ========== RUN MAIN ==========
if __name__ == "__main__":
    main()
    arduino.close()
    print("Connection closed.")

PAGE="""\
<html>
<head>
<title>Pi Cam</title>
</head>
<body>
<center><h1>Pi Cam</h1></center>
<center><img src="stream.mjpg" width="640" height="480"></center>
</body>
</html>
"""

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True