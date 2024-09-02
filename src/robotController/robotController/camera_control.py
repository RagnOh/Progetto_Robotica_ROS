import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robotController.image_processing import RedRectangleDetector
from robotController.camera_calibration import Camera_Calibration
from std_msgs.msg import Float32MultiArray
import cv2
import time

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        #Ascolto il topic
        self.subscription = self.create_subscription(
            Image,
            '/camera',  # Nome del topic video
            self.listener_callback,
            10)
        self.subscription  # Prevenire che venga eliminata
        self.publisher = self.create_publisher(Float32MultiArray, 'point_coordinates', 10)

        self.bridge = CvBridge()
        self.last_frame_time = time.time()

    def listener_callback(self, msg):
        current_time = time.time()
        # Verifica se è passato un minuto dall'ultima visualizzazione
        if current_time - self.last_frame_time >= 5:
            self.last_frame_time = current_time
            # Convertire il messaggio ROS Image in un'immagine OpenCV
            print("immagine aggiornata")
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Mostrare l'immagine
            processatore=RedRectangleDetector()
            #Identifico il pezzo, il colore ed il centro per il recupero
            punto_camera=processatore.process_image_colori(cv_image)

            calibrazione = Camera_Calibration()
            if (len(punto_camera)!=0):
             #Cerco coordinate centro pezzo per il robot dato il centro nell'immagine catturata da camera   
             punto_real = calibrazione.findRobotCoordinates(punto_camera[0],punto_camera[1])

             coord_msg = Float32MultiArray()
             x_real = round(punto_real[0]/1000,2)
             y_real = round(punto_real[1]/1000,2)

             print(x_real)
             print(y_real)
             coord_msg.data = [x_real, y_real,punto_camera[2]]
             self.publisher.publish(coord_msg)
            

            #cv2.imshow("Frame ogni minuto", cv_image)
            #cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()

    rclpy.spin(video_subscriber)

    # Pulizia e uscita
    video_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
