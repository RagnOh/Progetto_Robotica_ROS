import cv2
import numpy as np

class RedRectangleDetector:
    def __init__(self):
        """
        Inizializza il rilevatore di rettangoli rossi con i parametri di calibrazione della camera.
        
        :param camera_matrix: Matrice della camera ottenuta dalla calibrazione.
        :param dist_coeffs: Coefficienti di distorsione ottenuti dalla calibrazione.
        """
        #self.camera_matrix = camera_matrix
        #self.dist_coeffs = dist_coeffs

    def process_image(self, image):
        """
        Elabora l'immagine per rilevare rettangoli rossi e calcolarne il centro.

        :param image: Immagine OpenCV da elaborare.
        :return: Lista di centri dei rettangoli nel sistema di misura del robot.
        """
        # Convertire l'immagine in spazio colore HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Definire i range per il colore rosso in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        

        # Maschera per i colori rosso
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = mask1 #+ mask2

        # Trovare i contorni nella maschera
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centers_in_robot_frame = []

        for contour in contours:
            # Approssimare il contorno ad un poligono
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Se il poligono ha 4 lati, consideriamolo un rettangolo
            if len(approx) >= 4:
                # Disegna il contorno per visualizzazione (opzionale)
                cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
                #cv2.imshow("Rettangoli Rossi e Centri", image)
                #cv2.waitKey(1)

                # Calcola il centro del rettangolo
                M = cv2.moments(approx)
                if M['m00'] != 0:
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])

                    # Disegna il centro per visualizzazione (opzionale)
                    cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)

                    # Coordinate del centro nel sistema di misura della camera
                    image_points = np.array([[cX, cY]], dtype='float32')
                    image_points = np.expand_dims(image_points, axis=1)

                    print(image_points)
                    #print("ciao")
                    

                    # Undistort the image points using the camera matrix and distortion coefficients
                    #undistorted_points = cv2.undistortPoints(image_points, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)

                    # Aggiungi il centro alle coordinate del sistema robotico
                    #centers_in_robot_frame.append(undistorted_points.flatten())
                    centers_in_robot_frame.append(cX)
                    centers_in_robot_frame.append(cY)

                   # print(centers_in_robot_frame[0])
                   # print(centers_in_robot_frame[1])


        return centers_in_robot_frame

    def display_image(self, image):
        """
        Visualizza l'immagine elaborata con i rettangoli e i centri evidenziati.

        :param image: Immagine OpenCV da visualizzare.
        """
        cv2.imshow("Rettangoli Rossi e Centri", image)
        cv2.waitKey(1)

# Esempio di utilizzo:

# Supponiamo di avere i parametri di calibrazione della camera già disponibili
#camera_matrix = np.array([[fx, 0, cx],
#                          [0, fy, cy],
 #                         [0,  0,  1]])
#dist_coeffs = np.array([k1, k2, p1, p2, k3])

# Inizializza il rilevatore
#detector = RedRectangleDetector(camera_matrix, dist_coeffs)

# Supponiamo che "cv_image" sia l'immagine ricevuta dal topic ROS 2
# centers_in_robot_frame = detector.process_image(cv_image)

# Visualizza l'immagine (opzionale)
# detector.display_image(cv_image)
