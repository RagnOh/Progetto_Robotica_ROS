import cv2
import numpy as np

class RedRectangleDetector:

    LATO_REAL_PEZZO = 0.02 #Lunghezza reale lato cubo in metri
    H_REAL_ROBOT = 0.2 #Altezza in cui robot acquisirà foto
    FOCAL = 1.39 #Lunghezza focale camera

    

    #Funzione per identificare i cubi, i loro centri e altezza
    def process_image_colori(self,image):
        
    
     # Convertire l'immagine in spazio colore HSV
     hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

     # Definire i range per il colore rosso in HSV
     lower_red1 = np.array([0, 100, 100])
     upper_red1 = np.array([10, 255, 255])
     lower_red2 = np.array([160, 100, 100])
     upper_red2 = np.array([179, 255, 255])

     # Definire i range per il colore blu in HSV
     lower_blue = np.array([100, 150, 100])
     upper_blue = np.array([140, 255, 255])

     # Maschera per i colori rosso
     mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
     mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
     mask_red = mask_red1 + mask_red2

     # Maschera per il colore blu
     mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

     # Trovare i contorni nella maschera rossa
     contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

     # Trovare i contorni nella maschera blu
     contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

     centers_in_robot_frame = []

     # Verifica se sono stati rilevati contorni rossi
     if contours_red:
        self.colore = 1.0
        contours_to_process = contours_red
     elif contours_blue:
        self.colore = 2.0
        contours_to_process = contours_blue
     else:
        # Nessun poligono rilevato
        self.colore = 0
        return centers_in_robot_frame

     for contour in contours_to_process:
        # Approssimare il contorno ad un poligono
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        
        if len(approx) >= 4:
            # Disegna il contorno per visualizzazione 
            cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)

            # Calcola il centro del rettangolo
            M = cv2.moments(approx)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])

                # Disegna il centro per visualizzazione 
                cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)

                # Coordinate del centro nel sistema di misura della camera
                image_points = np.array([[cX, cY]], dtype='float32')
                image_points = np.expand_dims(image_points, axis=1)

                print(image_points)

                #Calibrazione calcolo altezza
                points = np.array(approx).reshape(-1, 2)

                # Lista per tenere traccia delle distanze
                distances = []

                # Calcola la distanza tra ogni coppia di punti consecutivi (lati del poligono)
                for i in range(len(points)):
                  pt1 = points[i]
                  pt2 = points[(i + 1) % len(points)]  # Cicla al primo punto dopo l'ultimo
                  distance = cv2.norm(pt1 - pt2)
                  distances.append(distance)

                # Trova la distanza minima
                min_distance = min(distances)
                print("Distanza minima tra i lati:", min_distance)
                print(self.calcola_altezza_reale(min_distance))

                # Aggiungo il centro alle coordinate 
                centers_in_robot_frame.append(cX)
                centers_in_robot_frame.append(cY)
                #Aggiungo 1 se cubo è rosso, 2 se blu
                centers_in_robot_frame.append(self.colore)

     return centers_in_robot_frame
    
    #AutoCalcolo altezza
    def calcola_altezza_reale(self,delta):
        
        h_real = ((delta/1000) * RedRectangleDetector.H_REAL_ROBOT * RedRectangleDetector.H_REAL_ROBOT) / (2*RedRectangleDetector.LATO_REAL_PEZZO * RedRectangleDetector.FOCAL)

        return h_real

    def display_image(self, image):
        
        cv2.imshow("Rettangoli Rossi/Blu e Centri", image)
        cv2.waitKey(1)


