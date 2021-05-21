#! /usr/bin/env python
import sys
import os
import cv2
import time
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16
import rospy

class streetDetection:
    #esta medio jodido el ident por atom, quien sabe por que pero si revisas en nano esta bien en el ident, ya despues revisare por que
    def __init__(self,imagen):
        #print("streetDetection")
        self.height, self.width, self.channels= imagen.shape
        #Se establecen los intervalos para deteccion de los colores
        #rojo
        self.rojoBajo1 = np.array([0, 100, 20])
        self.rojoAlto1 = np.array([5, 255, 255])
	self.rojoBajo2 = np.array([175, 100, 20])
        self.rojoAlto2 = np.array([179, 255, 255])
        #Blanco
        self.blancoBajo = np.array([0,0,230])
        self.blancoAlto = np.array([180,255,255])
        #Cordenadas:
        self.refx = 0
	self.refy = 0
	self.refNuevaX = 0
	self.refNuevaY = 0
    	self.azulBajo = np.array([105, 100, 20])
	self.azulAlto = np.array([125, 255, 255])
    	self.bandera=0

    #De aqui en adelante todos los metodos giran en torno a procesar la imagen para reconocer la calle
    def calculaCentroide(self,m):
        try:
		cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
	except ZeroDivisionError:
		cx, cy = self.width/2, self.height/2
        return cx,cy

    #esta fucked up el ident por atom pero en nano jala bien
    def identificaColores(self,imagen):
        hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.rojoBajo1, self.rojoAlto1)
	mask2 = cv2.inRange(hsv, self.rojoBajo2, self.rojoAlto2)
	maskRojo = cv2.add(mask1, mask2)
	maskBlanco = cv2.inRange(hsv, self.blancoBajo, self.blancoAlto)
    	maskAzul = cv2.inRange(hsv, self.azulBajo, self.azulAlto)
	return mask1,mask2,maskRojo,maskBlanco,maskAzul

    def todosLosCentroidesDamelos(self,cntBlanco,resblanco):
        centros=[]
        for i in range(len(cntBlanco)):
            moments=cv2.moments(cntBlanco[i])
            try:
                centros.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
                cv2.circle(resblanco,centros[-1],10,(0,255,0),-1)
            except ZeroDivisionError:
                pass
        return centros

    def imprimeUnMonton(self,string):
        for i in range(10):
            print(string)

    def recorteBlanco(self,imagen_cv):
            imagenCrop = imagen_cv[0:480,0:350]
            hsv = cv2.cvtColor(imagenCrop, cv2.COLOR_BGR2HSV)
            blancoBajo = np.array([0,0,230])
            blancoAlto = np.array([180,255,255])
            maskBlanco = cv2.inRange(hsv, blancoBajo, blancoAlto)
            resblanco = cv2.bitwise_and(imagenCrop, imagenCrop, mask = maskBlanco)
            return maskBlanco,resblanco

    def publisher(self,instruccion):
            #print("Deberia de estar publicando tecnicamente")
	    topicoToPublish="/banderaEstacionamiento"
	    publishervel = rospy.Publisher(topicoToPublish,Int16,queue_size=1)
	    publishervel.publish(instruccion)



    def reconoceCalle(self,imagen,bandera):
        global switchazul
        switchazul=bandera
        print("Este es el valor de la bandera SANTIAGO:",switchazul)
        #codigo cualquiera que identifique cuadrados izquierda y linea blanca derecha
        mask1,mask2,maskRojo,maskBlanco,maskAzul = self.identificaColores(imagen)
        maskBlanco,resblanco= self.recorteBlanco(imagen)
        #resblanco = cv2.bitwise_and(imagen, imagen, mask = maskB2)
        resrojo = cv2.bitwise_and(imagen, imagen, mask = maskRojo)
        resazul = cv2.bitwise_and(imagen, imagen, mask = maskAzul)
        maskAzul = cv2.medianBlur(maskAzul, 9)
        
        m= cv2.moments(maskBlanco,False)
        #esto podria ayudarnos
        cx,cy = self.calculaCentroide(m)
        _, cntBlanco, _ = cv2.findContours(maskBlanco, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _, cntRojo, _ = cv2.findContours(maskRojo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _, cntAzul, _ = cv2.findContours(maskAzul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        stopSigns=self.detectorAltos(cntRojo)

        cv2.drawContours(resazul, cntAzul, -1, (0,255,0), 2)
        estacionacion=self.detectorEspacioDeEstacionamiento(cntAzul)
        #intersecciones =self.intersecciones(cntBlanco)
        # inter=self.deteccionDeIntersecciones(cntBlanco)
        # if inter == 1:
        #     objetivo="inter"
        #     return objetivo
        
        #if stopSigns == 1:
        #   objetivo = "stop"
        #    return objetivo
        elif estacionacion==1:
        	return "estacionate"
        else:
            centros=self.todosLosCentroidesDamelos(cntBlanco,resblanco)
            cv2.circle(resblanco,(int(cx),int(cy)),10,(0,0,255),-1)
            intersections=self.deteccionDeIntersecciones(centros,cntBlanco)
            if intersections==1:
                objetivo= "inter"
                return objetivo
            cv2.imshow("Res Blanco", resblanco)
            cv2.imshow("Res Rojo", resrojo)
            cv2.imshow("Res Azul", resazul)
            cv2.waitKey(1)
            objetivo = self.linesIdentifying(cx,cy)
            return objetivo
    ########################################################################################################


    def detectorAltos(self,cntRojo):
        #Ciclo para las distintas figuras de color rojo que hayan sido detectadas
	for c in cntRojo:
			#epsilon determina la precision de aproximacion para la deteccion de una figura
			#arcLenght calcula la longitud de curva del contorno que fue detectado, el True indica una curva cerrada
		epsilon = 0.01*cv2.arcLength(c, True)
			#approxPolyDP aproxima una forma de contorno dependiendo de la precision que se le ponga
		approx = cv2.approxPolyDP(c, epsilon, True)
			#Se calcula el area de la figura que fue detectada
		area = cv2.contourArea(c)
			#Se pregunta si el area es mayor a determinado numero, es decir que ocupa cierto espacio en la pantalla
        	
		if area > 6000 and area <13000:
	#Solo si la figura detectada tiene 8 vertices dispara la senialal de alto
		    
            	    return 1
	return 0

#Usamos esta funcion principalmente para debugging de las conecciones entre ros y opencv
    def videoToImageFile(self,imagen):
        os.chdir(os.path.expanduser("~"))
        cv2.imwrite("imagenPrueba1.jpg",imagen)
        self.mostrarEnViVo(imagen)
        return imagen

    def identificarMascaras(self,imagen):
        return 0


    def mostrarEnViVo(self,imagen):
        cv2.imshow('ImagenEnVivo',imagen)
        cv2.waitKey(1)


    def inicializamosCentro(self,cx,cy):
        if self.refx == 0 and self.refy == 0:
		self.refx = cx
		self.refy = cy
		#Se reescala el rango de valores de las coordenadas a estar entre -1 y 1
		self.refNuevaY = np.interp(self.refy,[1, 480],[-1,1])

    def linesIdentifying(self,cx,cy):
        self.inicializamosCentro(cx,cy)
        cordNuevaX = np.interp(cx, [1,350],[-1,1])
	cordNuevaY = np.interp(cx, [1,480],[-1,1])
	#print("CordNuevaX: {}, CordNuevaY: {}".format(cordNuevaX,cordNuevaY))
		#Se compara si el centroide actual esta mas a la derecha o a la izquierda que el primero que se genero
	if cordNuevaX < self.refNuevaX:
		#print("Debes girar a la derecha")
                return "a"
	elif cordNuevaX > self.refNuevaX:
		#print("Debes girar a la izquierda")
                return "d"

    def deteccionDeIntersecciones(self,centres,cntBlanco):
        #Se haran revisiones por intervalos en la imagen, en este caso de 64 pixeles de alto cada uno
		bajo = 1
		alto = 64
		#Puesto que la imagen tiene altura total de 640 se haran 10 intervalos
		for i in range(10):
			#Se establece un contador de centroides detectados
			contCentroides = 0
			#Se recorre el arreglo donde se guardaron todos los centroides verdes
			for j in range(len(centres)):
				#Si se detecta un centroide dentro del intervalo, se agrega una unidad al contador
				if centres[j][0] > bajo and centres[j][0] < alto:
					contCentroides += 1
			#Si se detecta 5 centroides o mas en el mismo intervalo entonces hay una interseccion en el mapa
			if contCentroides >= 6:
				#print("Detectado interseccion")
				areaBlanco=0
                    		for c in cntBlanco:
                        		areaBlanco = areaBlanco + cv2.contourArea(c)
                                if areaBlanco > 20000:
                            	        return 1
                            	else: 
                        		
                        		return 0
			#Si no es el caso, se analizara otro intervalo de pixeles en la imagen
			else:
				bajo += 64
				alto += 64
        	return 0 

    def detectorEspacioDeEstacionamiento(self,cntAzul):
        global switchazul
        switch = globals()['switchazul']
        numObjetos = 0
	areaTotal = 0
        for c in cntAzul:
            area= cv2.contourArea(c)
            if area > 1000:
                #print("Imprimir por que encontre un puto carro")
                numObjetos+=1
                areaTotal = areaTotal + area
        #print("Bandera:",switchazul)
        if numObjetos == 2:
		#print("ENTRAMOS EN FASE 2 QUIEN SOS")
	    self.publisher(1)
            self.estacionarse(0)
		#print("banderaDentroDelCiclo:",switchazul)

	if numObjetos == 1 and areaTotal > 10000 and switch == 1:
           	print("ENTRAMOS A FASE 3 QUIEN SOS")
		return 1        

        return 0


    def publisher2(self,giro,velocidad):
        topicoToPublish="/AutoModelMini/manual_control/speed"
        topicoToPublish2="/AutoModelMini/manual_control/steering"
        publishervel = rospy.Publisher(topicoToPublish,Int16,queue_size=1)
        publishergiro = rospy.Publisher(topicoToPublish2,Int16,queue_size=1)
        publishervel.publish(velocidad)
        publishergiro.publish(giro)
        print("Publicado")


    def estacionarse(self,cont):
        if cont ==0:
            vel=30
            giro=90
            self.publisher2(giro,vel)
            print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
            cont=cont+1
            time.sleep(0.11)
        if cont ==1:
            vel=30
            giro=90
            self.publisher2(giro,vel)
            print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
            cont=cont+1
            time.sleep(1)
        if cont==2:
            vel=-300
            giro=82
            self.publisher2(giro,vel)
            print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
            cont=cont+1
            time.sleep(12)
        if cont==3:
            vel=300
            giro=-30
            self.publisher2(giro,vel)
            print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
            cont=cont+1
            time.sleep(4)
        if cont==4:
            vel=220
            giro=220
            self.publisher2(giro,vel)
            print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
            time.sleep(3.7)
            cont=cont+1
        # if cont==5:
        #     vel=-130
        #     giro=-30
        #     self.publisher2(giro,vel)
        #     print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
        #     time.sleep(1.5)
        #     cont=cont+1
        # if cont==6:
        #     vel=170
        #     giro=200
        #     self.publisher2(giro,vel)
        #     print("cicloterminado","Vel:",vel,"Giro",giro,"Contador",cont)
        #     time.sleep(1.5)
        #     cont=cont+1
        if cont==5:
            vel=0
            giro=90
            self.publisher2(giro,vel)
            print("YA ME CANSE Oropeza","Vel:",vel,"Giro",giro,"Contador",cont)
            rospy.signal_shutdown("Suicidios")
