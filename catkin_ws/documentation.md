<h1>Documentacion para el automodel</h1> <h3>(Para mena y santiago)</h3>

****
<h5>Componentes:</h5>
    <h6>    interes:</h6>

1)Camara de profundiad Realsense SenZ3D

2)Fisheye camara ELP 1080        

3)IMU MPC6050 -->Giroscopio y Acelerometro

4)Odroid WIFI module

<h6>inutiles para nosotros ahorita:</h6>

1)arduino nano

2)4 port usb hub

3)Odroid XU4

4)Reguladores de voltaje    

5)Motores
****
<h5>Topicos:</h5>

**Topico camara profundidad:** /app/camera/depth/image_raw

*notas:* si utilizas el comando: **rosrun image_view image_view image:=/app/camera/rgb/image_raw** puedes ver una ventana con el video que transmite el automodel

****

<h5> Tecnolgias/librerias usadas y lineas de codigo importantes</h5>

**OpenCV**
Lo usamos para reconocer objetos con el comando openCv.CHINgTumadreyallhel

****
<h2>Errores que surgieron y su soulcion:</h2>

<h3>Santiago</h3>


**error:** TypeError: unbound method imgmsg_to_cv2() must be called with CvBridge instance as first argument (got Image instance instead)

**Solucion:** Este error se origino de una confusion entre metodos unbound y bound, unbound es cuando el def no esta ligado a la clase y bound cuando
esta ligado a la misma, era necesario instanciar un objeto de CvBridge para acceder al metodo


<h3>Mena(ANO ANCHO)</h3>



****


