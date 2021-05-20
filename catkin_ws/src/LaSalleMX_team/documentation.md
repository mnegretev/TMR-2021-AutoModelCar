<h1>Documentacion para el automodel</h1> <h3>Team La Salle</h3>

****
****
<h5>Topicos:</h5>

**Topico camara utilizado:** /app/camera/depth/image_raw
**Topicos Nuevos:** /instruccionesAutomodel
****

<h5> Tecnolgias/librerias/paquetes usadas y lineas de codigo importantes</h5>

**OpenCV**

**CvBridge**
****
<h5> Tipos de mensajes auxiliares que utilizamos</h5>

**sensor_msgs**

**image_transport**


****
<h5> Comando para iniciar el paquete</h5>
Una ves se haya iniciado la simulacion de gazebo , lanza el launch file selfdriving con el siguiente comando:

        roslaunch team_lasalle selfdriving.launch

Version esquiva obstaculos requiere de :

        roslaunch team_lasalle selfdriving2.launch
****