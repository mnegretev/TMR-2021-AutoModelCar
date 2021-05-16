# TMR 2021 - AVIM

Este repositorio es que se utilizará para la categoría AutoModelCar del Torneo Mexicano de Robótica 2021 a distancia. La dinámica para las pruebas se encuentra en https://www.femexrobotica.org/tmr2021/portfolio-item/autos-autonomos.

## Requerimientos:

* Ubuntu 18.04
* ROS Melodic

## Instalación:

El vehículo simulado para esta categoría está basado en el trabajo realizado por el equipo Eagle Knights del ITAM. Las instrucciones de instalación son las mismas que se encuentran en https://github.com/ITAM-Robotica/Eagle_Knights-Wiki/wiki

## Pruebas

Una vez instalado y compilado el repositorio, se pueden ejecutar los ambientes de simulación con los respectivos archivos 'launch':

* Navegación autónoma sin obstáculos: roslaunch bring_up navigation_without_obstacles.launch
* Navegación con obstáculos estáticos: roslaunch bring_up navigation_with_static_obstacles.launch
* Navegación con obstáculos dinámicos: roslaunch bring_up navigation_with_dynamic_obstacles.launch
* Prueba de estacionamiento: roslaunch bring_up parking.test

En cada prueba, el robot inicia en una posición y orientación aleatoria (dentro de un intervalo razonable).

## Tópicos relevantes

Este vehículo es una versión simulada de la versión 1 del vehículo a escala desarrollado por la Freie Universität Berlin, por lo que los nombres y tipos de tópicos corresponden con las especificaciones de dicho modelo, que se pueden consultar en https://github.com/AutoModelCar/AutoModelCarWiki/wiki.

## Contacto

Cualquier duda o comentario sobre esta prueba, escribir al responsable técnico:

Marco Negrete<br>
marco.negrete@ingenieria.unam.edu

