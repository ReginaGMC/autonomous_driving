# autonomous_driving
Simulación en ROS de múltiples pistas para el TurtleBot3 Waffle Pi de conducción autónoma con visión computacional

Para poder correr el proyecto primero se debe de descargar la carpeta 'autonomous driving' en el directorio '~/catkin_ws/src'. Las carpetas con los modelos se deben de agregar al directorio general de los modelos de Gazebo ('~/.gazebo/models').

En una terminal primero se debe de iniciar el nodo de ROS

```$ roscore ```

En una nueva ventana de la terminal se importa el modelo de TurtleBot3 (`burger`, `waffle`, o `waffle_pi`) y se empieza la simulación en Gazebo de alguno de los tres mundos (`autonomous_driving.launch`, `autonomous_driving1.launch`, o `autonomous_driving2.launch`).

```
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch autonomous_driving autonomous_driving.launch
```

Finalmente, en una nueva ventana de la terminal se corre el comando para empezar el programa de Python de conducción autónoma.

```$ rosrun autonomous_driving autonomous_driving.py```

Si el programa no funciona automátiamente es posible que ROS no lo esté detectando como un programa de Python, por lo que sería necesario correr sólo una vez el siguiente comando.

```$ chmod +x autonomous_driving.py ```

El programa correrá la simulación hasta que se fuerce el cierre. Para asegurarse de que el programa y la simulación se cerraron correctamente se debe de presion `ctrl+c` en todas las ventanas en orden (el programa de Python, la simulación en Gazebo, y el nodo de ROS). 

Si en algún momento no se cierra correctamente el sistema y al tratar de abrir la simulación se muestran errores se puede forzar el cierre de la simulación, después del nodo de ROS, cerrar la terminal y volver a correrlo. 

Finalmente, si el sistema físico (la computadora) se encuentra sobrecalentada o se corrieron múltiples simulaciones seguidas es posible que el sistema baje su rendimiento, por lo que se recomienda para todas las simulaciones y esperar hasta que el sistema ya no se encuentre sobrecalentado.
