# TFM

Para instalar el paquete de ros principal, jaguar_ros, añadirlo al catkin_ws y hacer cmake catkin_make. Se lanza con un roslaunch teleop_jaguar.launch. 
Tiene varias dependencias que se deben instalar con apt. El nodo joy para controlarlo con un joystick, gazebo11-ros-packages, jsk-visualization, y pcl-messages. 

Para instalar el paquete de ros de Karim de filtros de pointcloud para el velodyne es igual, copiar a catkin_ws y catkin_make, las dependencias estan añadidas en la lista anterior. 

El plugin de gazebo de jaguar se debe compilar con cmake y make. Esta añadido en el URDF del robot y en el SDF por lo que es necesario para correr la simulacion. Los otros plugins, son modificaciones de plugins que trae gazebo11 de serie. Yo lo tengo instalado en copia local y lo compilo para poder modificar estos plugins. Se descarga Gazebo el codigo de gazebo, se sustituyen todos los archivos con el mismo nombre y el CMakeLists.txt y se compila, tarda un rato, si se hace make -jX siendo X los nucleos de procesador que se puedan usar va un poco mas rápido. 

Con esto deberia funcionar todo bien al ejecutar teleop_jaguar.launch


