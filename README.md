# hunter_mosaic_ros2

Repository contenente i file sorgente dell'applicativo ros2 che si interpone fra l'Agilex Hunter e l'applicativo Mosaic in esecuzione sul nodo HIL nella simulazione.

Tali cartelle sono da intendersi posizionate in ~/ros2_ws/src (ovvero nella sottocartella src del workspace ros2, che potrebbe variare).


# Setup dell'ambiente di sviluppo

### Setup iniziale dell'ambiente ros2:
  ~/ros2_ws$ source /opt/ros/humble/setup.bash

### Ricompilazione dell'applicativo ros2:
  ~/ros2_ws$ colcon build --packages-select cpp_alex_hunter

(è consigliato ricompilare tutto la prima volta, basterà omettere "--packages-select cpp_alex_hunter" dal comando precedente)

### Setup dell'ambiente di esecuzione:
Aggiornamento della lista di ciò che si può eseguire, credo sia necessario effettuarlo solo una volta, alla prima compilazione di cpp_alex_hunter col comando precedente.

  ~/ros2_ws$ source ./install/setup.bash

### Esecuzione dell'applicativo ros2:
  ~/ros2_ws$ ros2 run cpp_alex_hunter listener
