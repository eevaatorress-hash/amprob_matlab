# Repositorio para Ampliación de robótica
Prácticas de Ampliación de robótica en matlab.

## Práctica 4
Desarrollo de módulo de navegación reactivo en tiempo real utilizando los sensores del robot mientras este navega entre los puntos inicio y destino, los cuales son determinados al ejecutar el programa. Para ello, se utilizan __campos potenciales__ con fuerzas de atracción (hacia el punto destino) y fuerzas de repulsión (producidas por los obstáculos).
Esta práctica se ve comprendida en el archivo `plantilla_campos_potenciales.m`, para observar los resultados obtenidos simplemente hay que ejecutar ese archivo y seleccionar puntos de inicio y fin en el mapa emergente.

## Práctica 5
Desarrollo de módulo de planificación de caminos utilizando el algortimo Dijkstra. El cual recibe como parámetros la tabla de adyacencias de los nodos (G), el nodo de origen y el de destino.
Esta práctica se ve comprendida en el archivo `dijkstra.m`. 
Sin embargo, es de suma importancia cargar el archivo `grafos.mat` antes de realizar cualquier prueba, puesto que este archivo contiene las tablas de adyacencias necesarias para el computo de este algoritmo. Además, será necesario para la práctica 6.
Para observar los resultados se escribirá en la command window `[coste,ruta] = dijkstra(Grafo,inicio,destino)`, donde grafo se corresponde a la tabla de adyacencias del grafo que se quiera estudiar.

## Práctica 6
Desarrollo del módulo de planificación de caminos utilizando el algoritmo A*. A la práctica anterior se le añadirá como parámetro una Heurística (H) que reducirá el coste computacional del algoritmo al evitar explorar secciones no necesarias.
Esta práctica se ve comprendida en el archivo `aestrella.m`.
Para observar los resultados se escribirá en la command window `[coste,ruta] = aestrella(Grafo,Heuristica,inicio,destino)`.

## Práctica 7
Uso del conjunto de prácticas anteriores para conseguir una navegación totalmente autónoma a partir de nodos de origen y destino. 
Esta práctica se ve comprendida en los archivos `practica7_dijkstra.m` y `practica7_aestrella.m`, para observar los resultados obtenidos simplemente hay que ejecutar cada archivo y seleccionar los nodos de inicio y fin en la command window.


