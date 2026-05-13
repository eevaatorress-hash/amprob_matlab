% AMPLIACION DE ROBOTICA
% PRACTICA 7: Navegación autónoma utilizando el algoritmo Dijkstra


clc
clearvars
close all
%% Carga del mapa de ocupacion

map_img=imread('mapa2.pgm');
map_neg=imcomplement(map_img);
map_bin=imbinarize(map_neg);
mapa=binaryOccupancyMap(map_bin);

%% Cargamos los nodos en el mapa
mapa2
show(mapa);
hold on;
plot(nodos(:,2), nodos(:,3), "r*");

% Decidir cual es el siguiente nodo
nodo_origen = input('Indica el nodo de inicio: ');
nodo_destino = input('Indica el nodo de destino: ');

% Configuracion del sensor (laser de barrido)
max_rango=10;
angulos=-pi/2:(pi/180):pi/2; % resolucion angular barrido laser

% Caracteristicas del vehiculo y parametros del metodo
v=0.4;            % Velocidad del robot
D=20;           % Rango del efecto del campo de repulsión de los obstáculos
alfa=2;           % Coeficiente de la componente de atracción
beta=100;      % Coeficiente de la componente de repulsión

%% Inicialización
meta = true; %para ver si ha llegado al destino o no
origen = nodos(nodo_origen,2:3);
destino = nodos(nodo_destino,2:3);

obstaculos = map_bin == 1; 

[y,x] = find(obstaculos);
alto = size(map_bin,1);
y = alto - y + 1;

pos_ant = zeros(10,2);  % Aquí guardaremos las últimas 10 posiciones del robot
%% Calculo de la trayectoria
Frep_sum = [0,0];

[coste,ruta] = dijkstra(costes,nodo_origen, nodo_destino);

origen = nodos(nodo_origen,2:3);
robot=[origen 0];     % El robot empieza en la posición de origen (orientacion cero)

for j= 1:length(ruta)-1
    nodo_origen = ruta(j);
    nodo_destino = ruta(j+1);
    origen = nodos(nodo_origen,2:3);
    destino = nodos(nodo_destino,2:3);

    path = [];                 % Se almacena el camino recorrido
    path = [path; robot];      % Se añade al camino la posicion actual del robot
    iteracion=0;               % Se controla el nº de iteraciones por si se entra en un minimo local

    while norm(destino-robot(1:2)) > 1.0 && iteracion<1000    % Hasta menos de una iteración de la meta (10 cm)
        if iteracion <= 10
            dif_dist = 5; %valor random mientras no es relevante
            pos_ant(iteracion+1,:) = robot(1:2); % guardamos las primeras 10 posiciones
        else
            dif_dist = norm(pos_ant(1,:)-robot(1:2)); % diferencia entre la posición actual y la de hace diez iteraciones
            pos_ant(1,:) = [];
            pos_ant(10,:) = robot(1:2);
        end

        if dif_dist<=2
            beta = 30;
            alpha = 10;
        else 
            beta = 100;
            alpha = 2;
        end
        % Calculamos fuerzas de atraccion
        Fatr = alfa*(destino - robot(1:2));
    
        % Ahora calculamos las fuerzas de repulsion
        Frep_sum = [0,0];
        for k = 1:length(x)
            dist_obst(k) = sqrt((x(k)-robot(1))^2 + (y(k)-robot(2))^2);
            if (dist_obst(k)<= D)
                Frep(k,1:2) = beta*(1/dist_obst(k) -1/D)*(robot(1:2)-[x(k),y(k)])/((dist_obst(k))^3);
            else
                Frep(k,1:2) = [0,0];
            end
        Frep_sum = Frep_sum + Frep(k,:);
        end
       
        Fres = Fatr + Frep_sum;
        theta = atan2(Fres(2),Fres(1));
        robot = robot + [v*cos(theta), v*sin(theta),theta];
    
    
        path = [path;robot];	% Se añade la nueva posición al camino seguido
        plot(path(:,1),path(:,2),'r');
        drawnow
    
        iteracion=iteracion+1;
        if iteracion==1000   % Se ha caído en un mínimo local
            fprintf('No se ha podido llegar del %d al %d\n', nodo_origen, nodo_destino)
            meta = false;
            origen = nodos(nodo_destino,2:3);
            robot=[origen 0];
        end
    end
end

if ~meta   % Se ha caído en un mínimo local
    fprintf('No se ha podido llegar al destino.\n')
else
    fprintf('Destino alcanzado.\n')
end
%% Algoritmo dijkstra
function [coste,ruta] = dijkstra(Grafo,inicio,destino)
    numnodos = size(Grafo,1);

    nodos = [1:numnodos]';
    fn = Inf(numnodos,1);
    p = zeros(numnodos,1);

    camino = [nodos,fn,p];
    camino(inicio,2) = 0;

    % Añadimos un vector que nos vaya guardando los nodos que ya estan
    % cerrados
    visitado = false(numnodos,1);

    % Generamos un bucle que nos vaya guardando los costes totales y referencia
    % al nodo del que obtienen ese coste (padre) hasta llegar al nodo destino
    nodo_act = inicio;
    
    while ~visitado(destino)
        % Si el menor coste disponible es infinito, no existe camino
        if isinf(camino(nodo_act,2))
            break
        end

        fila = Grafo(nodo_act,:)'; % Extraemos los costes de nodos adyacentes de la tabla

        nuevo_coste = camino(nodo_act,2) + fila;
        mask = (fila > 0) & ~visitado & (camino(:,2) > nuevo_coste);
        camino(mask,2) = nuevo_coste(mask);
        camino(mask,3) = nodo_act;
    
        visitado(nodo_act) = true;

        % Elegimos el siguiente nodo no visitado con menor coste acumulado
        costes_temp = camino(:,2);
        costes_temp(visitado) = Inf;
    
        % Comprobamos si hay nodo alcanzable
        if all(isinf(costes_temp))
            break
        end
        
        [~, nodo_act] = min(costes_temp);
    end

    % Ya cuando llegue al nodo destino nos podriamos salir del bucle (while en
    % este caso)

    % El coste lo podemos conseguir mirando directamente la tabla de camino
    coste = camino(destino,2);


    % Si no hay camino, devolvemos ruta vacía
    if isinf(coste)
        ruta = [];
        return
    end

    % Habiendo tenido en cuenta lo anterior podemos generar la ruta
    nodo_act = destino;
    ruta = destino;

    while nodo_act~=inicio
        nodo_act = camino(nodo_act,3);
        ruta = [nodo_act ruta];
    end

end

