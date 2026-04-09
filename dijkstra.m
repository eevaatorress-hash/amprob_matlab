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