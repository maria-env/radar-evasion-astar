# Required imports
import numpy as np
import networkx as nx
from Boundaries import Boundaries
from Map import EPSILON

# Number of nodes expanded in the heuristic search (stored in a global variable to be updated from the heuristic functions)
NODES_EXPANDED = 0

def h1(current_node, objective_node) -> np.float32:
    """ First heuristic to implement: Distancia euclídea """
    global NODES_EXPANDED
    
    # Convertimos los nodos a tuplas de números
    actual = eval(current_node)
    objetivo = eval(objective_node)
    
    # Calculamos la distancia euclídea
    h = np.sqrt((actual[0] - objetivo[0])**2 + (actual[1] - objetivo[1])**2) * EPSILON
    
    NODES_EXPANDED += 1
    return h

def h2(current_node, objective_node) -> np.float32:
    """ Second heuristic to implement: Distancia de Manhattan """
    global NODES_EXPANDED
    
    # Convertimos los nodos a tuplas de números
    actual = eval(current_node)
    objetivo = eval(objective_node)
    
    # Calculamos la distancia de Manhattan
    h = (abs(actual[0] - objetivo[0]) + abs(actual[1] - objetivo[1])) * EPSILON
    
    NODES_EXPANDED += 1
    return h

def build_graph(detection_map: np.array, tolerance: np.float32) -> nx.DiGraph:
    """ Builds an adjacency graph (not an adjacency matrix) from the detection map """
    # Creamos el grafo
    G = nx.DiGraph()
    
    # Obtenemos ancho y alto del mapa
    alto, ancho = detection_map.shape
    
    # Direcciones posibles
    direcciones = [(1, 0), (-1, 0), (0, -1), (0, 1)]
    
    # Vamos a cada celda del mapa
    for i in range(alto):
        for j in range(ancho):
            # Creamos un nodo con la posición de la celda actual
            nodo = str((i, j))
            
            # Tomamos en cuenta todas las direcciones posibles
            for direccion_i, direccion_j in direcciones:
                nodo_i, nodo_j = i + direccion_i, j + direccion_j
                
                # Verificamos que la dirección a la que queremos ir es posible de ejecutar
                if 0 <= nodo_i < alto and 0 <= nodo_j < ancho:
                    
                    # Creamos el nodo adyacente
                    adyacente = str((nodo_i, nodo_j))
                    
                    # Añadimos una arista al nuevo nodo si el nivel de detección en la celda es menor al indicado
                    if detection_map[nodo_i, nodo_j] <= tolerance:
                        G.add_edge(nodo, adyacente, weight=detection_map[nodo_i, nodo_j])
    
    return G

def discretize_coords(high_level_plan: np.array, boundaries: Boundaries, map_width: np.int32, map_height: np.int32) -> np.array:
    """ Converts coordiantes from (lat, lon) into (x, y) """
    # Obtenemos las latitudes y longitudes
    latitudes = np.linspace(start=boundaries.max_lat, stop=boundaries.min_lat, num=map_height)
    longitudes = np.linspace(start=boundaries.min_lon, stop=boundaries.max_lon, num=map_width)
    
    # Creamos una lista de numpy con el mismo tamaño que high_level_plan para 
    # almacenar las coordenadas (x, y) de cada (lat, lon)
    coordenadas = np.zeros((high_level_plan.shape[0], 2), dtype=np.int32)
    
    # Accedemos a cada punto de high_level_plan
    for i in range(high_level_plan.shape[0]):
        lat_punto, long_punto = high_level_plan[i]
        
        # Buscamos el valor más cercano de entre todos los del mapa
        fila_punto = np.abs(latitudes - lat_punto).argmin()
        columna_punto = np.abs(longitudes - long_punto).argmin()
        
        # Almacenar coordenadas
        coordenadas[i] = [fila_punto, columna_punto]
    
    return coordenadas

def path_finding(G: nx.DiGraph,
                 heuristic_function,
                 locations: np.array, 
                 initial_location_index: np.int32, 
                 boundaries: Boundaries,
                 map_width: np.int32,
                 map_height: np.int32) -> tuple:
    """ Implementation of the main searching / path finding algorithm """
    global NODES_EXPANDED
    NODES_EXPANDED = 0
    
    # Utilizamos la función que pasa de latitudes y longitudes a coordenadas
    coordenadas = discretize_coords(locations, boundaries, map_width, map_height)
    
    # Creamos la lista donde guardaremos la solución
    solucion = []
    
    # Elegimos el orden de visita (el dado directamente por locations) y el nodo inicial
    orden_visita = []
    for i in range(len(locations)):
        orden_visita.append(i)
    
    indice_actual = initial_location_index
    
    # Colocamos el índice inicial al principio de nuestro orden de visita
    if indice_actual != 0:
        orden_visita.remove(indice_actual)
        orden_visita.insert(0, indice_actual)
    
    # Obtenemos los indices del nodo de origen y el de destino para cada par de puntos
    for i in range(len(orden_visita) - 1):
        indice_origen = orden_visita[i]
        indice_destino = orden_visita[i + 1]
        
        # Obtenemos sus coordenadas
        coordenadas_origen = coordenadas[indice_origen]
        coordenadas_destino = coordenadas[indice_destino]
        
        # Convertir coordenadas a formato de nodo
        nodo_origen = f"({coordenadas_origen[0]}, {coordenadas_origen[1]})"
        nodo_destino = f"({coordenadas_destino[0]}, {coordenadas_destino[1]})"
        
        # Aplicamos A*
        camino = nx.astar_path(G, nodo_origen, nodo_destino, heuristic=heuristic_function, weight='weight')

        # Añadimos el camino a nuestra solución
        solucion.append(camino)
    
    return solucion, NODES_EXPANDED

def compute_path_cost(G: nx.DiGraph, solution_plan: list) -> np.float32:
    """ Computes the total cost of the whole planning solution """
    coste = 0.0
    
    # Para cada camino de la solución
    for camino in solution_plan:
        # Por cada par de nodos del camino, sumamos el coste de la arista
        for i in range(len(camino) - 1):
            coste += G[camino[i]][camino[i+1]]['weight']
    
    return coste