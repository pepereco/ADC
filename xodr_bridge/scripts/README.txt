COM FER SERVIR AQUESTA PART DEL CODI:
=====================================


- Crear graf a partir del fitxer XODR:

    xodr.get_graph_from_xodr_file( xodr_path : str ) -> retorna Graph()


- De unes Coords del cotxe obtenir la posició precisa, la connexió dins del graf i la distància dins la connexió:

    Graph.find_xodr_connection( car_coords : Coords ) -> retorna tuple (Connection, length, precise_coords)

    Coords és una classe pròpia amb arguments x, y i yaw (yaw és la orientació en radians).


- De dos Coordenades del cotxe (no cal que siguin precises), obtenir una instància de Path contenint el recorregut:

    Graph.from_point_to_point( current_coords : Coords, destination_coords : Coords ) -> Path

    Path conté una llista de yaw's indicant els radians que cal girar a cada intersecció, i una distància que cal recòrrer un cop
    arribats a la última carretera. Es pot accedir mitjançant Path.angles i Path.lastmeters.


- Indicar que no es pot anar per a un carrer (en un sentit), per a tenir-ho en compte en properes navegacions.

    Graph.can_not_turn( car_coords : Coords, direction : float ) -> intersecció

    Retorna 0 si ha pogut eliminar la connexió que es troba mirant a la direcció ABSOLUTA de la intersecció més pròxima a car_coords.
    Retorna un altre nombre en cas que no s'hagi trobat cap intersecció prou precisa (potser el cotxe no es troba massa proper a la
    intersecció, o l'angle absolut no és prou precís).

    1 -> Cotxe massa lluny de la intersecció més propera. ( Treshold: graph.py: DISTANCE_TOLERANCE )
    2 -> Cap intersecció en aquella direcció, potser ja ha estat eliminada. ( Treshold: graph.py: YAW_TOLERANCE )


YAWS:

          1.57
           |
           |
 3.14 ---- · ----  0
           |
           |
         -1.57