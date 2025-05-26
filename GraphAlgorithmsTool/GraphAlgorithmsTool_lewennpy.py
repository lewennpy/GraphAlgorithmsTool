from collections import defaultdict, deque
import heapq

class Grafo:
    def __init__(self):
        self.ady = defaultdict(list)
        self.nodos = set()
    
    def agregar_arista(self, u, v, peso):
        self.ady[u].append((v, peso))
        self.ady[v].append((u, peso))  # Grafo no dirigido
        self.nodos.add(u)
        self.nodos.add(v)
    
    def mostrar(self):
        print("\nLista de adyacencia:")
        for u in sorted(self.nodos):
            print(f"{u}: ", end="")
            for (v, w) in self.ady[u]:
                print(f"({v}, peso={w}) ", end="")
            print()

def leer_grafo(nombre_archivo):
    grafo = Grafo()
    with open(nombre_archivo, "r") as f:
        lineas = f.readlines()
    for linea in lineas:
        linea = linea.strip()
        if linea == "" or linea.startswith("N"):
            continue
        u, v, w = linea.split()
        grafo.agregar_arista(u, v, int(w))
    return grafo

def dijkstra(grafo, inicio):
    dist = {n: float('inf') for n in grafo.nodos}
    dist[inicio] = 0
    prev = {n: None for n in grafo.nodos}
    heap = [(0, inicio)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        for (v, w) in grafo.ady[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    return dist, prev

def reconstruir_camino(prev, inicio, fin):
    path = []
    u = fin
    while u is not None:
        path.append(u)
        u = prev[u]
    path.reverse()
    if path[0] == inicio:
        return path
    return []

def floyd_warshall(grafo):
    nodos = list(grafo.nodos)
    dist = {u: {v: float('inf') for v in nodos} for u in nodos}
    next_node = {u: {v: None for v in nodos} for u in nodos}
    
    for u in nodos:
        dist[u][u] = 0
    for u in nodos:
        for (v, w) in grafo.ady[u]:
            dist[u][v] = w
            next_node[u][v] = v
    
    for k in nodos:
        for i in nodos:
            for j in nodos:
                if dist[i][k] + dist[k][j] < dist[i][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    next_node[i][j] = next_node[i][k]
    return dist, next_node

def reconstruir_camino_fw(next_node, inicio, fin):
    if next_node[inicio][fin] is None:
        return []
    path = [inicio]
    while inicio != fin:
        inicio = next_node[inicio][fin]
        path.append(inicio)
    return path

def prim(grafo, inicio):
    nodos = grafo.nodos
    visitado = set([inicio])
    aristas = []
    heap = []
    for (v, w) in grafo.ady[inicio]:
        heapq.heappush(heap, (w, inicio, v))
    costo_total = 0
    
    while heap and len(visitado) < len(nodos):
        w, u, v = heapq.heappop(heap)
        if v in visitado:
            continue
        visitado.add(v)
        aristas.append((u, v, w))
        costo_total += w
        for (nx, pw) in grafo.ady[v]:
            if nx not in visitado:
                heapq.heappush(heap, (pw, v, nx))
    return aristas, costo_total

def kruskal(grafo):
    # Usaremos Union-Find
    parent = {}
    rank = {}
    
    def find(u):
        while parent[u] != u:
            parent[u] = parent[parent[u]]
            u = parent[u]
        return u
    
    def union(u, v):
        ru, rv = find(u), find(v)
        if ru == rv:
            return False
        if rank[ru] < rank[rv]:
            parent[ru] = rv
        elif rank[rv] < rank[ru]:
            parent[rv] = ru
        else:
            parent[rv] = ru
            rank[ru] += 1
        return True
    
    for nodo in grafo.nodos:
        parent[nodo] = nodo
        rank[nodo] = 0
    
    aristas = []
    for u in grafo.ady:
        for (v, w) in grafo.ady[u]:
            if u < v:  # para no duplicar en grafo no dirigido
                aristas.append((w, u, v))
    aristas.sort()
    
    mst = []
    costo_total = 0
    for (w, u, v) in aristas:
        if union(u, v):
            mst.append((u, v, w))
            costo_total += w
    return mst, costo_total

def bfs(grafo, inicio):
    visitado = set([inicio])
    cola = deque([inicio])
    orden = []
    while cola:
        u = cola.popleft()
        orden.append(u)
        for (v, _) in grafo.ady[u]:
            if v not in visitado:
                visitado.add(v)
                cola.append(v)
    return orden

def dfs(grafo, inicio):
    visitado = set()
    orden = []
    def dfs_visit(u):
        visitado.add(u)
        orden.append(u)
        for (v, _) in grafo.ady[u]:
            if v not in visitado:
                dfs_visit(v)
    dfs_visit(inicio)
    return orden

def main():
    print("Algoritmos disponibles:")
    print("1 - Dijkstra")
    print("2 - Floyd-Warshall")
    print("3 - Prim")
    print("4 - Kruskal")
    print("5 - Búsqueda en anchura (BFS)")
    print("6 - Búsqueda en profundidad (DFS)")
    
    opcion = int(input("Elige el algoritmo a aplicar (1-6): "))
    archivo = input("Ingresa el nombre del archivo del grafo: ")
    
    grafo = leer_grafo(archivo)
    grafo.mostrar()
    
    if opcion == 1:
        inicio = input("Nodo inicial: ")
        fin = input("Nodo final: ")
        dist, prev = dijkstra(grafo, inicio)
        camino = reconstruir_camino(prev, inicio, fin)
        if camino:
            print(f"Camino más corto: {' -> '.join(camino)}")
            print(f"Costo total: {dist[fin]}")
        else:
            print("No hay camino entre los nodos.")
    
    elif opcion == 2:
        dist, next_node = floyd_warshall(grafo)
        inicio = input("Nodo inicial: ")
        fin = input("Nodo final: ")
        camino = reconstruir_camino_fw(next_node, inicio, fin)
        if camino:
            print(f"Camino más corto (Floyd-Warshall): {' -> '.join(camino)}")
            print(f"Costo total: {dist[inicio][fin]}")
        else:
            print("No hay camino entre los nodos.")
    
    elif opcion == 3:
        inicio = input("Nodo inicial para Prim: ")
        aristas, costo = prim(grafo, inicio)
        print("Árbol de expansión mínima (Prim):")
        for (u, v, w) in aristas:
            print(f"{u} - {v} (peso {w})")
        print(f"Costo total: {costo}")
    
    elif opcion == 4:
        mst, costo = kruskal(grafo)
        print("Árbol de expansión mínima (Kruskal):")
        for (u, v, w) in mst:
            print(f"{u} - {v} (peso {w})")
        print(f"Costo total: {costo}")
    
    elif opcion == 5:
        inicio = input("Nodo inicial para BFS: ")
        orden = bfs(grafo, inicio)
        print("Recorrido BFS:")
        print(" -> ".join(orden))
    
    elif opcion == 6:
        inicio = input("Nodo inicial para DFS: ")
        orden = dfs(grafo, inicio)
        print("Recorrido DFS:")
        print(" -> ".join(orden))
    
    else:
        print("Opción inválida.")

if __name__ == "__main__":
    main()
