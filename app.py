from flask import Flask, request, jsonify
import heapq
from flask_cors import CORS
import networkx as nx

app = Flask(__name__)
CORS(app)

@app.route('/ruta-corta', methods=['POST'])
def calculate_route():
    data = request.json
    connections = data['connections']
    source_node = data['sourceNode']
    destination_node = data['destinationNode']

    # Crear un grafo con los datos de las conexiones
    graph = {}
    for connection in connections:
        start = connection['start']['address']
        end = connection['end']['address']
        distance = float(connection['distance'])
        if start not in graph:
            graph[start] = []
        if end not in graph:
            graph[end] = []
        graph[start].append((distance, end))
        graph[end].append((distance, start))

    # Implementar el algoritmo de Dijkstra
    def dijkstra(graph, start, end):
        queue, visited = [(0, start, [])], set()
        heapq.heapify(queue)

        while queue:
            (cost, node, path) = heapq.heappop(queue)
            if node not in visited:
                visited.add(node)
                path = path + [node]

                if node == end:
                    return (cost, path)

                for c, neighbour in graph.get(node, []):
                    if neighbour not in visited:
                        heapq.heappush(queue, (cost + c, neighbour, path))

        return float("inf"), []

    # Calcular la ruta más corta
    start_address = source_node['address']
    end_address = destination_node['address']
    cost, path = dijkstra(graph, start_address, end_address)

    # Crear una lista de nodos en orden que deben seguirse
    ordered_nodes = []
    for address in path:
        latlng = next((conn['start']['latlng'] for conn in connections if conn['start']['address'] == address), None)
        if latlng is None:
            latlng = next((conn['end']['latlng'] for conn in connections if conn['end']['address'] == address), None)
        ordered_nodes.append({'address': address, 'latlng': latlng})

    return jsonify({
        'cost': cost,
        'path': ordered_nodes
    })

@app.route('/arbol-expansion', methods=['POST'])
def calculate_mst():
    data = request.json
    connections = data['connections']
    source_node = data['sourceNode']
    destination_node = data['destinationNode']

    G = nx.Graph()

    for connection in connections:
        start_address = connection['start']['address']
        end_address = connection['end']['address']
        distance = float(connection['distance'])

        start_latlng = (connection['start']['latlng']['lat'], connection['start']['latlng']['lng'])
        end_latlng = (connection['end']['latlng']['lat'], connection['end']['latlng']['lng'])

        G.add_node(start_address, latlng=start_latlng)
        G.add_node(end_address, latlng=end_latlng)
        G.add_edge(start_address, end_address, weight=distance)

    mst = nx.minimum_spanning_tree(G, weight='weight')

    # Calculate total cost of the MST
    total_cost = sum(weight['weight'] for _, _, weight in mst.edges(data=True))

    # Extract the MST edges and create the path
    path = []
    for u, v, weight in mst.edges(data=True):
        path.append({
            'start': {
                'address': u,
                'latlng': {
                    'lat': G.nodes[u]['latlng'][0],
                    'lng': G.nodes[u]['latlng'][1]
                }
            },
            'end': {
                'address': v,
                'latlng': {
                    'lat': G.nodes[v]['latlng'][0],
                    'lng': G.nodes[v]['latlng'][1]
                }
            },
            'distance': str(weight['weight'])
        })

    # Create the response model
    response_model = {
        'cost': total_cost,
        'path': path
    }

    return jsonify(response_model)


if __name__ == '__main__':
    app.run()
