import osmnx as ox
import networkx as nx
import plotly.graph_objects as go
import numpy as np
import pandas as pd

from IPython.display import IFrame


# north, east, south, west = 13.79196, 48.93428, 11.91605, 49.47799
# G = ox.graph_from_bbox(north, south, east, west)

location_point = (49.2181757037439, 12.659965924974259)
sparkasse_roding = (49.235137, 12.706825)
dest = (49.14685, 12.342892)

ox.config(use_cache=True, log_console=True)
# G = ox.graph_from_point(location_point, dist=35000, dist_type="bbox", network_type="drive")
# G = ox.save_graphml(G, 'cham_35km.graphml')
G = ox.load_graphml('cham_35km.graphml')
# G = ox.graph_from_place('Regensburg, Germany')

# print(G.edges(keys=True, data=True))

# quit()

# ox.plot_graph(G)

# df = pd.read_csv('../haltestellenAktuell.csv')
# df['test'] = list(zip(df.Laengengrad, df.Breitengrad))
# tuples = df.test

# for index, value in tuples.items():
#     print(f"Index : {index}, Value: {value}")

origin_node = ox.nearest_nodes(G, sparkasse_roding[1], sparkasse_roding[0])
# nodes = G.nodes()

# print(f'{sparkasse_roding} {nodes[origin_node]}')
def plot_path(lat, long, origin_point, destination_point):
    fig = go.Figure(go.Scattermapbox(
        name = "Path",
        mode = "lines",
        lon = long,
        lat = lat,
        marker = {'size': 10},
        line = dict(width = 4.5, color = 'blue')))

    fig.add_trace(go.Scattermapbox(
        name = "Source",
        mode = "markers",
        lon = [origin_point[1]],
        lat = [origin_point[0]],
        marker = {'size': 12, 'color':"red"}))

    # adding destination marker
    fig.add_trace(go.Scattermapbox(
        name = "Destination",
        mode = "markers",
        lon = [destination_point[1]],
        lat = [destination_point[0]],
        marker = {'size': 12, 'color':'green'}))

    # getting center for plots:
    lat_center = np.mean(lat)
    long_center = np.mean(long)

    # defining the layout using mapbox_style
    fig.update_layout(mapbox_style="stamen-terrain", mapbox_center_lat = 30, mapbox_center_lon=-80)
    fig.update_layout(margin={"r":0,"t":0,"l":0,"b":0},
                     mapbox = {
                         'center': {'lat': lat_center, 'lon': long_center},
                         'zoom': 9})

    fig.show()


df = pd.read_csv('../haltestellenAktuell.csv')
df['test'] = list(zip(df.Breitengrad, df.Laengengrad))
tuples = df.test
routes = []

for i in range(len(tuples)):
    print(f'Route to {i}. node: {tuples[i]}')
    dest_node = ox.nearest_nodes(G, tuples[i][1], tuples[i][0])

    routes.append(nx.shortest_path(G, origin_node, dest_node, 'length'))

    # lines = node_list_to_path(G, route)

    # long2 = []
    # lat2 = []

    # # for i in route:
    # #     point = G.nodes[i]
    # #     long.append(point['x'])
    # #     lat.append(point['y'])

    # for i in range(len(lines)):
    #     z = list(lines[i])
    #     l1 = list(list(zip(*z))[0])
    #     l2 = list(list(zip(*z))[1])

    #     for j in range(len(l1)):
    #         long2.append(l1[j])
    #         lat2.append(l2[j])

    # plot_path(lat2, long2, sparkasse_roding, dest)

# fig,ax = ox.plot_graph(G, node_color='r')
# m1 = ox.plot_graph_folium(G, popup_attribute="name", weight=2, color="#8b0000")
# filepath = "./data/graph.html"
# m1.save(filepath)
# IFrame(filepath, width=600, height=500)

colors = ox.plot.get_colors(len(routes))
fig, ax = ox.plot_graph_routes(G, routes, route_colors=colors ,route_linewidth=6, node_size=1)


def node_list_to_path(G, node_list):
    """
    Given a list of nodes, return a list of lines that together follow the path
    defined by the list of nodes.
    Parameters
    ----------
    G : networkx multidigraph
    route : list
        the route as a list of nodes
    Returns
    -------
    lines : list of lines given as pairs ( (x_start, y_start), (x_stop, y_stop) )
    """
    edge_nodes = list(zip(node_list[:-1], node_list[1:]))
    lines = []
    for u, v in edge_nodes:
        # if there are parallel edges, select the shortest in length
        data = min(G.get_edge_data(u, v).values(), key=lambda x: x['length'])

        # if it has a geometry attribute (ie, a list of line segments)
        if 'geometry' in data:
            # add them to the list of lines to plot
            xs, ys = data['geometry'].xy
            lines.append(list(zip(xs, ys)))
        else:
            # if it doesn't have a geometry attribute, the edge is a straight
            # line from node to node
            x1 = G.nodes[u]['x']
            y1 = G.nodes[u]['y']
            x2 = G.nodes[v]['x']
            y2 = G.nodes[v]['y']
            line = [(x1, y1), (x2, y2)]
            lines.append(line)
    return lines
