import streamlit as st
import pandas as pd
import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import folium
from streamlit_folium import folium_static

st.set_page_config(page_title="Optimización de Rutas", layout="wide")

# ----------------------------
# FUNCIONES DE CÁLCULO
# ----------------------------
def create_data_model(distance_matrix, demands=None, vehicle_capacities=None,
                      time_windows=None, cost_per_km=1):
    data = {
        'distance_matrix': distance_matrix,
        'num_vehicles': len(vehicle_capacities) if vehicle_capacities else 1,
        'depot': 0,
        'cost_per_km': cost_per_km,
    }
    if demands:
        data['demands'] = demands
    if vehicle_capacities:
        data['vehicle_capacities'] = vehicle_capacities
    if time_windows:
        data['time_windows'] = time_windows
    return data


def solve_vrp(data, use_capacity=False, use_time_windows=False):
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(data['distance_matrix'][from_node][to_node] * data['cost_per_km'])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    if use_capacity:
        def demand_callback(from_index):
            return data['demands'][manager.IndexToNode(from_index)]

        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,
            data['vehicle_capacities'],
            True,
            'Capacity')

    if use_time_windows:
        def time_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['distance_matrix'][from_node][to_node]

        time_callback_index = routing.RegisterTransitCallback(time_callback)
        routing.AddDimension(
            time_callback_index,
            30,
            1440,
            False,
            'Time')
        time_dimension = routing.GetDimensionOrDie('Time')
        for location_idx, time_window in enumerate(data['time_windows']):
            index = manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    solution = routing.SolveWithParameters(search_parameters)
    if not solution:
        return None, manager, routing
    return solution, manager, routing


def get_routes(solution, manager, routing, data):
    routes = []
    total_cost = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route = []
        route_cost = 0
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route.append(node)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_cost += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        route.append(manager.IndexToNode(index))
        total_cost += route_cost
        routes.append((route, route_cost))
    return routes, total_cost


# ----------------------------
# INTERFAZ STREAMLIT
# ----------------------------
st.title("📦 Optimización de Rutas con OR-Tools")
model_type = st.selectbox("Selecciona el tipo de modelo", ["VRP", "CVRP", "VRPTW"])

with st.expander("Carga de datos"):
    n = st.number_input("Cantidad de nodos (incluyendo el depósito)", min_value=2, max_value=50, value=5)
    distance_matrix = []
    st.write("Ingresa la matriz de distancias (en km):")
    for i in range(n):
        row = st.text_input(f"Fila {i} (separada por comas)", value=','.join(['0']*n))
        distance_matrix.append(list(map(float, row.strip().split(','))))

    cost_per_km = st.number_input("Costo por km recorrido", min_value=0.0, value=1.0)

    demands = []
    capacities = []
    time_windows = []

    if model_type in ["CVRP", "VRPTW"]:
        st.write("Demanda por cliente (debe comenzar en 0 para el depósito):")
        for i in range(n):
            d = st.number_input(f"Demanda nodo {i}", min_value=0, value=0, key=f"demanda_{i}")
            demands.append(d)

        num_vehicles = st.number_input("Número de vehículos", min_value=1, value=2)
        for v in range(num_vehicles):
            c = st.number_input(f"Capacidad vehículo {v}", min_value=1, value=15, key=f"capacidad_{v}")
            capacities.append(c)
    else:
        num_vehicles = 1

    if model_type == "VRPTW":
        st.write("Ventanas de tiempo para cada nodo (ej: 0,100)")
        for i in range(n):
            tw = st.text_input(f"Ventana de tiempo nodo {i}", value="0,1000", key=f"tw_{i}")
            tw_parsed = list(map(int, tw.strip().split(',')))
            time_windows.append(tuple(tw_parsed))

# ----------------------------
# RESOLVER
# ----------------------------
if st.button("Resolver Ruteo"):
    data = create_data_model(
        distance_matrix,
        demands=demands if model_type in ["CVRP", "VRPTW"] else None,
        vehicle_capacities=capacities if model_type in ["CVRP", "VRPTW"] else None,
        time_windows=time_windows if model_type == "VRPTW" else None,
        cost_per_km=cost_per_km
    )

    solution, manager, routing = solve_vrp(
        data,
        use_capacity=(model_type in ["CVRP", "VRPTW"]),
        use_time_windows=(model_type == "VRPTW")
    )

    if not solution:
        st.error("No se encontró solución óptima para los datos ingresados.")
    else:
        routes, total_cost = get_routes(solution, manager, routing, data)
        st.success(f"Costo total de transporte: ${total_cost:.2f}")
        for i, (route, cost) in enumerate(routes):
            st.write(f"Vehículo {i}: Ruta: {route}, Costo: ${cost:.2f}")
