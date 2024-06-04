# This file contains all the required routines to make an A* search algorithm.
#
__author__ = '1718986'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2023 - 2024
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
    path_list = []
    expanded_node = path.last
    for key in map.connections[expanded_node]:
        temp_path = copy.deepcopy(path)
        temp_path.add_route(key)
        path_list.append(temp_path)
    
    return path_list


def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """
    new_path_list = []
    for path in path_list:
        to_add = True
        for i in range(len(path.route)-1):
            if path.route[i] == path.last:
                to_add = False
        if to_add == True:
            new_path_list.append(path)
    
    return new_path_list


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    list_of_path = expand_paths + list_of_path
    return list_of_path


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """
    path_list = [Path(origin_id)]
    while path_list and path_list[0].last != destination_id:
        curr_path = path_list[0]
        e = expand(curr_path,map)
        e = remove_cycles(e)
        path_list.pop(0)
        path_list = insert_depth_first_search(e,path_list)
    if path_list:
        return path_list[0]
    else:
        return []


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    list_of_path = list_of_path + expand_paths
    return list_of_path


def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    path_list = [Path(origin_id)]
    while path_list and path_list[0].last != destination_id:
        curr_path = path_list[0]
        e = expand(curr_path,map)
        e = remove_cycles(e)
        path_list.pop(0)
        path_list = insert_breadth_first_search(e,path_list)
    if path_list:
        return path_list[0]
    else:
        return []


def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """
    if type_preference == 0:
        for path in expand_paths:
            if map.connections[path.last][path.penultimate] > 0:
                path.update_g(1)
    elif type_preference == 1:
        for path in expand_paths:
            path.update_g(map.connections[path.last][path.penultimate])
    elif type_preference == 2:
        for path in expand_paths:
            if map.stations[path.last]["line"] == map.stations[path.penultimate]["line"]:
                path.update_g(map.connections[path.last][path.penultimate]*map.velocity[map.stations[path.last]["line"]])
    elif type_preference == 3:
        for path in expand_paths:
            if map.stations[path.last]["line"] != map.stations[path.penultimate]["line"]:
                path.update_g(1)
    
            
    
    return expand_paths


def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """
    list_of_path = list_of_path + expand_paths
    list_of_path = sorted(list_of_path, key = lambda path:path.g)

    return list_of_path


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    path_list = [Path(origin_id)]
    while path_list and path_list[0].last != destination_id:
        curr_path = path_list[0]
        e = expand(curr_path,map)
        e = remove_cycles(e)
        e = calculate_cost(e, map, type_preference)
        path_list.pop(0)
        path_list = insert_cost(e,path_list)
    if path_list:
        return path_list[0]
    else:
        return []


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            destination_id (int): Final station id
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """
    
    if type_preference == 0:
        for path in expand_paths:
            if  not destination_id in map.connections[path.last] and destination_id != path.last:
                path.update_h(1)
            elif destination_id == path.last:
                path.update_h(0)
        
    elif type_preference == 1:
        max_vel = 0
        for vel in map.velocity:
            if map.velocity[vel] > max_vel:
                max_vel = map.velocity[vel]
        for path in expand_paths:
            elem1 = [map.stations[path.last]['x'],map.stations[path.last]['y']]
            elem2 = [map.stations[destination_id]['x'],map.stations[destination_id]['y']]
            dist = euclidean_dist(elem1,elem2)
            path.update_h(dist/max_vel)
        
    elif type_preference == 2:
        for path in expand_paths:
            elem1 = [map.stations[path.last]['x'],map.stations[path.last]['y']]
            elem2 = [map.stations[destination_id]['x'],map.stations[destination_id]['y']]
            dist = euclidean_dist(elem1,elem2)
            path.update_h(dist)
    
    elif type_preference == 3:
        for path in expand_paths:
            if map.stations[path.last]["line"] != map.stations[destination_id]["line"]:
                path.update_h(1)
            elif path.last == destination_id:
                path.update_h(0)
        
    return expand_paths

            
def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    for path in expand_paths:
        path.update_f()
    
    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g-cost at this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
             visited_stations_cost (dict): Updated visited stations cost
    """
    t_expand_paths = copy.deepcopy(expand_paths)
    for path in expand_paths:
        if path.last not in visited_stations_cost:
            visited_stations_cost[path.last] = path.g
        
        else:
            if visited_stations_cost[path.last] <= path.g:
                for t_path in t_expand_paths:
                    if t_path.route == path.route:
                        t_expand_paths.remove(t_path)
                        break
            else:
                visited_stations_cost[path.last] = path.g
                for l_path in list_of_path:
                    if path.last in l_path.route:
                        list_of_path.remove(l_path)

    expand_paths = t_expand_paths
    return expand_paths, list_of_path, visited_stations_cost

def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    list_of_path = expand_paths+list_of_path
    list_of_path = sorted(list_of_path, key = lambda path:path.f)
    
    return list_of_path

def distance_to_stations(coord, map):
    """
        From coordinates, it computes the distance to all stations in map.
        Format of the parameter is:
        Args:
            coord (list): Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            (dict): Dictionary containing as keys, all the Indexes of all the stations in the map, and as values, the
            distance between each station and the coord point
    """
    result = {}
    for station_id in map.stations:
        x = map.stations[station_id]['x']
        y = map.stations[station_id]['y']
        station_coords = [x,y]
        distance = euclidean_dist(coord,station_coords)
        result[station_id] = distance
    
    sorted_result = sorted(result.items(), key=lambda x: (x[1], x[0]))

    sorted_result = dict(sorted_result) 

    return sorted_result


def Astar(origin_id, destination_id, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    visited_stations = {}
    path_list = [Path(origin_id)]
    while path_list and path_list[0].last != destination_id:
        curr_path = path_list[0]
        e = expand(curr_path,map)
        e = remove_cycles(e)
        e = calculate_cost(e, map, type_preference)
        e, path_list, visited_stations = remove_redundant_paths(e,path_list, visited_stations)
        e = calculate_heuristics(e, map, destination_id, type_preference)
        e = update_f(e)
        path_list.pop(0)
        path_list = insert_cost_f(e,path_list)
    if path_list:
        return path_list[0]
    else:
        return []


def Astar_improved(origin_coord, destination_coord, map):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_coord (list): Two REAL values, which refer to the coordinates of the starting position
            destination_coord (list): Two REAL values, which refer to the coordinates of the final position
            map (object of Map class): All the map information

        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_coord to destination_coord
    """
    user_velocity = 5
    origin_to_stations = distance_to_stations(origin_coord,map)
    destination_to_stations = distance_to_stations(destination_coord,map)
    #for time
    for key in origin_to_stations:
        origin_to_stations[key] = origin_to_stations[key] / user_velocity
    
    for key in destination_to_stations:
        destination_to_stations[key] = destination_to_stations[key] / user_velocity

    #update map given the user coordinates
    map.connections[0] = origin_to_stations
    map.connections[-1] = destination_to_stations
    map.connections[0][-1] = map.connections[-1][0] = euclidean_dist(origin_coord,destination_coord) / user_velocity
    map.stations[0] = {'name':"init_pos",'line':0,'x':origin_coord[0],'y':origin_coord[1]}
    map.stations[-1] = {'name':"dest_pos",'line':-1,'x':destination_coord[0],'y':destination_coord[1]}
    map.velocity[-1] = user_velocity
    for station_id in map.connections:
        if station_id!= 0 and station_id!=-1:
            map.connections[station_id][0] = origin_to_stations[station_id]
            map.connections[station_id][-1] = destination_to_stations[station_id]
    
    #apply A* given the user coordinates
    path_A = Astar(0,-1,map,1)

    return path_A
    

