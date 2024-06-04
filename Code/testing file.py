from SearchAlgorithm import *
from SubwayMap import *
from utils import *

def print_list_of_path_with_heu(path_list):
    for p in path_list:
        print("Route: {}, \t Cost: {}".format(p.route, round(p.h,2)))

if __name__=="__main__":
    ROOT_FOLDER = '../CityInformation/Lyon_SmallCity/'
    map = read_station_information(os.path.join(ROOT_FOLDER, 'Stations.txt'))
    connections = read_cost_table(os.path.join(ROOT_FOLDER, 'Time.txt'))
    map.add_connection(connections)

    infoVelocity_clean = read_information(os.path.join(ROOT_FOLDER, 'InfoVelocity.txt'))
    map.add_velocity(infoVelocity_clean)



    ###BELOW HERE WE CAN CALL ANY FUNCTION PROGRAMED IN ORDER TO TEST IT###

    #example
    expanded_paths = [Path([10,7,6]),Path([10,7,8])]
    updated_paths = calculate_heuristics(expanded_paths,map,destination_id=9,type_preference=1)
    updated_paths = update_f(updated_paths)
    print_list_of_path_with_heu(updated_paths)


