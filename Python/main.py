
from solution import *
from network import network_raw
import sys

if __name__ == "__main__":
    
    if sys.argv.__len__() != 3:
        print ("Error: requires initial and final supercharger names.")
        sys.exit(-1)

    departure = sys.argv[1]
    destination = sys.argv[2]

    solution = Solution(network_raw)
    if not departure in solution.graph.keys() or not destination in solution.graph.keys():
        print ("Error: initial or final supercharger names are incorrect.")
        sys.exit(-1)
        
    # stops = solution.a_star('Orlando_FL', 'Triadelphia_WV')
    solution.a_star(departure, destination)
    print (solution.path)
    print (solution.total_time)
