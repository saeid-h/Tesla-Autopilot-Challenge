#include "network.h"
#include <map>
#include <math.h>
// #include <queue>
#include <algorithm>

using namespace std;

#define EPS 1e-5
#define NET_SIZE 303
#define MAX_CHARG 320.
#define SPEED 105.
#define R_EARTH 6356.752

struct edge_weight {
        double travel_time;
        double rate;
    };
#define Graph map<string, map<string, edge_weight>>
#define Dict map<string,double>
#define Dict2 map<string,Dict>
#define RawGraph array<row, NET_SIZE>
// #define p_queue priority_queue<pair<double,string>, vector<pair<double,string>>, greater<pair<double,string>>>

#define degree_to_radian(x) (x/180.*3.141592653589793238463)


class Solution {
    // https://en.wikipedia.org/wiki/A*_search_algorithm
    public:
        Graph graph;
        vector<string> city_list;
        double total_time;

        Solution(RawGraph);                         // Default constructor
        Solution(RawGraph, double, double);         // Constructor by average speed and maximum distance travel
        string a_star(string, string);              // Find shortest path based on A* algorithm
        double get_battery_charge_at(string);       // Get battery charge at station
        double get_charging_time_beteen(string);    // Get calculated charging time between stations
        
    private:
        double speed = SPEED;                       // Km/hr
        double max_charge = MAX_CHARG / SPEED;      // Km
        double R = R_EARTH;                         // Km
        RawGraph network;                           // Input nettwork
        Dict battery;                               // Battery charge at any station
        Dict2 charging_time;                        // Charging time between stations

        void make_graph (RawGraph);                 // Convert the given graph to my graph
        double geo_distance(row, row);              // Find straight distance between two stations
        void optimize_charging(vector<string>);     // Optimize the charging time between stations based on the shortest path from A* algorithm
           
};

Solution::Solution(RawGraph net){
    network = net;
}
Solution::Solution(RawGraph net, double avg_speed, double max_distance){
    // Converges everything to time unit measurement
    speed = avg_speed;
    max_charge = avg_speed / max_distance;
    network = net;
}

double Solution::geo_distance(row x, row y){
    // http://www.jtrive.com/calculating-distance-between-geographic-coordinate-pairs.html
    double phi_x = degree_to_radian(x.lat);
    double lambda_x = degree_to_radian(x.lon);
    double phi_y = degree_to_radian(y.lat);
    double lambda_y = degree_to_radian(y.lon);

    double a1 = sin((phi_x-phi_y)/2) * sin((phi_x-phi_y)/2);
    double a2 = cos(phi_x) * cos(phi_y) * sin((lambda_x-lambda_y)/2.) * sin((lambda_x-lambda_y)/2.);
    double a = a1 + a2;

    return 2. * R * asin(sqrt(a));
}

void Solution::make_graph (RawGraph net){
    // Translates the given network to graph which is a map from stations to weights
    // All weights are transformed to time unit 
    for (int i=0; i < NET_SIZE; i++){
        city_list.push_back(net[i].name);
        for (int j=0; j < NET_SIZE; j++){
            double d = geo_distance(net[i], net[j]);
            graph[net[i].name][net[j].name] = {d/speed, net[i].rate/speed};
        }
    };
}

void Solution::optimize_charging(vector<string> stops){
    // Optimizes the charging time in each station for a givin path
    int n = stops.size();
    if (n < 3) return; 
    // Reset the calculated charging times.
    for (auto i=0; i < n-1; ++i){
        charging_time[stops[i]][stops[i+1]] = 0.;
    }
    // Check if station i charges faster than station i+1, 
    // charge as much as possible to get station i+2 without 
    // charging at station i+1.
    for (auto i=1; i < n-1; ++i){
        auto r_i = graph[stops[i]][stops[i]].rate;
        auto r_i_1 = graph[stops[i+1]][stops[i+1]].rate;
        auto d_i = graph[stops[i]][stops[i+1]].travel_time;

        auto t_new = max(0., d_i - battery[stops[i]]) / r_i + EPS;
        charging_time[stops[i]][stops[i+1]] += t_new;
        battery[stops[i+1]] = battery[stops[i]] + t_new * r_i - d_i;
        if (i+2 < n && r_i > r_i_1){
            t_new = min(max_charge-battery[stops[i+1]], max(0., graph[stops[i+1]][stops[i+2]].travel_time - battery[stops[i+1]])) / r_i + EPS;
            charging_time[stops[i]][stops[i+1]] += t_new;
            battery[stops[i+1]] += t_new * r_i;
            battery[stops[i+2]] = t_new * r_i;
        }
    }
}

string Solution::a_star(string start, string goal){
    // Famous A* algorith to find the shortest path between start and goal.
    // Heuristic is the straight line between stations.

    // Initialize graph and variables
    make_graph (network);
    map<string, string> comes_before;

    Dict travel_time;
    for (auto & city : city_list){travel_time[city] = INFINITY;}
    travel_time[start] = 0;

    Dict priorities; //////////////////////////
    for (auto & city : city_list){priorities[city] = INFINITY;}

    Dict heuristic;
    for (auto & city : city_list){heuristic[city] = graph[city][goal].travel_time;}
    priorities[start] = heuristic[start];

    Dict visited;
    for (auto & city : city_list){visited[city] = false;}

    for (auto & city : city_list){battery[city] = 0.;}
    battery[start] = max_charge;

    for (auto & city_1 : city_list){
        for (auto & city_2 : city_list){
            charging_time[city_1][city_2] = 0.;
        }
    }

    // Here is the main loop of the search
    while (true){
        double lowest_priority = INFINITY;
        string curren_city = "";

        // Find the next priority to explore
        for (auto & city : city_list){
            if (priorities[city] < lowest_priority && ! visited[city]){
                lowest_priority = priorities[city];
                curren_city = city;
            }
        }

        if (curren_city == ""){
            return "";                      // If nothing found --> There is no answer.
        } else if (curren_city == goal){    // Path was found. Extracts the path.
            auto next_stop = goal;
            vector<string> stops;
            stops.push_back(next_stop);
            while (next_stop != start){
                next_stop = comes_before[next_stop];
                stops.push_back(next_stop);
            }
            // Order from departure to destination and optimize the current shortest path.
            reverse(stops.begin(), stops.end());
            optimize_charging(stops);

            // Generate a string compy with the output format.
            string result = "";
            total_time = 0.;
            string pre_stop = "";
            for (auto stop: stops){
                if (pre_stop != "" && pre_stop != start){
                    result += to_string(charging_time[pre_stop][stop]) + ", ";
                }
                result += stop;
                if (stop != goal){
                    result += ", ";
                }
                if (pre_stop != ""){
                    total_time += graph[pre_stop][stop].travel_time + charging_time[pre_stop][stop];
                }
                pre_stop = stop;
            }
            
            return result;
        }
        
        // Main body of the A* algorithm
        // See https://en.wikipedia.org/wiki/A*_search_algorithm
        visited[curren_city] = true;
        for (auto & next_city : city_list){
            auto dist = graph[curren_city][next_city].travel_time;
            auto rate = graph[curren_city][next_city].rate;
            if (dist < max_charge && dist != 0 && !visited[next_city]){
                if (travel_time[curren_city] + dist < travel_time[next_city]){
                    if (graph[curren_city][goal].travel_time > battery[curren_city] && battery[curren_city] < dist){
                        charging_time[curren_city][next_city] = (dist - battery[curren_city]) / rate + EPS;
                    } else {
                        charging_time[curren_city][next_city] = 0.;
                    }
                    battery[next_city] = battery[curren_city] - dist + charging_time[curren_city][next_city] * rate;
                    if (travel_time[next_city] > travel_time[curren_city] + dist + charging_time[curren_city][next_city]){
                        travel_time[next_city] = travel_time[curren_city] + dist + charging_time[curren_city][next_city];
                        priorities[next_city] = travel_time[next_city] + heuristic[next_city];
                        comes_before[next_city] = curren_city;
                    }
                }
                visited[curren_city] = true;
            }
        }
    }
}
    

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        cout << "Error: requires initial and final supercharger names." << std::endl;        
        return -1;
    }
    
    string initial_charger_name = argv[1];
    string goal_charger_name = argv[2];
    bool initial_not_found = true;
    bool goal_not_found = true;
    for (auto &r : network){
       if (r.name == initial_charger_name) initial_not_found = false;
       if (r.name == goal_charger_name) goal_not_found = false;
    }
    if (initial_not_found || goal_not_found)
    {
        cout << "Error: initial and/or final supercharger not found." << std::endl;        
        return -1;
    }

    Solution solution(network);
    auto result = solution.a_star(initial_charger_name, goal_charger_name);
    cout << result << endl;
    // cout << solution.total_time << endl << endl;

    return 0;
}
