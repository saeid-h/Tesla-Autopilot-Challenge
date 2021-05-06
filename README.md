## Introduction
This is a solution for a challeng from a Tesla Autopilot team under Dr. Andrej Karpathy leadership. The next section is problem statement and the following part is my solution for this problem.

## Problem Statement
Your objective is to construct a search algorithm to find the minimum time path through the tesla network of supercharging stations. Each supercharger will refuel the vehicle at a different rate given in km/hr of charge time. Your route does not have to fully charge at every visited charger, so long as it never runs out of charge between two chargers. You will be provided with a code skeleton which includes a header with the charger network data in the format:

```name, latitude in degrees, longitude in degrees, charge rate in km/hr```

### Input
Your program should take as input two strings: `start charger name`, `end charger name`.

### Output 
Your program’s only output should be a print to `std::out` of a string in the format:

```initial charger name, first charger name, charge time in hrs, second charger name, charge time in hrs, …, …, goal charger name```

This is the format required by the checker program as well, for example the command:

```bash
./solution Council_Bluffs_IA Cadillac_MI
``` 

might return:

```bash
Council_Bluffs_IA, Worthington_MN, 1.18646, Albert_Lea_MN, 1.90293, Onalaska_WI, 
0.69868, Mauston_WI, 1.34287, Sheboygan_WI, 1.69072, Cadillac_MI
```

### Evaluation	
You can check the solution by providing your output to the included checker, for example
		
```bash
./checker_osx “Council_Bluffs_IA, Worthington_MN, 1.18646, Albert_Lea_MN, 1.90293, 
Onalaska_WI, 0.69868, Mauston_WI, 1.34287, Sheboygan_WI, 1.69072, Cadillac_MI”
```

will return 

```bash		
Finding Path Between Council_Bluffs_IA and Cadillac_MI
Reference result: Success, cost was 17.2531
Candidate result: Success, cost was 17.2548
```

### Constrains
You should ensure that your submission compiles under `gcc 4.8.4` with optimization level 1, for example:

```bash 
g++ -std=c++11 -O1 main.cpp network.cpp -o candidate_solution
```

The solution needs to be self-contained with just the use of STL algorithms (i.e. do not use off-the-shelf packages). 

## Solution
The solution is A* algorithm and a one pass optimization throgh the shortest path.

### Assumptions
Assumptions from the problem statement:
* The car begins at the start charger with a full charge of `320 Km`
* The car travels at a constant speed of `105 Km/hr` along great circle routes between chargers
* The Earth is a sphere of radius `6356.752 Km`

My assumption:
* There are routes available between all the pairs of the stations throgh a straight line,
* A car cannot reach beyond the maximum capacity of its battery.
* We charge the car enough to get to the next stations.
* As a result of the previous assumption we get the next station with zero charge or maximum charge minus the distance between two stations.
* To avoid a numerical error, I added an epsilon value extra charge in charging station.

### Algorithm
I used the famous A* algorithm to find the shortest path. The heuristic is the streight line distance between the current station and the next candidate stations. The search will be performed around the current stations neighbors. A neighbor is a station that is located within a reachable distance imposed by battery capacity. After finding the shortest path, I optimized the cahrging times at stations in one pass check on the path if it's better to charge in station i or i+1.

### Optimization
After finding the short path based on the A* algorithm, there is chance to get better results by leveling the charging in each station. 
* Check if the charger in station i is faster than charger in station i+1,
* Check if station i+1 is not the final destination,
* If both conditions are satisfied,
* Add more charge to get the station i+2 if we have enough battery capacity, or
* Full charge at station i.

## Reults
My result for the given exaple is:
```bash
Council_Bluffs_IA, Worthington_MN, 2.80748, Albert_Lea_MN, 0.00000, Onalaska_WI, 0.69868, Mauston_WI, 2.76405, Sheboygan_WI, 0.00000, Cadillac_MI
```
And here is the comparison with the reference solution:
```bash
Finding Path Between Council_Bluffs_IA and Cadillac_MI
Reference result: Success, cost was 17.2531
Candidate result: Success, cost was 16.7033
```

## References
[A* algorithm](https://www.algorithms-and-technologies.com/a_star/python)
[A* algorithm - Python](https://www.algorithms-and-technologies.com/a_star/python)
[Geo Distance Calculation](http://www.jtrive.com/calculating-distance-between-geographic-coordinate-pairs.html)
