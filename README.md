Drone Path Simulator:
Dijkstra-based flight path planner modelling restricted airspace, wind corridors, and IMC regions with live telemetry output and mission efficiency metrics. Built in C, terminal only, no external dependencies.

What does the program do:
Simulates a drone navigation, autonomously, across a 20x20 grid. The drone finds the most sustainable path parallel to energy efficiency from a user defined launch point and waypoint.
The drone follows a route that does not clash with no fly zones and takes wind forces into account.
There is a legend printed in the output that explains how to read the airspace grid, gives a telemetry reading, and mission statistics.

Algorithm:
The program uses Dijkstra algorithm - weighted shortest path where it finds the most energy optimal route. Various routes are compared using BFS. 

Using:
It uses gcc, and no external libraries. The user will be able to state coordinates when the program launches, and the algorithm will then give out the results.
Flight speed, airspace map, and certain other factors have been hardcoded into the program to make it convinient for the user to work around.
In later phases, more fluidity can be provided and other factors could also be calculated, relating tto the drone's flight path.
