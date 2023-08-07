import pygame
import environment 
import graph
import argparse
import sys

# Command line arguments
parser = argparse.ArgumentParser(description='Implements the Visibility PRM algorithm for \
	path planning.')
parser.add_argument('-o', '--obstacles', type=bool, action=argparse.BooleanOptionalAction,
	metavar='', required=False, help='Obstacles on the map')
parser.add_argument('-init', '--x_init', nargs='+', type=int, metavar='', required=False,
	help='Initial node position in X and Y respectively')
parser.add_argument('-goal', '--x_goal', nargs='+', type=int, metavar='', required=False,
	help='Goal node position in X and Y respectively')
parser.add_argument('-srn', '--show_random_nodes', type=bool, action=argparse.BooleanOptionalAction,
	metavar='', required=False, help='Show random nodes on screen')
parser.add_argument('-srjn', '--show_rejected_nodes', type=bool,
	action=argparse.BooleanOptionalAction, metavar='', required=False,
	help='Show rejected nodes on screen')
parser.add_argument('-sen', '--show_enumerated_nodes', type=bool, 
	action=argparse.BooleanOptionalAction, metavar='', required=False, 
	help='Show the node enumerated in the order it was sampled')
parser.add_argument('-sve', '--show_volume_estimation', type=bool, 
	action=argparse.BooleanOptionalAction, metavar='', required=False, default=True,
	help='Show the node enumerated in the order it was sampled')
parser.add_argument('-M', '--M', type=int, metavar='', required=False, default=10,
	help='Maximum number of failures before allowed before inserting a new guard node into the \
	roadmap ')
parser.add_argument('-r', '--radius', type=int, metavar='', required=False, default=10,
	help='Set the robot radius')
args = parser.parse_args()

# Initialization 
pygame.init()

# Constants
MAP_DIMENSIONS = 640, 480

# Initial and final position of the robot
x_init = tuple(args.x_init) if args.x_init is not None else (50, 50)
x_goal = tuple(args.x_goal) if args.x_goal is not None else (540, 380)

# Instantiating the environment and the graph
environment_ = environment.Environment(map_dimensions=MAP_DIMENSIONS)
graph_ = graph.Graph(start=x_init, goal=x_goal, map_dimensions=MAP_DIMENSIONS, radius=args.radius)

def main():
	run = True
	clock = pygame.time.Clock()
	guards = []
	connections = []
	configurations = []
	initial = graph_.draw_initial_node(map_=environment_.map)
	goal = graph_.draw_goal_node(map_=environment_.map)
	configurations.append(initial)
	configurations.append(goal)
	environment_.make_obstacles()
	obstacles = environment_.draw_obstacles() if args.obstacles else []
	graph_.obstacles = obstacles
	# Number of failures before the insertion of a new guard node
	ntry = 0
	node_number = 0

	repeated_guards = []
	is_found_repeated = False

	len_guards = len(guards)
	len_connections = len(connections)

	connectors = 0

	# To keep track and later merge the hanging guardian nodes
	hanging_guardians = []
	temp_dictionary = {}
	is_track_finished = False # Avoid overflow of the dictionary

	while run:
		clock.tick(environment_.FPS) 
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		obstacles = environment_.draw_obstacles() if args.obstacles else []
		
		while ntry < args.M:
			# Select a random free configuration
			x_rand = graph_.generate_random_node()
			collision_free = graph_.is_free(point=x_rand, obstacles=obstacles)
			is_connected = False

			if collision_free:
				if args.show_random_nodes:
					graph_.draw_random_node(map_=environment_.map)

				if len(guards) == 0:
					guards.append(x_rand)
					graph_.draw_guard_node(map_=environment_.map, position=x_rand.center)

				elif len(guards) == 1:
					for i in range(len(guards)):
						cross_obstacle1 = graph_.cross_obstacle(configuration1=x_rand,
							configuration2=guards[i], map_=environment_.map)

						if cross_obstacle1:
							guards.append(x_rand)
							graph_.draw_guard_node(map_=environment_.map, position=x_rand.center)
						else:
							if not is_connected and args.show_rejected_nodes:
								graph_.draw_rejected_node(map_=environment_.map,
									position=x_rand.center)

				for i in range(len(guards)):
					rejection = 0
					cross_obstacle1 = graph_.cross_obstacle(configuration1=x_rand, 
						configuration2=guards[i], map_=environment_.map)

					if not cross_obstacle1: # Connected to the first component
						for j in range(i+1, len(guards)):
							if x_rand == guards[j]:
								break # Prevents attempting to connect the node with the same node
							cross_obstacle2 = graph_.cross_obstacle(configuration1=x_rand, 
								configuration2=guards[j], map_=environment_.map)

							if not cross_obstacle2: # Connected to the second component
								# Avoid repeated connections once two guards have already 
								# been connected
								for guard in repeated_guards:
									if guards[i] == guard[0] and guards[j] == guard[1]:
										is_found_repeated = True
								
								if is_found_repeated:
									if args.show_rejected_nodes:
										graph_.draw_rejected_node(map_=environment_.map,
											position=x_rand.center)
									is_found_repeated = False
									continue

								repeated_guards.append([guards[i], guards[j]])
								is_connected = True
								connections.append(x_rand)

								# Add the neighbors of the connector node
								graph_.neighbors.update({x_rand.center: [guards[i].center, guards[j].center]})

								# Keep track of the guardians neighbors
								hanging_guardians.append([guards[i].center, x_rand.center])
								hanging_guardians.append([guards[j].center, x_rand.center])

								graph_.draw_connection_node(map_=environment_.map, 
									position=x_rand.center)
								graph_.draw_local_planner(p1=x_rand, p2=guards[i], 
									map_=environment_.map)
								graph_.draw_local_planner(p1=x_rand, p2=guards[j], 
									map_=environment_.map)
							else:
								if not is_connected and args.show_rejected_nodes:
									graph_.draw_rejected_node(map_=environment_.map, 
										position=x_rand.center)

					if cross_obstacle1:
						rejection += 1
						ntry += 1
						for j in range(i+1, len(guards)):
							if x_rand == guards[j]:
								break # Prevents attempting to connect the node with the same node
							cross_obstacle2 = graph_.cross_obstacle(configuration1=x_rand, 
								configuration2=guards[j], map_=environment_.map)

							if cross_obstacle2:
								rejection += 1
								ntry += 1

								# All the attempts to connect to guards were unsuccessful, therefore
								# the node is a guard node
								if rejection == len(guards):
									ntry = 0
									guards.append(x_rand)
									graph_.draw_guard_node(map_=environment_.map, 
										position=x_rand.center)								
							else:
								rejection = 0
								if not is_connected and args.show_rejected_nodes:
									graph_.draw_rejected_node(map_=environment_.map, 
										position=x_rand.center)

				node_number += 1
				pygame.display.update()
				is_change_made = len(guards) > len_guards or len(connections) > len_connections

				if args.show_enumerated_nodes:
					if not args.show_rejected_nodes and is_change_made:
						len_guards = len(guards)
						len_connections = len(connections)
						environment_.draw_node_number(number=node_number, point=x_rand.center)
					elif args.show_rejected_nodes:
						environment_.draw_node_number(number=node_number, point=x_rand.center)

		# Merge the guardians and their neighbors in a dictionary 
		if not is_track_finished:
			for guard in hanging_guardians:
			    key = guard[0]
			    value = guard[1:]
			    if key in temp_dictionary:
			        temp_dictionary[key].extend(value)
			    else:
			        temp_dictionary[key] = value
			is_track_finished = True

		if args.show_volume_estimation:	
			print(f'Estimated volume not yet covered by visibility domains {100*(1/ntry):.4f}%')
			print(f'Estimated volume covered by visibility domains {100*(1-1/ntry):.4f}%')
			args.show_volume_estimation = False

		# Merge guard and connection nodes, and query the initial and goal configurations
		configurations += guards + connections
		graph_.neighbors.update(temp_dictionary)
		graph_.query(init=initial, goal=goal, configurations=configurations, map_=environment_.map)

		a_star = (graph_.a_star(nodes=configurations, map_=environment_.map))
		graph_.draw_path_to_goal(map_=environment_.map, environment=environment_,
				 obstacles=obstacles)
		pygame.display.update()

	pygame.quit()
	sys.exit()

if __name__ == '__main__':
	main()