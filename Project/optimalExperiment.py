import random
import os
from deap import base, creator, tools
import pandas as pd
from Exercise_3 import SimpleRoadModel
from pypdevs.simulator import Simulator

# Constants for the Genetic Algorithm and Traffic Light Settings
POPULATION_SIZE = 5  # Number of individuals in the population
GENERATIONS = 10  # Number of generations for the genetic algorithm
NUM_TRAFFIC_LIGHTS = 6  # Number of traffic lights per individual
GREEN_RANGE = (30, 180)  # Range for green light duration (in seconds)
YELLOW_RANGE = (10, 30)  # Range for yellow light duration (in seconds)
RED_RANGE = (30, 180)  # Range for red light duration (in seconds)

# DEAP setup: Defining the optimization problem
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))  # Minimize the fitness value (travel time)
creator.create("Individual", list, fitness=creator.FitnessMin)  # An individual is a list of traffic light timings

toolbox = base.Toolbox()

# Genetic representation: A function to generate random traffic light timings
def generate_traffic_light():
    return [
        random.randint(*GREEN_RANGE),  # Green light duration
        random.randint(*YELLOW_RANGE),  # Yellow light duration
        random.randint(*RED_RANGE),  # Red light duration
    ]

# Registering the individual and population creation functions
toolbox.register("attr_traffic_light", generate_traffic_light)
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_traffic_light, n=NUM_TRAFFIC_LIGHTS)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# Simulation evaluation: Evaluating the fitness of an individual
def evaluate(individual):
    """
    Simulate the traffic system with the given timings and calculate the average travel time. Might want to add some more parameters to optimize -christian
    """
    timings = individual  # List of traffic light timings (green, yellow, red)

    # Ensure all traffic lights have the correct structure
    for idx, light in enumerate(timings):
        if len(light) != 3:
            raise ValueError(f"Traffic light at index {idx} has invalid structure: {light}")

    # Create the traffic system with the given timings; remember to update the number of traffic lights both here and in the coupled model script -christian
    trafficSystem = SimpleRoadModel(
        time_next_car=300,  # Interval for new cars entering the system (in seconds)
        output_file='sim-out.csv',  # Output file for simulation results
        light_timings=timings,  # Traffic light timings
        num_traffic_lights=len(timings),  # Number of traffic lights
    )

    # Run the simulation
    sim = Simulator(trafficSystem)
    sim.setTerminationTime(21600.0)  # Simulate for x hours (in seconds); I usually run simulations for 12 hours, but it takes time -christian
    sim.setClassicDEVS()  # Use classic DEVS simulation mode
    sim.simulate()

    # Calculate the average travel time from the simulation output
    df = pd.read_csv('sim-out.csv', names=['Id', 'CT','Exit Time', 'Distance', 'Travel Time', 'local', 'destination'])
    
    avg_travel_time = df["Travel Time"].mean()  # Average travel time of vehicles
    return avg_travel_time,  # Return as a tuple for compatibility with DEAP

toolbox.register("evaluate", evaluate)

# Crossover: Two-point crossover with safety checks
def safe_cx_two_point(ind1, ind2):
    """
    Perform two-point crossover only if both individuals have more than one traffic light. I added the checks after getting some errors on the indexing of ind -christian
    """
    if len(ind1) > 1 and len(ind2) > 1:
        tools.cxTwoPoint(ind1, ind2)
    return ind1, ind2

# Mutation: Modify one parameter (green, yellow, or red) for a random traffic light
def mutate_single_light(ind):
    """
    Mutates a single parameter (green, yellow, or red) for one traffic light in the individual.
    """
    num_lights = len(ind)  # Number of traffic lights

    if num_lights == 0:
        raise ValueError("The individual has no traffic lights to mutate.")

    # Choose a random traffic light index
    light_idx = random.randint(0, num_lights - 1)

    # Choose a random parameter to mutate (0: green, 1: yellow, 2: red)
    param = random.randint(0, 2)

    # Ensure the traffic light structure is valid
    if len(ind[light_idx]) != 3:
        raise ValueError(f"Traffic light at index {light_idx} has invalid structure: {ind[light_idx]}")

    # Mutate the chosen parameter within its range
    if param == 0:  # Green
        ind[light_idx][param] = random.randint(*GREEN_RANGE)
    elif param == 1:  # Yellow
        ind[light_idx][param] = random.randint(*YELLOW_RANGE)
    elif param == 2:  # Red
        ind[light_idx][param] = random.randint(*RED_RANGE)

    return ind

toolbox.register("mate", safe_cx_two_point)
toolbox.register("mutate", mutate_single_light)
toolbox.register("select", tools.selTournament, tournsize=3)  # Tournament selection

# Genetic Algorithm
def run_ga():
    """
    Run the genetic algorithm to optimize traffic light timings.
    """
    # Initialize the population
    population = toolbox.population(n=POPULATION_SIZE)

    for gen in range(GENERATIONS):
        # Clone the population to create offspring
        offspring = list(map(toolbox.clone, population))

        # Apply crossover
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < 0.7:  # Crossover probability
                toolbox.mate(child1, child2)

        # Apply mutation
        for mutant in offspring:
            if random.random() < 0.2:  # Mutation probability
                toolbox.mutate(mutant)

        # Evaluate fitness of invalid individuals
        for ind in offspring:
            if not ind.fitness.valid:
                ind.fitness.values = toolbox.evaluate(ind)

        # Select the next generation
        population = toolbox.select(offspring, k=len(population))

    # Get the best individual
    best_ind = tools.selBest(population, k=1)[0]

    print("Best individual:", best_ind)
    print("Best fitness:", best_ind.fitness.values[0])

    # Save the results
    save_results(best_ind, best_ind.fitness.values[0]) #autosave useful for batch simulations

    return best_ind, best_ind.fitness.values[0]

# Save results to a CSV file
def save_results(best_individual, best_fitness):
    """
    Save the best individual and its fitness value to a CSV file.
    """
    data = {
        'Best Individual': [best_individual],
        'Best Fitness': [best_fitness]
    }
    df = pd.DataFrame(data)

    output_file = 'best_results.csv'

    if os.path.exists(output_file):  # Append if file exists
        df.to_csv(output_file, mode='a', header=False, index=False)
    else:  # Create new file if it doesn't exist
        df.to_csv(output_file, mode='w', header=True, index=False)

# Main execution
if __name__ == "__main__":
    num_simulations = 1  # Number of times to run the simulation
    #sometimes the simulation fails and there's a DEVS exception, for example:
    #DEVS Exception: Negative time advance in atomic model 'system.sectionL3_1' with value -0.3428570000001855 at time 5441.6
    #the negative time advance always happens to one of the atomic models near the middle interception (L3_1, L3_2, R3_1,R3_2)
    # -christian
    for i in range(num_simulations):
        print(f"Running simulation {i + 1} of {num_simulations}...")
        best_solution = run_ga()
        print(f"Simulation {i + 1} completed.")
