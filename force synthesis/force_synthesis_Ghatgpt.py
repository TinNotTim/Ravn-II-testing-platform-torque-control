import numpy as np
from scipy.optimize import minimize

# Define the objective function 
def objective(lengths):
    return np.linalg.norm(np.dot(vectors.T, lengths) - desired_vector)

# Define the constraints 
def constraint(lengths):
    return np.array([min_lengths - lengths, lengths - max_lengths]).flatten()

# Define the desired 3D vector and the direction of the 6 vectors 
desired_vector = np.array([1, 2, 3])
vector_directions = np.array([
    [1, 0, 0], 
    [0, 1, 0], 
    [0, 0, 1], 
    [-1, 0, 0], 
    [0, -1, 0], 
    [0, 0, -1]
])

# Define the ranges for the lengths of the vectors 
min_lengths = np.array([0.1] * 6)
max_lengths = np.array([10] * 6)

# Normalize the vector directions 
vector_directions = vector_directions / np.linalg.norm(vector_directions, axis=1)[:, np.newaxis]

# Solve the optimization problem 
vectors = vector_directions
initial_guess = np.ones(6)
bounds = [(min_lengths[i], max_lengths[i]) for i in range(6)]
solution = minimize(objective, initial_guess, bounds=bounds, constraints={'type': 'ineq', 'fun': constraint})

# Get the lengths of the vectors 
lengths = solution.x 

# Calculate the actual vectors 
actual_vectors = np.dot(vectors.T, lengths) 

# Print the solution 
print("Lengths: ", lengths)
print("Actual Vectors: ", actual_vectors)
