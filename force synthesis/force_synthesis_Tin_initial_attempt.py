import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Force_unit_controller:

    def __init__(self):

        #define the goal external force
        self.goal_force = np.array(([1], [0], [0])) #column vector (3,1)
        print("For debug - goal_force = ", self.goal_force, np.shape(self.goal_force))

        #define the location of the end effector
        self.end_eff_location = np.array(([0],[0],[0])) #column (3,1)
        print("For debug - end_eff_location = ", self.end_eff_location, np.shape(self.end_eff_location))

        #define the location of each force unit
        self.force_unit_locations = np.zeros((6,3,1))
        self.force_unit_locations[0] = np.array(([1], [0], [0])) #column vector (3,1)
        self.force_unit_locations[1] = np.array(([0], [1], [0])) #column vector (3,1)
        self.force_unit_locations[2] = np.array(([0], [0], [1])) #column vector (3,1)
        self.force_unit_locations[3] = np.array(([-1], [0], [0])) #column vector (3,1)
        self.force_unit_locations[4] = np.array(([0], [-1], [0])) #column vector (3,1)
        self.force_unit_locations[5] = np.array(([0], [0], [-1])) #column vector (3,1)
        print("For debug - force_unit_location = ", self.force_unit_locations, np.shape(self.force_unit_locations))

        #calculate the unit vector of each force unit 
        self.force_unit_vectors = np.zeros((6,3,1))
        

#synthe size the goal force using all 6 unit vectors
    def update_force_unit_vectors(self):
        for i in range(len(self.force_unit_vectors)):
            self.force_unit_vectors[i] = self.calculate_unit_vector(self.end_eff_location, self.force_unit_locations[i])#column vector (3,1)
        print("for debug - force_unit_vectors = ", self.force_unit_vectors)
   
    def calculate_unit_vector(self, start_point, end_point):
        v = end_point - start_point
        v_norm = np.linalg.norm(v)
        u = v/v_norm

        print(u)
        return u


    def visualize(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # plot end effector location as a red dot
        ax.scatter(self.end_eff_location[0][0],self.end_eff_location[1][0],self.end_eff_location[2][0], color='red')
        
        # plot goal force as a green arrow
        ax.arrow(self.end_eff_location[0][0], self.end_eff_location[1][0], self.end_eff_location[2][0],self.goal_force[0][0], self.goal_force[1][0], self.goal_force[2][0], head_width=0.05, head_length=0.1, fc='green', ec='green')
        
        # # plot force unit locations as red dots and arrows connecting them to end effector location
        # for i in range(len(self.force_unit_locations)):
        #     ax.scatter(self.force_unit_locations[i][0], self.force_unit_locations[i][1], color='red')
        #     ax.arrow(self.end_eff_location[0], self.end_eff_location[1], self.force_unit_vectors[i][0], self.force_unit_vectors[i][1], head_width=0.05, head_length=0.1, fc='black', ec='black')
        
        # set axes limits and show plot
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([-1.5, 1.5])
        plt.show()
        
if __name__ == "__main__":
    controller = Force_unit_controller()

    # for i in range(3):
    # while True:
        # controller.update_force_unit_vectors()
    controller.update_force_unit_vectors()
    controller.visualize()

