# This class is at the intermediate level where it takes in the robot
# module class and uses the compatibility rules from the previous
# class to constructs meaningful robots, this also does all the
# housekeeping of creating multiple instances of the robot environment
# i.e., creating branches and populating the correct branch

# Also contains problem level objects to define the cost and
# compute optimal design in the given expanded branch

import copy
from pydrake.all import *

"""
**LCL-Robots** is a project for the automatic generation of task-specific robots using
user specified modules. The current library, `design-on-demand (DoD)` provides the enables
the automatic design process of robots provided the modules and the required task.
"""

class robotNode:
    """
    This class is at the intermediate level where it takes in the robot
    module class and uses the compatibility rules from the previous
    class to constructs meaningful robots, this also does all the
    housekeeping of creating multiple instances of the robot environment
    i.e., creating branches and populating the correct branch

    :param roboDoD: An instance of the class robotDoD
    """
    def __init__(self, roboDoD):
        self.depth = 0
        # Dont use self.base_node to prevent editing it by mistake
        self.parent_node = roboDoD
        # Also save the current_parent_node for all the references
        self.current_parent_node = roboDoD
        self.current_parent_node_path = ''
        self.num_modules = len(roboDoD.declared_modules)
        # Directory to save the constructed URDFs
        self.node_path = './node_urdfs/'

    # ToDo: THE CURRENT ROBOT IS NOT BEING UPDATED WITH THE LATEST MODULES!!!

    def get_compatible_modules(self):
        """
        Returns a list of compatible modules for the last module in the kinematic tree
        """
        # Get the type of the last node
        if not self.parent_node.tree:
            raise AssertionError("Error: No modules found in base node")
        last_module_type = self.current_parent_node.tree[list(self.current_parent_node.tree)[-1]][0].type

        # Identify all compatible nodes
        # self.compatible_modules = []
        compatible_modules = []
        for module in self.parent_node.declared_modules:
            if last_module_type in module.parent_type:
                compatible_modules.append(module)
        return compatible_modules

    # Create multiple instances of the parent node
    # If num_instances is not specified then create
    # as many as the number of modules
    def create_instances(self, num_instances=None):
        """
        Returns instances of the a set of same class objects that can be used to expand
        the robot architecture with different modules in each one of them

        :param num_instances: None, number of instances to be created
        """
        if num_instances == None:
            num_instances = self.num_modules
        node_instances = []
        for i in range(num_instances):
            # Makes instances of whatever is present in the parent node
            node_instance = copy.deepcopy(self.current_parent_node)
            module_name = 'node'+str(self.depth)+str(i)
            # Change instance name
            node_instance.name = module_name
            # Change URDF name
            node_instance.robot.name = module_name
            node_instances.append(node_instance)
        return node_instances


    # Expand corresponding nodes of robots based on compatible modules
    def expand_node(self, action=None):
        """
        Returns number of instances of the parent node equal to the number of
        compatible modules
        """
        # if specific module is not demanded (like in A*)
        if action == None:
            # Get compatible nodes first
            compatible_modules = self.get_compatible_modules()
            self.num_child_nodes = len(compatible_modules)
        else:
            compatible_modules = [action]
            self.num_child_nodes = 1

        # Now create required number of instances
        node_instances = self.create_instances(self.num_child_nodes)
        # Add each module to each new instance
        for i in range(self.num_child_nodes):
            node_instances[i].add_module(compatible_modules[i])

        self.child_nodes = node_instances
        # Increase the depth of the tree
        self.depth += 1



    # compute path costs based on modules selected
    def compute_node_mass(self, node):
        """
        Returns the mass of the selected node, i.e., the sum of masses of
        all associated modules

        :param node: The selected node
        """
        mod_list = list(node.tree.keys())
        mass = 0
        for mod in mod_list:
            mass+=node.tree[mod][0].mass
        return mass

    # compute path costs based on modules selected
    def compute_path_cost(self, node):
        """
        Returns the path cost of the selected node, i.e., the cost associated
        with the selected modules

        :param node: The selected node
        """
        mod_list = list(node.tree.keys())
        path_cost = 0
        for mod in mod_list:
            path_cost+=node.tree[mod][0].cost
        return path_cost

    # This function is from v0.2.0 and is depricated in v0.3.0
    #     # Use this for the combined cost in A*
    # def get_node_cost(self, compute_heuristic_cost, node_instance, node_path):
    #     g_cost = self.compute_path_cost(node_instance)
    #     # h_cost, end_flag = self.compute_heuristic_cost(node_path, node_instance.name)
    #     # Making the heuristic function external
    #     h_cost, end_flag = compute_heuristic_cost(node_path, node_instance.name)
    #     f_cost = g_cost+h_cost
    #     return f_cost, end_flag

    # Use this for the combined cost in A*
    def get_node_cost(self, compute_heuristic_cost=None, node_instance=None, gh_weighting=[1,1]):
        g_cost = self.compute_path_cost(node_instance)
        # Making the heuristic function external
        h_cost, end_flag = compute_heuristic_cost(node_instance = node_instance)
        f_cost = g_cost*gh_weighting[0]+h_cost*gh_weighting[1]
        print(g_cost, h_cost)
        return f_cost, end_flag

        # Use this for requirements based depth first expansion
    def get_node_costs(self, compute_heuristic_cost, node_instance, node_path):
        # cost associated with the modules or N_realised
        g_cost = self.compute_path_cost(node_instance)
        # h_cost, end_flag = self.compute_heuristic_cost(node_path, node_instance.name)
        # Making the heuristic function external
        h_cost, res = compute_heuristic_cost(node_path, node_instance.name)
        # f_cost = g_cost+100*h_cost
        return g_cost, h_cost, res

    # TO BE DEPRECIATED SOON!!!-> REPLACED BY GET_BEST_CHILD
    def get_best_child_index(self):
        """
        Returns the object and the index of the best child node based on the cost function defined
        """
        self.children_cost = []
        for ii in range(self.num_child_nodes):
            self.children_cost.append(
#                 self.compute_heuristic_cost(self.children_path[ii], self.child_nodes[ii].name)+self.compute_path_cost(self.child_nodes[ii]))
            self.compute_heuristic_cost(self.children_path[ii], self.child_nodes[ii].name)+
            self.compute_path_cost(self.child_nodes[ii])
            )
        # print(self.children_cost)
        # Get the best child index
        best_child_index = self.children_cost.index(np.min(self.children_cost))
        return best_child_index

    # TODO: Accept the cost function as input in the next iteration
    def get_best_child(self):
        """
        Returns the index of the best child node based on the cost function defined
        """
        self.children_cost = []
        for ii in range(self.num_child_nodes):
            h_cost, end_flag = self.compute_heuristic_cost(self.children_path[ii], self.child_nodes[ii].name)
            g_cost = self.compute_path_cost(self.child_nodes[ii])
            self.children_cost.append(h_cost+g_cost)
        # Get the best child index
        best_child_index = self.children_cost.index(np.min(self.children_cost))
        # Update the current_parent now
        # self.current_parent_node = self.child_nodes[best_child_index]
        return self.child_nodes[best_child_index], best_child_index, end_flag

    def get_best_child_v2(self, compute_heuristic_cost):
        """
        Returns the index of the best child node based on the cost function defined
        """
        self.children_cost = []
        for ii in range(self.num_child_nodes):
            h_cost, end_flag = compute_heuristic_cost(self.children_path[ii], self.child_nodes[ii].name)
            g_cost = self.compute_path_cost(self.child_nodes[ii])
            self.children_cost.append(h_cost+g_cost)
        # Get the best child index
        best_child_index = self.children_cost.index(np.min(self.children_cost))
        # Update the current_parent now
        # self.current_parent_node = self.child_nodes[best_child_index]
        return self.child_nodes[best_child_index], best_child_index, end_flag

    # Default visualisation is set to False
    def computeOptimalDesign(self, visualise_search = False):
        """
        TBD
        Returns the optimal design (composition of modules) using the prescribed search algorithm,
        with respect to the cost function defined

        :param visualise_search: Visualise the search procedure in the meshcat server
        """
        pass

# -----------------------------------------------------------------
# Below this are task specific functions needs to be set for DoD
# -----------------------------------------------------------------

    # Task specific cost of a given child
#     def compute_heuristic_cost(self, urdf_path, model_name):
#         """
#         Returns the heuristic cost of the selected module.
#         This function needs to be modified according to the task definition and
#         hence should be overridden for a required problem.
#
#         :param urdf_path: The path of the saved node
#         :param model_name: The name of the node
#         """
#         builder = DiagramBuilder()
#         plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
#         model = Parser(plant, scene_graph).AddModelFromFile(urdf_path, model_name)
#         # Weld the robot to the world frame
#         plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(model)[0]).body_frame())
#         # We can sample end-points on the surface of the sphere
# #         cart0 = np.array([-0.3, 0.4, 0.3])
#         plant.Finalize()
#         # Select the last frame as the eef_frame
#         gripper_frame = plant.GetBodyByName("eef").body_frame()
#         diagram = builder.Build()
#         context = diagram.CreateDefaultContext()
#         plant_context = plant.GetMyMutableContextFromRoot(context)
#         q0 = plant.GetPositions(plant_context)
#
#         # ADD POSITIONS HERE!!!
#         cart0 = np.array([0.15, 0.1, 0.45])
#         cartf = np.array([-0.05, 0.35, 0.3])
#
#         # Position-1
#         ik = InverseKinematics(plant, plant_context)
#         ik.AddPositionConstraint(
#                     gripper_frame, [0, 0, 0], plant.world_frame(),
#                     cart0, cart0)
#         prog = ik.get_mutable_prog()
#         q = ik.q()
#         prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
#         prog.SetInitialGuess(q, q0)
#         result = Solve(ik.prog())
#         res1 = result.is_success()
#         qr = result.GetSolution(ik.q())
#         # Bring it back to -pi and pi
#         qr = (np.arctan2(np.sin(qr), np.cos(qr)))
#         plant.SetPositions(plant.GetMyContextFromRoot(context),model,qr)
#         eef_pos_realised = plant.GetBodyByName('eef').EvalPoseInWorld(plant_context).translation()
#         cost1 = np.linalg.norm(eef_pos_realised-cart0)
#         # Position-2
#         ik = InverseKinematics(plant, plant_context)
#         ik.AddPositionConstraint(
#                     gripper_frame, [0, 0, 0], plant.world_frame(),
#                     cartf, cartf)
#         prog = ik.get_mutable_prog()
#         q = ik.q()
#         prog.AddQuadraticErrorCost(np.identity(len(q)), qr, q)
#         prog.SetInitialGuess(q, qr)
#         result = Solve(ik.prog())
#         res2 = result.is_success()
#         qr = result.GetSolution(ik.q())
#         plant.SetPositions(plant.GetMyContextFromRoot(context),model,qr)
#         eef_pos_realised = plant.GetBodyByName('eef').EvalPoseInWorld(plant_context).translation()
#         cost2 = np.linalg.norm(eef_pos_realised-cart0)
#         # Total cost
#         total_cost = cost1+cost2
#         # Final result
#         result = res1 and res2
#         return total_cost, result

    # # TODO: MODIFY THIS TO JUST DISPLAY CURRENT PARENT NODE
    # def publish_node_pose(self, node_instance, meshcat, t_sleep=0.5):
    #     """
    #     Displays the node poses given two points in space and returns the URDF location
    #     Needs modifying in the next code iteration
    #
    #     :param node: The object for the type `node` of which the attribute `current_parent_node` will be displayed
    #     :param meshcat: The meshcat object started
    #     :param t_sleep: Amount of time to wait between the two IK poses
    #     """
    #     # Setup basic diagram and the sphere at the desired location
    #     builder = DiagramBuilder()
    #     # Get path of the best child
    #     # urdf_path = node.children_path[best_child_idx]
    #     # model_name = node.child_nodes[best_child_idx].robot.name
    #     # Save the current node URDF and display it
    #
    #     temp_node_instance = copy.deepcopy(node_instance.current_parent_node)
    #     temp_node_instance.add_eef()
    #     urdf_path = temp_node_instance.constructURDF(node_instance.node_path)
    #     model_name = temp_node_instance.name
    #
    #     plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
    #     model = Parser(plant, scene_graph).AddModelFromFile(urdf_path, model_name)
    #     # Weld the robot to the world frame
    #     plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(model)[0]).body_frame())
    #
    #     # Spawn spherical work marker
    #     sphere1 = Parser(plant, scene_graph).AddModelFromFile('./urdfs/helper/sphere.urdf','sphere1')
    #     sphere2 = Parser(plant, scene_graph).AddModelFromFile('./urdfs/helper/sphere.urdf','sphere2')
    #     # We can sample end-points on the surface of the sphere
    #     cart0 = np.array([0.15, 0.1, 0.45])
    #     cartf = np.array([-0.05, 0.35, 0.3])
    #     X_R1 = RigidTransform(cart0)
    #     X_R2 = RigidTransform(cartf)
    #     plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(sphere1)[0]).body_frame(), X_R1)
    #     plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(sphere2)[0]).body_frame(), X_R2)
    #     plant.Finalize()
    #
    #     # Select the last frame as the eef_frame
    #     gripper_frame = plant.GetBodyByName("eef").body_frame()
    #
    #     meshcat.Delete()
    #     meshcat.DeleteAddedControls()
    #
    #     visualizer = MeshcatVisualizerCpp.AddToBuilder(builder, scene_graph, meshcat)
    #
    #     diagram = builder.Build()
    #     context = diagram.CreateDefaultContext()
    #     plant_context = plant.GetMyMutableContextFromRoot(context)
    #
    #     q0 = plant.GetPositions(plant_context)
    #     diagram.Publish(context)
    #
    #     # Check design
    #     ik = InverseKinematics(plant, plant_context)
    #     ik.AddPositionConstraint(
    #                 gripper_frame, [0, 0, 0], plant.world_frame(),
    #                 cart0, cart0)
    #     prog = ik.get_mutable_prog()
    #     q = ik.q()
    #     prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
    #     prog.SetInitialGuess(q, q0)
    #     result = Solve(ik.prog())
    #     qr = result.GetSolution(ik.q())
    #     qr = (np.arctan2(np.sin(qr), np.cos(qr)))
    #     # Visualise the best solution
    #     plant.SetPositions(plant.GetMyContextFromRoot(context),model,qr)
    #     diagram.Publish(context)
    #     end_game = result.is_success()
    #     time.sleep(t_sleep)
    #     # Check for position-2
    #     ik = InverseKinematics(plant, plant_context)
    #     ik.AddPositionConstraint(
    #                 gripper_frame, [0, 0, 0], plant.world_frame(),
    #                 cartf, cartf)
    #     prog = ik.get_mutable_prog()
    #     q = ik.q()
    #     prog.AddQuadraticErrorCost(np.identity(len(q)), qr, q)
    #     prog.SetInitialGuess(q, qr)
    #     result = Solve(ik.prog())
    #     qr = result.GetSolution(ik.q())
    #     # Visualise the best solution
    #     plant.SetPositions(plant.GetMyContextFromRoot(context),model,qr)
    #     diagram.Publish(context)
    #     return urdf_path
