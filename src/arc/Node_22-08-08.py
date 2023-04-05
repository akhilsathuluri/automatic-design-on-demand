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
        self.num_modules = len(roboDoD.declared_modules)
        # Directory to save the constructed URDFs
        self.node_path = './node_urdfs/'

    def get_compatible_modules(self):
        """
        Returns a list of compatible modules for the last module in the kinematic tree
        """
        # Get the type of the last node
        if not self.parent_node.tree:
            raise AssertionError("Error: No modules found in base node")
        # parent_type = self.parent_node.tree[list(roboDoD.tree)[-1]][0].type
        parent_type = self.parent_node.tree[list(self.parent_node.tree)[-1]][0].type
        # Identify all compatible nodes
        # self.compatible_modules = []
        compatible_modules = []
        for module in self.parent_node.declared_modules:
            if parent_type in module.parent_type:
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
            node_instance = copy.deepcopy(self.parent_node)
            module_name = 'node'+str(self.depth)+str(i)
            # Change instance name
            node_instance.name = module_name
            # Change URDF name
            node_instance.robot.name = module_name
            node_instances.append(node_instance)
        return node_instances


    # Expand corresponding nodes of robots based on compatible modules
    def expand_node(self):
        """
        Returns number of instances of the parent node equal to the number of
        compatible modules
        """
        # Get compatible nodes first
        compatible_modules = self.get_compatible_modules()
        self.num_child_nodes = len(compatible_modules)
        # Now create required number of instances
        node_instances = self.create_instances(self.num_child_nodes)
        # Add each module to each new instance
        self.children_path = []
        for i in range(len(compatible_modules)):
            node_instances[i].add_module(compatible_modules[i])
            # Construct the URDF files of these instances
            # Add an eef link at the end
            temp_node_instance = copy.deepcopy(node_instances[i])
            temp_node_instance.add_eef()
            self.children_path.append(
                temp_node_instance.constructURDF(self.node_path))

        self.child_nodes = node_instances
        # Increase the depth of the tree
        self.depth += 1

    # !!!IGNORING PATH COST FOR NOW
    def get_best_child_index(self):
        """
        Returns the index of the best child node based on the cost function defined
        """
        self.children_cost = []
        for ii in range(self.num_child_nodes):
            self.children_cost.append(
#                 self.compute_heuristic_cost(self.children_path[ii], self.child_nodes[ii].name)+self.compute_path_cost(self.child_nodes[ii]))
            self.compute_heuristic_cost(self.children_path[ii], self.child_nodes[ii].name))
        # Get the best child index
        best_child_index = self.children_cost.index(np.min(self.children_cost))
        return best_child_index

    # Default visualisation is set to False
    def computeOptimalDesign(self, visualise_search = False):
        """
        TBD
        Returns the optimal design (composition of modules) using the prescribed search algorithm,
        with respect to the cost function defined

        :param visualise_search: Visualise the search procedure in the meshcat server
        """
        pass

    # compute path costs based on modules selected
    def compute_path_cost(self, node):
        """
        Returns the path cost of the current robot i.e., the cost associated
        with the selected modules of the current architecture (/Sigma C(M))

        :param node: The selected node
        """
        mod_list = list(node.tree.keys())
        path_cost = 0
        for mod in mod_list:
            path_cost+=node.tree[mod][0].cost
        return path_cost

# -----------------------------------------------------------------
# Below this are task specific functions needs to be set for DoD
# ToDo: These needs to be given by the user -> Can use drake or anything
# else they prefer
# -----------------------------------------------------------------

    # Task specific cost of a given child
    def compute_heuristic_cost(self, urdf_path, model_name):
        """
        Returns the heuristic cost of the selected module.
        This function needs to be modified according to the task definition and
        hence should be overridden for a required problem.

        :param urdf_path: The path of the saved node
        :param model_name: The name of the node
        """
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
        model = Parser(plant, scene_graph).AddModelFromFile(urdf_path, model_name)
        # Weld the robot to the world frame
        plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(model)[0]).body_frame())
        # We can sample end-points on the surface of the sphere
#         cart0 = np.array([-0.3, 0.4, 0.3])
        plant.Finalize()
        # Select the last frame as the eef_frame
        gripper_frame = plant.GetBodyByName("eef").body_frame()
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyMutableContextFromRoot(context)
        q0 = plant.GetPositions(plant_context)

        # ADD POSITIONS HERE!!!
        cart0 = np.array([0.15, 0.1, 0.45])
        cartf = np.array([-0.05, 0.35, 0.3])

        # Position-1
        ik = InverseKinematics(plant, plant_context)
        ik.AddPositionConstraint(
                    gripper_frame, [0, 0, 0], plant.world_frame(),
                    cart0, cart0)
        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
        prog.SetInitialGuess(q, q0)
        result = Solve(ik.prog())
        qr = result.GetSolution(ik.q())
        # Bring it back to -pi and pi
        qr = (np.arctan2(np.sin(qr), np.cos(qr)))
        plant.SetPositions(plant.GetMyContextFromRoot(context),model,qr)
        eef_pos_realised = plant.GetBodyByName('eef').EvalPoseInWorld(plant_context).translation()
        cost1 = np.linalg.norm(eef_pos_realised-cart0)
        # Position-2
        ik = InverseKinematics(plant, plant_context)
        ik.AddPositionConstraint(
                    gripper_frame, [0, 0, 0], plant.world_frame(),
                    cartf, cartf)
        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), qr, q)
        prog.SetInitialGuess(q, qr)
        result = Solve(ik.prog())
        qr = result.GetSolution(ik.q())
        plant.SetPositions(plant.GetMyContextFromRoot(context),model,qr)
        eef_pos_realised = plant.GetBodyByName('eef').EvalPoseInWorld(plant_context).translation()
        cost2 = np.linalg.norm(eef_pos_realised-cart0)
        # Total cost
        total_cost = cost1+cost2
        return total_cost
