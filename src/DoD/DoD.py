"""
**LCL-Robots** is a project for the automatic generation of task-specific robots using
user specified modules. The current library, `design-on-demand (DoD)` provides the enables
the automatic design process of robots provided the modules and the required task.
"""

# Base imports
import numpy as np

# Drake imports
from pydrake.all import *
from manipulation.scenarios import AddMultibodyTriad

# DoD-specific imports
# for dev
from odio_urdf import *
# odio_urdf is now built as a part of DoD
# from DoD.odio_urdf import *

from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize, LinearConstraint, NonlinearConstraint

# Initiate the tree structure with a base
# For now we have the module class within the main class
# This class allows one to define the modules and the relations between them
# Also contains functions to automatically attach modules together
# creating robots. The class is also capable of exporting the robot as a
# URDF and uploads it to the meshcat server for visualisation
class robotDoD:
    """
    Returns the object for a higher-level class with basic set of tools to setup the modules.
    The higher-level class which consists of `robotModule` class and contains functions to manipulate modules.
    Further houses the necessary functions to manipulate the constructed robots.

    :param name: Required name for the robot that is to be created
    """

    # A class to define all the modules and their description
    class robotModule:
        """
        Returns the object with basic set of tools to describe the modules.
        The base class for describing the compatibility rules and module classes.
        Further houses the necessary functions to assemble various modules together.

        :param name: Required name for the module that is to be created
        """
        def __init__(self, name):
            self.name = name;
            # Define interfaces poition and orientation
            self.intA = np.array([[0,0,0],[0,0,0]]) # Interface A origin and orientation
            self.intB = np.array([[0,0,0],[0,0,0]]) # Interface B origin and orientation
            self.mesh_path = "" # Set the location of the path
            self.scale = "0.001 0.001 0.001" # Set scale as string
            self.type = 0 # 0-link; 1-joint
            self.cost = 0 # Cost associated with a particular module
            # Parent type gives the compatibility between modules
            # Links cannot connect to links and same with joints
            self.parent_type = -1 # -1 if parent type is not set
            # Set default values of inertial properties
            self.mass = 0
            self.inertia = np.zeros(3)


    # A helper class for transformation
    class helperFuncs:
        """
        Class containing all the helper functions, i.e., primitive mathematical functions required to
        manipulate the modules.
        """
        def __init__(self):
            pass

        def list2space(self, com):
            """
            Converts a given list into space seperated string required by a URDF file

            :param com: An array or list that needs to be converted
            """
            return str(com[0])+' '+str(com[1])+' '+str(com[2])

        def rotZ(self, angle):
            """
            Returns a rotation matrix for a rotation about the Z-axis by a given angle

            :param angle: Angle of rotation in radians
            """
            return np.array([[np.cos(angle), -np.sin(angle), 0],[np.sin(angle), np.cos(angle), 0],[0,0,1]])

        def rotY(self, angle):
            """
            Returns a rotation matrix for a rotation about the Y-axis by a given angle

            :param angle: Angle of rotation in radians
            """
            return np.array([[np.cos(angle), 0, np.sin(angle)],[0, 1, 0],[-np.sin(angle), 0, np.cos(angle)]])

        def rotX(self, angle):
            """
            Returns a rotation matrix for a rotation about the X-axis by a given angle

            :param angle: Angle of rotation in radians
            """
            return np.array([[1,0,0], [0, np.cos(angle), -np.sin(angle)],[0, np.sin(angle), np.cos(angle)]])

        def transform_vector(self, axis, angle, vector):
            """
            Transforms a given vector about a given axis and angle

            :param axis: Axis of rotation
            :param angle: Angle of rotation in radians
            :param vector: The vector which needs to be transformed
            """
            if axis == 0:
                return self.rotX(angle).dot(vector)
            if axis == 1:
                return self.rotY(angle).dot(vector)
            if axis == 2:
                return self.rotZ(angle).dot(vector)
            else:
                raise AssertionError("Error: Rotation about unknown axis requested")

        # Resolve the axis and angle when aligned
        def resolve_axis(self, module):
            """
            Resolves the axis and the angle given a singularity, i.e., if there is a gimbal lock

            :param module: The module object
            """
            def foo(x):
                return x[0]*x[1]*x[2]

            def non_lin(x):
                return np.linalg.norm(x)

            lin_constraint = LinearConstraint(module.intA[1], 0, 0)
            nonlin_constraint = NonlinearConstraint(non_lin, 1, 1)

            sol = minimize(foo,[1/np.sqrt(3),1/np.sqrt(3),1/np.sqrt(3)], constraints = [lin_constraint, nonlin_constraint])
            return sol['x']

    def __init__(self, name):
        self.name = name
        # Initiate the helper class
        self.helper_funcs = self.helperFuncs()
        # Declare materials
        self.materials = Group(
            Material("blue", Color(rgba="0 0 0.8 1.0")),
            Material("red", Color(rgba="0.8 0 0 1.0")),
            Material("white", Color(rgba="1.0 1.0 1.0 0.4")),
            Material("black", Color(rgba="0.0 0.0 0.0 1.0")),
            Material("gray", Color(rgba="0.42 0.47 0.53 1.0")),)
        self.material_color = ["blue", "red", "white", "gray", "black"]
        self.module_types = ["linkMod", "jointMod", "connMod", "eefMod", "baseMod"]
        # Declare the robot
        self.robot = Robot(self.name, self.materials)
        # Declare robots tree structure
        self.tree = {}
        self.prev_module = []
        self.prev_module_type = -1
        # Initate all the modules internally
        self.initiate_modules()

    def add_root(self, default=True):
        """
        Add a root frame, or a world frame to initiate the kinematic tree
        """
        # Add root module to the robot URDF
        if default == True:
            root_mod = self.robotModule('root_mod')
            # Define the module
            root_mod.intA = np.array([[0,0,0],[0,0,-1]])
            root_mod.intB = np.array([[0,0,0.01],[0,0,1]])
            root_mod.type = -1
            root_mod.mass = 0
            root_mod.inertia = np.zeros(3)
            root_mod.cost = 0

            root_link_urdf = Link(
                    Visual(
                        Origin(rpy = "0 0 0", xyz = self.helper_funcs.list2space(root_mod.intA[0])),
                        Material (name = "white"),
                        Geometry(Cylinder(length = 0.001, radius = 0.1))),
                    name = root_mod.name)

        else:
            raise NotImplementedError

        self.prev_module = root_mod.intB
        self.prev_module_type = root_mod.type
        self.robot(root_link_urdf)
        self.tree[root_mod.name] = [root_mod, root_mod.intB]

    def add_module(self, module):
        """
        Adds a given module to the kinematic tree.
        This addition not only adds the corresponding link in the URDF, but also
        adds the corresponding joint based on the module type. So a joint module also
        adds a joint tag and a transmission tag along with the link in the URDF.

        :param module: The module object
        """
        # Always start from the root module which initates the self.prev_module
        # Check if prev_module is empty => root not initiated
        if not self.prev_module.any():
            raise AssertionError("Error: Add root module first before initiating the tree")
        # Get info from self.prev_module
        # Compute connection rules from prev and current modules
        current_origin, current_orientation = self.computeConnection(module)

        link_name = self.module_types[module.type]+ '_'+str(len(self.tree.keys()))
        # Generate module URDF
        module_link_urdf = Link(
                            Inertial(
                            # Need to verify the origin and orientation of the CoM
                             Origin(xyz = self.helper_funcs.list2space(current_origin), rpy = self.helper_funcs.list2space(current_orientation)),
                             Mass(value= str(module.mass)),
                             Inertia(
                                ixx= str(module.inertia[0, 0]),
                                ixy= str(module.inertia[0, 1]),
                                ixz= str(module.inertia[0, 2]),
                                iyy= str(module.inertia[1, 1]),
                                iyz= str(module.inertia[1, 2]),
                                izz= str(module.inertia[2, 2])
                                ),
                            ),
                             Visual(
                                Origin(xyz = self.helper_funcs.list2space(current_origin), rpy = self.helper_funcs.list2space(current_orientation)),
                                Material(name = self.material_color[module.type]),
                                Geometry(Mesh(filename = module.mesh_path, scale = module.scale))),
                            # Adding collision geometry
                            Collision(
                               Origin(xyz = self.helper_funcs.list2space(current_origin), rpy = self.helper_funcs.list2space(current_orientation)),
                               Material(name = self.material_color[module.type]),
                               Geometry(Mesh(filename = module.mesh_path, scale = module.scale))),

                            name= link_name)

        # Add the module to the robot URDF
        self.robot(module_link_urdf)

        # Now add joints everytime a new module is added
        # If link is added then add a revolute joint to the previous module
        # If a joint is added then add a fixed joint with the previous module
        if module.type == -1:
            # If the module type is root, the fix it to the module of type world
            joint_type = "fixed"
            joint_name  = 'joint_' + str(len(self.tree.keys()))+'_'+ joint_type
            module_joint_urdf = Joint(
                                    Parent(list(self.tree.keys())[-1]),
                                    Child(link_name),
                                    Origin(xyz = self.helper_funcs.list2space(self.prev_module[0]), rpy = self.helper_funcs.list2space([0,0,0])),
                                    Limit(effort= "50",upper= "3.14",velocity= "10",lower= "-3.14"),
                                    type = joint_type,
                                    name= joint_name)
            self.robot(module_joint_urdf)

        # i.e., if the previous module is a connector or a link, then its a fixed joint
        # if self.prev_module_type == 2 or self.prev_module_type == 0:
        if self.prev_module_type == 3 or self.prev_module_type == 2 or self.prev_module_type == 0:
            joint_type = "fixed"
            joint_name  = 'joint_' + str(len(self.tree.keys()))+'_'+ joint_type
            module_joint_urdf = Joint(
                                    Parent(list(self.tree.keys())[-1]),
                                    Child(link_name),
                                    Origin(xyz = self.helper_funcs.list2space(self.prev_module[0]), rpy = self.helper_funcs.list2space([0,0,0])),
                                    Axis(xyz = self.helper_funcs.list2space(self.prev_module[1])),
                                    Limit(effort= "50",upper= "3.14",velocity= "10",lower= "-3.14"),
                                    type = joint_type,
                                    name= joint_name)
            self.robot(module_joint_urdf)

        if self.prev_module_type == 1 or (self.prev_module_type == -1 and module.type!=-1): # If joint or base
            joint_type = "revolute"
            joint_name  = 'joint_' + str(len(self.tree.keys()))+'_'+ joint_type
            module_joint_urdf = Joint(
                                    Parent(list(self.tree.keys())[-1]),
                                    Child(link_name),
                                    Origin(xyz = self.helper_funcs.list2space(self.prev_module[0]), rpy = self.helper_funcs.list2space([0,0,0])),
                                    Axis(xyz = self.helper_funcs.list2space(self.prev_module[1])),
                                    Limit(effort= "50",upper= "3.14",velocity= "10",lower= "-3.14"),
                                    type = joint_type,
                                    name= joint_name)

            self.robot(module_joint_urdf)

            # If revolute then also add an actuator here
            actuator_name = 'act_'+joint_name
            transmission_name = 'trans_'+joint_name
            module_actuator_urdf = Transmission(Type("SimpleTransmission"),
                                    Actuator(Mechanicalreduction('1'), name = transmission_name),
                                    Transjoint(joint_name),
                                    name = actuator_name)

            self.robot(module_actuator_urdf)

        # Save all the origins and orientations in a tree
        self.prev_module = self.new_prev_module
        self.prev_module_type = module.type
        self.tree[link_name] = [module, self.prev_module]



    def add_eef(self, eef_load=0):
        """
        Adds an end_effector to the end of the kinematic tree with a
        default mass of 0kg

        """
        eef_link_name = 'eef'
        # Generate module URDF
        # eef_link_urdf = Link(Inertial(Mass(value= str(eef_load)),
        #                     name= eef_link_name)
        #                     )

        eef_link_urdf = Link(
                            Inertial(
                             Mass(value= str(eef_load)),
                            ),
                            name= eef_link_name)

        # Use the saved rotation matrix
        # orieef = self.save_rmat.as_euler('xyz')
        orieef = self.r.as_euler('xyz')

        # Add the eef link to the robot URDF
        self.robot(eef_link_urdf)
        # Now add the eef to the end link
        joint_type = "fixed"
        joint_name  = 'eef_joint'
        eef_joint_urdf = Joint(
                        Parent(list(self.tree.keys())[-1]),
                        Child(eef_link_name),
                        # Origin(xyz = self.helper_funcs.list2space(self.prev_module[0]), rpy = self.helper_funcs.list2space([0,0,0])),
                        Origin(xyz = self.helper_funcs.list2space(self.prev_module[0]), rpy = self.helper_funcs.list2space(orieef)),
                        # Axis(xyz = self.helper_funcs.list2space(self.prev_module[1])),
                        Limit(effort= "50",upper= "3.14",velocity= "10",lower= "-3.14"),
                        type = joint_type,
                        name= joint_name)

        self.robot(eef_joint_urdf)


    def constructURDF(self, path = None):
        """
        Saves a the URDF file to disk

        :param path: None by default, but path for the URDF file to be saved can be given
        """
        name = self.robot.name+str(np.random.randint(99999))
        if path != None:
            name = path+name

        name = name+'.urdf'
        # Writing robot URDF to path
        with open(name, 'w') as f:
            print(self.robot, file=f)
            # print('Robot design successfully saved as '+name)
        # Return address to all the created robots, so that
        # They can be directly loaded to drake if needed
        return name

    def visualiseRobot(self, urdf_path, meshcat, frames=False):
        """
        Given a URDF file, publishes the diagram to a given meshcat server

        :param urdf_path: The URDF file path
        :param meshcat: The initialised meshcat instance
        :param frames: Optionally add frame information to the published plant (robot)
        """
        # Loading the updated robot URDF
        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=0.001)
        parser = Parser(self.plant, self.scene_graph)
        parser.package_map().Add("mods", "/home/laniakea/git/research/lcl-robots-DoD/src/DoD/")
        self.model = parser.AddModelFromFile(urdf_path)

        self.plant.Finalize()

        if frames == True:
            for body in self.plant.GetBodyIndices(self.model):
                body_name = self.plant.get_body(body).name()
                AddMultibodyTriad(self.plant.GetFrameByName(body_name, self.model), \
                                  self.scene_graph)

        meshcat.Delete()
        meshcat.DeleteAddedControls()

        # MeshcatVisualizerCpp --> depreciated
        # self.visualizer = MeshcatVisualizerCpp.AddToBuilder(self.builder, self.scene_graph, meshcat)
        self.visualizer = MeshcatVisualizer.AddToBuilder(self.builder, self.scene_graph, meshcat)

        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.diagram.ForcedPublish(self.diagram_context)

    def computeConnection(self, module):
        """
        Computes spatial tranformations with the previous module in the kinematic tree.

        :param module: The module to be added
        """
        current_orientation = np.zeros(3)
        # Compute the rotation axis
        temp = np.cross(self.prev_module[1], module.intA[1])
        sinangle = np.linalg.norm(temp)
        cosangle = np.round(np.dot(self.prev_module[1], module.intA[1]))
        # Hangle sinangle being zero
        # print(module.name)
        if sinangle == 0 and cosangle == 1:
            # We need the vector flipped
            # We need to resolve redundancy here by solving an optimisation problem
            rotaxis = self.helper_funcs.resolve_axis(module)
            rotangle = np.pi

        if sinangle == 0 and cosangle == -1:
            rotangle = 0
            rotaxis = np.zeros(3)

        if sinangle != 0:
            rotaxis = temp/sinangle
            # Compute the rotation angle
            rotangle = np.arctan2(sinangle, cosangle)

        # Compute the quaternion
        # r = R.from_rotvec(rotangle*rotaxis)
        self.r = R.from_rotvec(rotangle*rotaxis)
        # Convert to Euler rotation 'xyz'
        current_orientation = self.r.as_euler('xyz')

        # Rotation matrix
        rmat = self.r.as_matrix()
        # self.save_rmat = r
        r = self.r
        current_origin = np.dot(rmat,(-module.intA[0]))
        # Next orientation
        next_orientation = np.dot(rmat, module.intB[1])
        next_origin = current_origin + np.dot(rmat, module.intB[0])
        # Update prev_module
        self.new_prev_module = np.array([next_origin, np.round(next_orientation)])

        return current_origin, current_orientation

    def visualiseModule(self, module, meshcat):
        """
        Visualise input module

        :param module: The module instance
        :param meshcat: The initialised meshcat instance
        """
        # Loading the module
        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=0.001)

        module_urdf = Robot(module.name, self.materials)
        # Generate module URDF
        module_link_urdf = Link(
                         Visual(
                            Origin(xyz = self.helper_funcs.list2space([0,0,0]), rpy = self.helper_funcs.list2space([0,0,0])),
                            Material(name = self.material_color[module.type]),
                            Geometry(Mesh(filename = module.mesh_path, scale = module.scale))),
                        name=module.name)

        # Add the module to the robot URDF
        module_urdf(module_link_urdf)
        name = module.name+'.urdf'
        with open(name, 'w') as f:
            print(module_urdf, file=f)
            print('Module URDF successfully saved as '+name)

        urdf_path = './'+name
        self.model = Parser(self.plant).AddModelFromFile(urdf_path)
        self.plant.Finalize()

        meshcat.Delete()
        meshcat.DeleteAddedControls()

        self.visualizer = MeshcatVisualizer.AddToBuilder(self.builder, self.scene_graph, meshcat)

        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.diagram.ForcedPublish(self.diagram_context)

# -----------------------------------------------------------------
# Below this are the declared modules for DoD
# -----------------------------------------------------------------

    # One can initiate it from within or externally define new modules that
    # needs to be considered for the simulation
    def initiate_modules(self):
        """
        Initiates default modules as per the LCL-Robots projects.
        These modules can be inspected using the meshes folder
        """
        # jmod
        self.jmod = self.robotModule('jmod')
        # Define the module
        self.jmod.intA = np.array([[0,-0.0035,0],[0,-1,0]])
        self.jmod.intB = np.array([[0.03, 0.0495, 0.0003],[1,0,0]])
        self.jmod.type = 1
        self.jmod.mesh_path = "package://mods/meshes/j_module.obj"
        self.jmod.cost = 100
        self.jmod.parent_type = [-1, 0]
        # Inertial properties about the CoM
        self.jmod.mass = 0.6
        # Approx mod length
        mod_len = np.linalg.norm(self.jmod.intA[0]-self.jmod.intB[0])
        self.jmod.inertia = np.diag([1/3*self.jmod.mass*(mod_len**2), 1/3*self.jmod.mass*(mod_len**2), 1/2*self.jmod.mass*(mod_len**2)])
        # Define compatibilities
        # link-0, joint-1, connector-2
        link_com = [1,-1,2]
        joint_com = [2,0,-1,1]
        # joint_com = [2,0]
        conn_com = [-1,1,2,0]
        # conn_com = [-1,1]

        # l1mod
        self.l1mod = self.robotModule('l1mod')
        # Define the module
        self.l1mod.intA = np.array([[0.04,0,0],[1,0,0]])
        # l1mod.intB = np.array([[0.035,0.16,0],[1,0,0]])
        self.l1mod.intB = np.array([[0.04,0.16,0],[1,0,0]])
        self.l1mod.type = 0
        self.l1mod.mesh_path = "package://mods/meshes/l1_module.obj"
        self.l1mod.cost = 20
        self.l1mod.parent_type = [-1, 1]
        # Inertial properties about the CoM
        self.l1mod.mass = 0.2
        # Approx mod length
        mod_len = np.linalg.norm(self.jmod.intA[0]-self.jmod.intB[0])
        self.l1mod.inertia = np.diag([1/2*self.jmod.mass*(mod_len**2), 0, 1/2*self.jmod.mass*(mod_len**2)])


        # l2mod
        self.l2mod = self.robotModule('l2mod')
        # Define the module
        self.l2mod.intA = np.array([[0.04,0,0],[1,0,0]])
        self.l2mod.intB = np.array([[0,0.148,0],[0,1,0]])
        self.l2mod.type = 0
        self.l2mod.mesh_path = "package://mods/meshes/l2_module.obj"
        self.l2mod.cost = 20
        self.l2mod.parent_type = [-1, 1]
        # Inertial properties about the CoM
        self.l2mod.mass = 0.2
        # Approx mod length
        mod_len = np.linalg.norm(self.jmod.intA[0]-self.jmod.intB[0])
        self.l2mod.inertia = np.diag([1/2*self.jmod.mass*(mod_len**2), 0, 1/2*self.jmod.mass*(mod_len**2)])

        # l3mod
        self.l3mod = self.robotModule('l3mod')
        # Define the module
        self.l3mod.intA = np.array([[0,0.148,0],[0,1,0]])
        self.l3mod.intB = np.array([[0.04, 0, 0],[1,0,0]])
        self.l3mod.type = 0
        self.l3mod.mesh_path = "package://mods/meshes/l3_module.obj"
        self.l3mod.cost = 20
        self.l3mod.parent_type = [-1, 1]
        # Inertial properties about the CoM
        self.l3mod.mass = 0.2
        # Approx mod length
        mod_len = np.linalg.norm(self.jmod.intA[0]-self.jmod.intB[0])
        self.l3mod.inertia = np.diag([1/2*self.jmod.mass*(mod_len**2), 0, 1/2*self.jmod.mass*(mod_len**2)])

        # NEW LCL MODULES
        # !!! Set the module compatibility rules
        # Class rank: -1:root, 0:links, 1:joints, 2:connectors

        # 90-connector (c90)
        self.c90 = self.robotModule('c90')
        # Define the module
        self.c90.intA = np.array([[0,-0.02,0],[0,-1,0]])
        self.c90.intB = np.array([[0,0,0.06],[0,0,1]])
        self.c90.type = 2
        self.c90.mesh_path = "package://mods/meshes/90-connector.obj"
        self.c90.cost = 25
        # Connects to base, links and joints
        self.c90.parent_type = conn_com
        # Inertial properties about the CoM
        self.c90.mass = 0.263
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.c90.inertia = np.round(np.array([[2.264e+05,0.157,-0.103],
                                              [0.157,2.300e+05,-14552.154],
                                              [-0.103,-14552.154,1.252e+05]])*1e-9,6)


        # base_mod (base360)
        self.base360 = self.robotModule('base360')
        # Define the module
        self.base360.intA = np.array([[0,-0.055,0],[0,-1,0]])
        self.base360.intB = np.array([[-0.0175,0.0785,0],[0,1,0]])
        self.base360.type = -1
        self.base360.mesh_path = "package://mods/meshes/base-big-360.obj"
        self.base360.cost = 0
        self.base360.parent_type = [-1, 0]
        # Inertial properties about the CoM
        self.base360.mass = 1.245
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.base360.inertia = np.round(np.array([
                                [2.797e+06,-459.674,-151.03],
                                [-459.674,2.417e+06,14514.353],
                                [-151.03,14514.353,3.107e+06]])*1e-9,6)

        # 90-joint (j90)
        self.j90 = self.robotModule('j90')
        # Define the module
        self.j90.intA = np.array([[0.052, 0, 0],[1,0,0]])
        self.j90.intB = np.array([[0,-0.052,0],[0,-1,0]])
        self.j90.type = 1
        # self.j90.mesh_path = "./meshes/90-joint.obj"
        # replace with absolute paths to load from string?
        # self.j90.mesh_path = "/home/laniakea/git/research/lcl-robots-DoD/examples/meshes/90-joint.obj"
        # adding a package map
        self.j90.mesh_path = "package://mods/meshes/90-joint.obj"
        self.j90.cost = 40
        self.j90.parent_type = joint_com
        # Inertial properties about the CoM
        self.j90.mass = 0.184 # Adding the mass of the SM40BL motor
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.j90.inertia = np.round(np.array([
                                [77643.962,-13403.94,-0.001],
                                [-13403.94,77666.869,0.002],
                                [-0.001,0.002,90470.907]])*1e-9,6)

        # 180-joint (j180)
        self.j180 = self.robotModule('j180')
        # Define the module
        self.j180.intA = np.array([[0, 0, 0],[0,1,0]])
        self.j180.intB = np.array([[0,-0.05,0],[0,-1,0]])
        self.j180.type = 1
        self.j180.mesh_path = "package://mods/meshes/j180.obj"
        self.j180.cost = 40
        self.j180.parent_type = [2,0,-1]
        # Inertial properties about the CoM
        self.j180.mass = 0.184 # Adding the mass of the SM40BL motor
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.j180.inertia = np.round(np.array([
                                [77643.962,-13403.94,-0.001],
                                [-13403.94,77666.869,0.002],
                                [-0.001,0.002,90470.907]])*1e-9,6)

        # 180-connector (c180)
        self.c180 = self.robotModule('c180')
        # Define the module
        self.c180.intA = np.array([[0,0.035,0.025], [0,0,1]])
        self.c180.intB = np.array([[0,-0.035,0.025], [0,0,1]])
        self.c180.type = 2
        self.c180.mesh_path = "package://mods/meshes/180-connector-2.obj"
        self.c180.cost = 30
        self.c180.parent_type = conn_com
        # Inertial properties about the CoM
        self.c180.mass = 0.423
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.c180.inertia = np.round(np.array([
                                [5.240e+05,0.01,0.049],
                                [0.01,1.980e+05,2703.107],
                                [0.049,2703.107,5.217e+05]])*1e-9,6)

        # 180-connector-twisted (c180t)
        self.c180t = self.robotModule('c180t')
        # Define the module
        self.c180t.intA = np.array([[0.0,0.0,0.0], [0,0,-1]])
        self.c180t.intB = np.array([[0.0885,-0.0425,0.0555], [0,-1,0]])
        self.c180t.type = 2
        self.c180t.mesh_path = "package://mods/meshes/c180t.obj"
        self.c180t.cost = 30
        self.c180t.parent_type = conn_com
        # Inertial properties about the CoM
        self.c180t.mass = 0.423
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.c180t.inertia = np.round(np.array([
                                [5.240e+05,0.01,0.049],
                                [0.01,1.980e+05,2703.107],
                                [0.049,2703.107,5.217e+05]])*1e-9,6)

        # c-link-1 (cl1)
        self.cl1 = self.robotModule('cl1')
        # Define the module
        self.cl1.intA = np.array([[-0.01,-0.06,0],[-1,0,0]])
        self.cl1.intB = np.array([[-0.01,0.06,0],[-1,0,0]])
        self.cl1.type = 0
#         self.cl1.mesh_path = "package://mods/meshes/c-link-2.obj"
        self.cl1.mesh_path = "package://mods/meshes/curved-link-3.obj"
        self.cl1.cost = 5
        self.cl1.parent_type = link_com
        # Inertial properties about the CoM
        self.cl1.mass = 0.11
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.cl1.inertia = np.round(np.array([
                                [2.426e+05,0.00,0.0],
                                [0.0,2.039e+05,0.0],
                                [0.0,0.0,4.256e+05]])*1e-9,6)

        # connector-0 (c0)
        self.c0 = self.robotModule('c0')
        # Define the module
        self.c0.intA = np.array([[0.062, 0, 0],[1,0,0]])
        self.c0.intB = np.array([[0.06, 0, 0],[-1,0,0]])
        self.c0.type = 2
        self.c0.mesh_path = "package://mods/meshes/connector-0.obj"
        self.c0.cost = 10
        self.c0.parent_type = conn_com
        # Inertial properties about the CoM
        self.c0.mass = 0.032
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.c0.inertia = np.round(np.array([
                                [20549.133,0.00,0.0],
                                [0.0,10874.708,0.0],
                                [0.0,0.0,11554.177]])*1e-9,6)

        # link-50-alu (l50)
        self.l50alu = self.robotModule('l50alu')
        # Define the module
        self.l50alu.intA = np.array([[0.062,0,0],[-1,0,0]])
        self.l50alu.intB = np.array([[0.112,0,0],[1,0,0]])
        self.l50alu.type = 0
        self.l50alu.mesh_path = "./modules_v2/alu/l50_alu.obj"
        self.l50alu.cost = 5
        self.l50alu.parent_type = link_com
        # Inertial properties about the CoM
        self.l50alu.mass = 0.050
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.l50alu.inertia = np.round(np.array([
                                [8152.248,0.00,0.0],
                                [0.0,13129.584,0.0],
                                [0.0,0.0,15692.664]])*1e-9,6)


        # link-50 (l50)
        self.l50 = self.robotModule('l50')
        # Define the module
        self.l50.intA = np.array([[0.062,0,0],[-1,0,0]])
        self.l50.intB = np.array([[0.112,0,0],[1,0,0]])
        self.l50.type = 0
        self.l50.mesh_path = "package://mods/meshes/link-50.obj"
        self.l50.cost = 5
        self.l50.parent_type = link_com
        # Inertial properties about the CoM
        self.l50.mass = 0.050
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.l50.inertia = np.round(np.array([
                                [8152.248,0.00,0.0],
                                [0.0,13129.584,0.0],
                                [0.0,0.0,15692.664]])*1e-9,6)

        # link-100 (l100)
        self.l100 = self.robotModule('l100')
        # Define the module
        self.l100.intA = np.array([[0.062,0,0],[-1,0,0]])
        self.l100.intB = np.array([[0.162,0,0],[1,0,0]])
        self.l100.type = 0
        self.l100.mesh_path = "package://mods/meshes/link-100.obj"
        self.l100.cost = 10
        self.l100.parent_type = link_com
        # Inertial properties about the CoM
        self.l100.mass = 0.050
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.l100.inertia = np.round(np.array([
                                [65040.781,0.00,0.0],
                                [0.0,2.060e+05,0.0],
                                [0.0,0.0,2.060e+05]])*1e-9,6)

        # link-150 (l150)
        self.l150 = self.robotModule('l150')
        # Define the module
        self.l150.intA = np.array([[0.062,0,0],[-1,0,0]])
        self.l150.intB = np.array([[0.212,0,0],[1,0,0]])
        self.l150.type = 0
        self.l150.mesh_path = "package://mods/meshes/link-150.obj"
        self.l150.cost = 15
        self.l150.parent_type = link_com
        # Inertial properties about the CoM
        self.l150.mass = 0.167
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.l150.inertia = np.round(np.array([
                                [27410.011,0.00,0.0],
                                [0.0,3.221e+05,0.0],
                                [0.0,0.0,3.308e+05]])*1e-9,6)

        
        # eef1l (end-effector with the load cell)
        self.eef1l = self.robotModule('eef1l')
        # Define the module
        self.eef1l.intA = np.array([[-0.029,0,0],[-1,0,0]])
        self.eef1l.intB = np.array([[0.08,0,0.01],[1,0,0]])
        self.eef1l.type = 3
        self.eef1l.mesh_path = "package://mods/meshes/eef1l.obj"
        self.eef1l.cost = 0
        self.eef1l.parent_type = link_com
        # Inertial properties about the CoM
        self.eef1l.mass = 0.5
        # Inertia data from CAD in gm*mm^2 -> kg*m^2
        self.eef1l.inertia = np.round(np.array([
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]]),6)

        self.declared_modules = [
#             self.c90,
                                 self.j90,
                                 self.c180,
#                                  self.cl1,
#                                  self.c0,
                                 self.l50,
                                 self.l100,
                                 self.l150,
                                 self.eef1l,
                                ]
                                
                                

# """
# **LCL-Robots** is a project for the automatic generation of task-specific robots using
# user specified modules. The current library, `design-on-demand (DoD)` provides the enables
# the automatic design process of robots provided the modules and the required task.
# """

# class robotNode:
#     """
#     This class is at the intermediate level where it takes in the robot
#     module class and uses the compatibility rules from the previous
#     class to constructs meaningful robots, this also does all the
#     housekeeping of creating multiple instances of the robot environment
#     i.e., creating branches and populating the correct branch

#     :param roboDoD: An instance of the class robotDoD
#     """
#     def __init__(self, roboDoD):
#         self.depth = 0
#         # Dont use self.base_node to prevent editing it by mistake
#         self.parent_node = roboDoD
#         # Also save the current_parent_node for all the references
#         self.current_parent_node = roboDoD
#         self.current_parent_node_path = ''
#         self.num_modules = len(roboDoD.declared_modules)
#         # Directory to save the constructed URDFs
#         self.node_path = './node_urdfs/'

#     # ToDo: THE CURRENT ROBOT IS NOT BEING UPDATED WITH THE LATEST MODULES!!!

#     def get_compatible_modules(self):
#         """
#         Returns a list of compatible modules for the last module in the kinematic tree
#         """
#         # Get the type of the last node
#         if not self.parent_node.tree:
#             raise AssertionError("Error: No modules found in base node")
#         last_module_type = self.current_parent_node.tree[list(self.current_parent_node.tree)[-1]][0].type

#         # Identify all compatible nodes
#         # self.compatible_modules = []
#         compatible_modules = []
#         for module in self.parent_node.declared_modules:
#             if last_module_type in module.parent_type:
#                 compatible_modules.append(module)
#         return compatible_modules

#     # Create multiple instances of the parent node
#     # If num_instances is not specified then create
#     # as many as the number of modules
#     def create_instances(self, num_instances=None):
#         """
#         Returns instances of the a set of same class objects that can be used to expand
#         the robot architecture with different modules in each one of them

#         :param num_instances: None, number of instances to be created
#         """
#         if num_instances == None:
#             num_instances = self.num_modules
#         node_instances = []
#         for i in range(num_instances):
#             # Makes instances of whatever is present in the parent node
#             node_instance = copy.deepcopy(self.current_parent_node)
#             module_name = 'node'+str(self.depth)+str(i)
#             # Change instance name
#             node_instance.name = module_name
#             # Change URDF name
#             node_instance.robot.name = module_name
#             node_instances.append(node_instance)
#         return node_instances


#     # Expand corresponding nodes of robots based on compatible modules
#     def expand_node(self):
#         """
#         Returns number of instances of the parent node equal to the number of
#         compatible modules
#         """
#         # Get compatible nodes first
#         compatible_modules = self.get_compatible_modules()
#         self.num_child_nodes = len(compatible_modules)
#         # Now create required number of instances
#         node_instances = self.create_instances(self.num_child_nodes)
#         # Add each module to each new instance
#         for i in range(self.num_child_nodes):
#             node_instances[i].add_module(compatible_modules[i])
#             # # Add an eef link at the end
#             # temp_node_instance = copy.deepcopy(node_instances[i])
#             # temp_node_instance.add_eef()
#             # self.children_path.append(
#             #     temp_node_instance.constructURDF(self.node_path))

#         self.child_nodes = node_instances
#         # Increase the depth of the tree
#         self.depth += 1

#     # compute path costs based on modules selected
#     def compute_node_mass(self, node):
#         """
#         Returns the mass of the selected node, i.e., the sum of masses of
#         all associated modules

#         :param node: The selected node
#         """
#         mod_list = list(node.tree.keys())
#         mass = 0
#         for mod in mod_list:
#             mass+=node.tree[mod][0].mass
#         return mass

#     # compute path costs based on modules selected
#     def compute_path_cost(self, node):
#         """
#         Returns the path cost of the selected node, i.e., the cost associated
#         with the selected modules

#         :param node: The selected node
#         """
#         mod_list = list(node.tree.keys())
#         path_cost = 0
#         for mod in mod_list:
#             path_cost+=node.tree[mod][0].cost
#         return path_cost

#     # This function is from v0.2.0 and is depricated in v0.3.0
#     #     # Use this for the combined cost in A*
#     # def get_node_cost(self, compute_heuristic_cost, node_instance, node_path):
#     #     g_cost = self.compute_path_cost(node_instance)
#     #     # h_cost, end_flag = self.compute_heuristic_cost(node_path, node_instance.name)
#     #     # Making the heuristic function external
#     #     h_cost, end_flag = compute_heuristic_cost(node_path, node_instance.name)
#     #     f_cost = g_cost+h_cost
#     #     return f_cost, end_flag

#     # Use this for the combined cost in A*
#     def get_node_cost(self, compute_heuristic_cost=None, node_instance=None, gh_weighting=[1,1]):
#         g_cost = self.compute_path_cost(node_instance)
#         # Making the heuristic function external
#         h_cost, end_flag = compute_heuristic_cost(node_instance = node_instance)
#         f_cost = g_cost*gh_weighting[0]+h_cost*gh_weighting[1]
#         print(g_cost, h_cost)
#         return f_cost, end_flag

#         # Use this for requirements based depth first expansion
#     def get_node_costs(self, compute_heuristic_cost, node_instance, node_path):
#         # cost associated with the modules or N_realised
#         g_cost = self.compute_path_cost(node_instance)
#         # h_cost, end_flag = self.compute_heuristic_cost(node_path, node_instance.name)
#         # Making the heuristic function external
#         h_cost, res = compute_heuristic_cost(node_path, node_instance.name)
#         # f_cost = g_cost+100*h_cost
#         return g_cost, h_cost, res

#     # TO BE DEPRECIATED SOON!!!-> REPLACED BY GET_BEST_CHILD
#     def get_best_child_index(self):
#         """
#         Returns the object and the index of the best child node based on the cost function defined
#         """
#         self.children_cost = []
#         for ii in range(self.num_child_nodes):
#             self.children_cost.append(
# #                 self.compute_heuristic_cost(self.children_path[ii], self.child_nodes[ii].name)+self.compute_path_cost(self.child_nodes[ii]))
#             self.compute_heuristic_cost(self.children_path[ii], self.child_nodes[ii].name)+
#             self.compute_path_cost(self.child_nodes[ii])
#             )
#         # print(self.children_cost)
#         # Get the best child index
#         best_child_index = self.children_cost.index(np.min(self.children_cost))
#         return best_child_index

#     # TODO: Accept the cost function as input in the next iteration
#     def get_best_child(self):
#         """
#         Returns the index of the best child node based on the cost function defined
#         """
#         self.children_cost = []
#         for ii in range(self.num_child_nodes):
#             h_cost, end_flag = self.compute_heuristic_cost(self.children_path[ii], self.child_nodes[ii].name)
#             g_cost = self.compute_path_cost(self.child_nodes[ii])
#             self.children_cost.append(h_cost+g_cost)
#         # Get the best child index
#         best_child_index = self.children_cost.index(np.min(self.children_cost))
#         # Update the current_parent now
#         # self.current_parent_node = self.child_nodes[best_child_index]
#         return self.child_nodes[best_child_index], best_child_index, end_flag

#     def get_best_child_v2(self, compute_heuristic_cost):
#         """
#         Returns the index of the best child node based on the cost function defined
#         """
#         self.children_cost = []
#         for ii in range(self.num_child_nodes):
#             h_cost, end_flag = compute_heuristic_cost(self.children_path[ii], self.child_nodes[ii].name)
#             g_cost = self.compute_path_cost(self.child_nodes[ii])
#             self.children_cost.append(h_cost+g_cost)
#         # Get the best child index
#         best_child_index = self.children_cost.index(np.min(self.children_cost))
#         # Update the current_parent now
#         # self.current_parent_node = self.child_nodes[best_child_index]
#         return self.child_nodes[best_child_index], best_child_index, end_flag

#     # Default visualisation is set to False
#     def computeOptimalDesign(self, visualise_search = False):
#         """
#         TBD
#         Returns the optimal design (composition of modules) using the prescribed search algorithm,
#         with respect to the cost function defined

#         :param visualise_search: Visualise the search procedure in the meshcat server
#         """
#         pass

# # -----------------------------------------------------------------
# # Below this are task specific functions needs to be set for DoD
# # -----------------------------------------------------------------

#     # Task specific cost of a given child
# #     def compute_heuristic_cost(self, urdf_path, model_name):
# #         """
# #         Returns the heuristic cost of the selected module.
# #         This function needs to be modified according to the task definition and
# #         hence should be overridden for a required problem.
# #
# #         :param urdf_path: The path of the saved node
# #         :param model_name: The name of the node
# #         """
# #         builder = DiagramBuilder()
# #         plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
# #         model = Parser(plant, scene_graph).AddModelFromFile(urdf_path, model_name)
# #         # Weld the robot to the world frame
# #         plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(model)[0]).body_frame())
# #         # We can sample end-points on the surface of the sphere
# # #         cart0 = np.array([-0.3, 0.4, 0.3])
# #         plant.Finalize()
# #         # Select the last frame as the eef_frame
# #         gripper_frame = plant.GetBodyByName("eef").body_frame()
# #         diagram = builder.Build()
# #         context = diagram.CreateDefaultContext()
# #         plant_context = plant.GetMyMutableContextFromRoot(context)
# #         q0 = plant.GetPositions(plant_context)
# #
# #         # ADD POSITIONS HERE!!!
# #         cart0 = np.array([0.15, 0.1, 0.45])
# #         cartf = np.array([-0.05, 0.35, 0.3])
# #
# #         # Position-1
# #         ik = InverseKinematics(plant, plant_context)
# #         ik.AddPositionConstraint(
# #                     gripper_frame, [0, 0, 0], plant.world_frame(),
# #                     cart0, cart0)
# #         prog = ik.get_mutable_prog()
# #         q = ik.q()
# #         prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
# #         prog.SetInitialGuess(q, q0)
# #         result = Solve(ik.prog())
# #         res1 = result.is_success()
# #         qr = result.GetSolution(ik.q())
# #         # Bring it back to -pi and pi
# #         qr = (np.arctan2(np.sin(qr), np.cos(qr)))
# #         plant.SetPositions(plant.GetMyContextFromRoot(context),model,qr)
# #         eef_pos_realised = plant.GetBodyByName('eef').EvalPoseInWorld(plant_context).translation()
# #         cost1 = np.linalg.norm(eef_pos_realised-cart0)
# #         # Position-2
# #         ik = InverseKinematics(plant, plant_context)
# #         ik.AddPositionConstraint(
# #                     gripper_frame, [0, 0, 0], plant.world_frame(),
# #                     cartf, cartf)
# #         prog = ik.get_mutable_prog()
# #         q = ik.q()
# #         prog.AddQuadraticErrorCost(np.identity(len(q)), qr, q)
# #         prog.SetInitialGuess(q, qr)
# #         result = Solve(ik.prog())
# #         res2 = result.is_success()
# #         qr = result.GetSolution(ik.q())
# #         plant.SetPositions(plant.GetMyContextFromRoot(context),model,qr)
# #         eef_pos_realised = plant.GetBodyByName('eef').EvalPoseInWorld(plant_context).translation()
# #         cost2 = np.linalg.norm(eef_pos_realised-cart0)
# #         # Total cost
# #         total_cost = cost1+cost2
# #         # Final result
# #         result = res1 and res2
# #         return total_cost, result

#     # # TODO: MODIFY THIS TO JUST DISPLAY CURRENT PARENT NODE
#     # def publish_node_pose(self, node_instance, meshcat, t_sleep=0.5):
#     #     """
#     #     Displays the node poses given two points in space and returns the URDF location
#     #     Needs modifying in the next code iteration
#     #
#     #     :param node: The object for the type `node` of which the attribute `current_parent_node` will be displayed
#     #     :param meshcat: The meshcat object started
#     #     :param t_sleep: Amount of time to wait between the two IK poses
#     #     """
#     #     # Setup basic diagram and the sphere at the desired location
#     #     builder = DiagramBuilder()
#     #     # Get path of the best child
#     #     # urdf_path = node.children_path[best_child_idx]
#     #     # model_name = node.child_nodes[best_child_idx].robot.name
#     #     # Save the current node URDF and display it
#     #
#     #     temp_node_instance = copy.deepcopy(node_instance.current_parent_node)
#     #     temp_node_instance.add_eef()
#     #     urdf_path = temp_node_instance.constructURDF(node_instance.node_path)
#     #     model_name = temp_node_instance.name
#     #
#     #     plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
#     #     model = Parser(plant, scene_graph).AddModelFromFile(urdf_path, model_name)
#     #     # Weld the robot to the world frame
#     #     plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(model)[0]).body_frame())
#     #
#     #     # Spawn spherical work marker
#     #     sphere1 = Parser(plant, scene_graph).AddModelFromFile('./urdfs/helper/sphere.urdf','sphere1')
#     #     sphere2 = Parser(plant, scene_graph).AddModelFromFile('./urdfs/helper/sphere.urdf','sphere2')
#     #     # We can sample end-points on the surface of the sphere
#     #     cart0 = np.array([0.15, 0.1, 0.45])
#     #     cartf = np.array([-0.05, 0.35, 0.3])
#     #     X_R1 = RigidTransform(cart0)
#     #     X_R2 = RigidTransform(cartf)
#     #     plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(sphere1)[0]).body_frame(), X_R1)
#     #     plant.WeldFrames(plant.world_frame(), plant.get_body(plant.GetBodyIndices(sphere2)[0]).body_frame(), X_R2)
#     #     plant.Finalize()
#     #
#     #     # Select the last frame as the eef_frame
#     #     gripper_frame = plant.GetBodyByName("eef").body_frame()
#     #
#     #     meshcat.Delete()
#     #     meshcat.DeleteAddedControls()
#     #
#     #     visualizer = MeshcatVisualizerCpp.AddToBuilder(builder, scene_graph, meshcat)
#     #
#     #     diagram = builder.Build()
#     #     context = diagram.CreateDefaultContext()
#     #     plant_context = plant.GetMyMutableContextFromRoot(context)
#     #
#     #     q0 = plant.GetPositions(plant_context)
#     #     diagram.Publish(context)
#     #
#     #     # Check design
#     #     ik = InverseKinematics(plant, plant_context)
#     #     ik.AddPositionConstraint(
#     #                 gripper_frame, [0, 0, 0], plant.world_frame(),
#     #                 cart0, cart0)
#     #     prog = ik.get_mutable_prog()
#     #     q = ik.q()
#     #     prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
#     #     prog.SetInitialGuess(q, q0)
#     #     result = Solve(ik.prog())
#     #     qr = result.GetSolution(ik.q())
#     #     qr = (np.arctan2(np.sin(qr), np.cos(qr)))
#     #     # Visualise the best solution
#     #     plant.SetPositions(plant.GetMyContextFromRoot(context),model,qr)
#     #     diagram.Publish(context)
#     #     end_game = result.is_success()
#     #     time.sleep(t_sleep)
#     #     # Check for position-2
#     #     ik = InverseKinematics(plant, plant_context)
#     #     ik.AddPositionConstraint(
#     #                 gripper_frame, [0, 0, 0], plant.world_frame(),
#     #                 cartf, cartf)
#     #     prog = ik.get_mutable_prog()
#     #     q = ik.q()
#     #     prog.AddQuadraticErrorCost(np.identity(len(q)), qr, q)
#     #     prog.SetInitialGuess(q, qr)
#     #     result = Solve(ik.prog())
#     #     qr = result.GetSolution(ik.q())
#     #     # Visualise the best solution
#     #     plant.SetPositions(plant.GetMyContextFromRoot(context),model,qr)
#     #     diagram.Publish(context)
#     #     return urdf_path
