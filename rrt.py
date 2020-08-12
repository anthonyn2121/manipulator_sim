import pybullet as p 
import pybullet_data
import numpy as np
import time 



UR5_JOINT_INDICES = [0, 1, 2]



def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id


def distMeasure(Qnew, Qold, type='euclidean'): 
    if type == 'euclidean': 
        return np.linalg.norm(np.asarray(Qnew)-np.asarray(Qold)) 
     
        

def extended_rrt(tree, Qrand): 
    # find nearest node in graph 
    dist = float('inf')
    for node in tree: 
        curr_dist = distMeasure(Qnew=Qrand, Qold=node, type='euclidean')
        if curr_dist < dist: 
            dist = curr_dist
            q_near = node # identify nearest node in tree 
    
    direction_vector = (np.asarray(Qrand) - np.asarray(q_near))/np.linalg.norm(np.asarray(Qrand) - np.asarray(q_near)) # unit vector to step in direction of 
    delta = .75 # difference 
    q_new = q_near + delta*direction_vector # progress q_near by stepsize along straight line between q_near and q_rand 
    q_new = q_new.tolist() # convert from numpy array to list 
    # check for collision 
    if collision_fn(q_new):
        pass 
    else: 
        set_joint_positions(ur5, UR5_JOINT_INDICES, q_near) # go to the nearest node 
        set_joint_positions(ur5, UR5_JOINT_INDICES, q_new)  # go to the new node 

    return q_near, q_new 


def find_path(graph, start, end, path=[]): 
    path = path + [start]
    print('start: ',start)
    if start == end: 
        return path 

    for node in graph[tuple(start)]: 
        # print(node)
        # node = tuple(node)
        if node not in path: 
            next_path = find_path(graph, node, end, path)
            if next_path: 
                return next_path


def rrt(max_iters, start_config, end_config): 
    counter = 0
    graph = {}
    graph_list = []
    graph_list.append(list(start_config)) 
    while counter <= max_iters:
        counter += 1  
        # randomly generate configuration q_new  
        joint1 = np.random.uniform(-np.pi, np.pi, 1)
        joint2 = np.random.uniform(-2*np.pi, 2*np.pi, 1)
        joint3 = np.random.uniform(-2*np.pi, 2*np.pi, 1)
        q_rand = [joint1[0], joint2[0], joint3[0]]
        
        # check for collision w/ obstacle or self-collision 
        if collision_fn(q_rand): 
            continue
        
        # find a new node in the tree 
        q_near, q_new = extended_rrt(graph_list, q_rand)

        if q_new is not None: 
            graph_list.append(q_new)
            graph[tuple(q_near)] = (q_new)

            dist_to_goal = distMeasure(q_new, end_config)
            if dist_to_goal <= 1: 
                graph_list.append(end_config)
                graph[tuple(q_new)] = (end_config)

                path_config = find_path(graph, start_config, end_config)
                
                return path_config

    pass 

if __name__=="__main__": 
    # set up simulator 
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))


    # load objects 
    plane = p.loadURDF('plane.urdf')
    ur5 = p.loadURDF('assets/ur5/ur5.urdf', basePosition=[0, 0, 0.02], useFixedBase=True)
    obstacle1 = p.loadURDF('assets/block.urdf',
                           basePosition=[1/4, 0, 1/2],
                           useFixedBase=True)
    obstacle2 = p.loadURDF('assets/block.urdf',
                           basePosition=[2/4, 0, 2/3],
                           useFixedBase=True)
    obstacles = [plane, obstacle1, obstacle2] 

    # start and goal 
    start_conf = (-0.813358794499552, -0.37120422397572495, -0.754454729356351)
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    goal_conf = (0.7527214782907734, -0.6521867735052328, -0.4949270744967443)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    goal_marker = draw_sphere_marker(position=goal_position, radius=0.02, color=[1, 0, 0, 1])
    set_joint_positions(ur5, UR5_JOINT_INDICES, start_conf)

    from collision_utils import get_collision_fn
    collision_fn = get_collision_fn(ur5, UR5_JOINT_INDICES, obstacles=obstacles,
                                       attachments=[],   self_collisions=True,
                                       disabled_collisions=set())

    # print('hi')
    path_config = rrt(max_iters=1000, start_config=start_conf, end_config=goal_conf)

    if path_config:
        for q in path_config: 
            set_joint_positions(ur5, UR5_JOINT_INDICES, q)
            time.sleep(1)
    
    # p.disconnect()