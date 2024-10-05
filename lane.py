import numpy as np
from panda3d.core import Vec3
from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject
from direct.task import Task
from direct.actor.Actor import Actor

class World(DirectObject):

    def __init__(self):
        base = ShowBase() 
        base.disableMouse()
        base.cam.setPosHpr(0, 0, 55, 0, -90, 0)

        self.loadModels()
        self.setBioCrowds()

    def loadModels(self):
        self.target = loader.loadModel("models/ball")  
        self.target.setColor(0, 0, 10)
        self.target.setPos(10, 0, 0)
        self.target.setScale(3)
        self.target.reparentTo(render)

        self.obstacles = []
        for i in range(4): 
            obstacle = loader.loadModel("models/box")
            obstacle.setColor(1, 1, 1) 
            obstacle.setScale(1.5)
            obstacle.setPos(-0.75, 5.5 * (i - 1), 0) 
            obstacle.reparentTo(render)
            self.obstacles.append(obstacle)
        
        for i in range(4): 
            obstacle = loader.loadModel("models/box")
            obstacle.setColor(1, 1, 1) 
            obstacle.setScale(1.5)
            obstacle.setPos(-1.5, 5.5 * (i - 1), 0) 
            obstacle.reparentTo(render)
            self.obstacles.append(obstacle)
        for i in range(4): 
            obstacle = loader.loadModel("models/box")
            obstacle.setColor(1, 1, 1) 
            obstacle.setScale(1.5)
            obstacle.setPos(-2.25, 5.5 * (i - 1), 0) 
            obstacle.reparentTo(render)
            self.obstacles.append(obstacle)
        for i in range(4): 
            obstacle = loader.loadModel("models/box")
            obstacle.setColor(1, 1, 1) 
            obstacle.setScale(1.5)
            obstacle.setPos(-3, 5.5 * (i - 1), 0) 
            obstacle.reparentTo(render)
            self.obstacles.append(obstacle)

    def setBioCrowds(self):
        self.bioCrowds = []  
        self.createBioCrowdsAgents()
        taskMgr.add(self.BioCrowdsUpdate, "BioCrowdsUpdate")

    def createBioCrowdsAgents(self):
        num_agents = 50 
        for i in range(num_agents):  
            agent = Actor("models/ralph", {"walk": "models/ralph-run"}) 
            # Define lanes for agents
            lane_position = np.random.randint(-5, 6)  # Random lane position
            agent.setPos(Vec3(np.random.uniform(-20, -10), lane_position, 0))
            agent.setScale(0.5)
            agent.reparentTo(render)
            self.bioCrowds.append(agent)
            agent.loop("walk") 

    def BioCrowdsUpdate(self, task):
        new_positions = []
        agent_radius = 0.8 
        FPS = 30
        Smax = 1.0 
        R = 3.0    
        s_imax = 1.0 
        Smax = Smax / FPS

        for agent in self.bioCrowds:
            pi = np.array(agent.getPos())
            avoidance_vector = np.zeros(3)
            pursuit_vector = np.zeros(3)

            # Avoidance from obstacles
            for obstacle in self.obstacles:
                obstacle_pos = np.array(obstacle.getPos())
                distance = np.linalg.norm(pi - obstacle_pos)
                if distance < R:
                    evasion_direction = pi - obstacle_pos
                    if np.linalg.norm(evasion_direction) > 0:
                        evasion_direction = evasion_direction / np.linalg.norm(evasion_direction)
                    avoidance_vector += evasion_direction * (R - distance) / R 

            if np.linalg.norm(avoidance_vector) > 0:
                avoidance_vector = avoidance_vector / np.linalg.norm(avoidance_vector) * Smax

            # Pursuit towards the target
            target_pos = np.array(self.target.getPos())
            direction_to_target = target_pos - pi
            distance_to_target = np.linalg.norm(direction_to_target)

            if distance_to_target > 0:
                direction_to_target = direction_to_target / distance_to_target
                pursuit_vector = direction_to_target * min(s_imax, Smax)

            # Final movement vector
            final_vector = pursuit_vector + avoidance_vector

            current_pos = np.array(agent.getPos())
            smooth_vector = final_vector * 0.7 + (current_pos - pi) * 0.3
            new_position = current_pos + smooth_vector
            
            # Limit positions to stay within the screen
            new_position[0] = np.clip(new_position[0], -20, 10)  # Limiting x positions
            new_position[1] = np.clip(new_position[1], -10, 10)  # Limiting y positions
            
            new_positions.append(new_position)

        # Collision detection and adjustment
        for i, (agent, pos) in enumerate(zip(self.bioCrowds, new_positions)):
            collision_detected = False
            
            # Check for collision with obstacles
            for obstacle in self.obstacles:
                obstacle_pos = np.array(obstacle.getPos())
                if np.linalg.norm(pos - obstacle_pos) < 1.5 * agent_radius: 
                    pos = pos + (pos - obstacle_pos) / np.linalg.norm(pos - obstacle_pos) * agent_radius
                    collision_detected = True
            
            # Check for collision with other agents
            for j, other_agent in enumerate(self.bioCrowds):
                if i != j:
                    other_pos = np.array(other_agent.getPos())
                    if np.linalg.norm(pos - other_pos) < 1.5 * agent_radius:  
                        pos = pos + (pos - other_pos) / np.linalg.norm(pos - other_pos) * agent_radius
                        collision_detected = True
            
            # Set new position for the agent
            agent.setPos(Vec3(*pos))

            # Check if agent has reached the target
            if np.linalg.norm(pos - target_pos) < 2.0:  # Define threshold for reaching the target
                agent.removeNode()  # Remove agent from the scene
                self.bioCrowds.pop(i)  # Remove from the list
                return Task.cont  # Stop processing this agent

        return Task.cont

w = World()
run()
