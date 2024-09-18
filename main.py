import numpy as np
from panda3d.core import Vec3
from direct.directbase.DirectStart import *
from direct.showbase.DirectObject import DirectObject
from direct.task import Task
from direct.actor.Actor import Actor
from panda3d.ai import AIWorld, AICharacter

class World(DirectObject):

    def __init__(self):
        base.disableMouse()
        base.cam.setPosHpr(0, 0, 55, 0, -90, 0)

        self.loadModels()
        self.setAI()
        self.setBioCrowds()

    def loadModels(self):
        # Seeker
        ralphStartPos = Vec3(-10, 0, 0)
        self.seeker = Actor("ralph", {"run": "ralph-run"})
        self.seeker.reparentTo(render)
        self.seeker.setScale(0.5)
        self.seeker.setPos(ralphStartPos)
        # Target
        self.target = loader.loadModel("ball")
        self.target.setColor(1, 0, 0)
        self.target.setPos(5, 0, 0)
        self.target.setScale(1)
        self.target.reparentTo(render)

    def setAI(self):
        # Creating AI World
        self.AIworld = AIWorld(render)

        self.AIchar = AICharacter("seeker", self.seeker, 100, 0.05, 5)
        self.AIworld.addAiChar(self.AIchar)
        self.AIbehaviors = self.AIchar.getAiBehaviors()

        self.AIbehaviors.seek(self.target)
        self.seeker.loop("run")

        # AI World update        
        taskMgr.add(self.AIUpdate, "AIUpdate")

    def AIUpdate(self, task):
        self.AIworld.update()
        return Task.cont

    def setBioCrowds(self):
        self.bioCrowds = []  # List to keep track of biocrowds agents
        self.createBioCrowdsAgents()

        # Add biocrowds update task
        taskMgr.add(self.BioCrowdsUpdate, "BioCrowdsUpdate")

    def createBioCrowdsAgents(self):
        for i in range(10):  # Assuming we have 10 biocrowds agents
            agent = loader.loadModel("ralph")
            agent.setPos(Vec3(-15 + i, 0, 0))  # Example starting positions
            agent.reparentTo(render)
            self.bioCrowds.append(agent)

    def BioCrowdsUpdate(self, task):
        # Convert agents to numpy arrays for easier calculation
        agent_positions = np.array([list(agent.getPos()) for agent in self.bioCrowds])
        new_positions = []

        # Parameters
        FPS = 30
        Smax = 10  # Example value
        R = 5      # Interaction radius
        s_imax = 5 # Maximum speed

        # Convert Smax to per frame value
        Smax = Smax / FPS

        for i, agent in enumerate(self.bioCrowds):
            # Initialize Si for each agent
            Si = []
            pi = np.array(agent.getPos())

            for a in self.bioCrowds:
                if a != agent:
                    ai = np.array(a.getPos())
                    distance = np.linalg.norm(pi - ai)

                    if distance <= R:
                        Si.append(a)

            if Si:
                Si = [np.array(a.getPos()) - pi for a in Si]
                distances = np.linalg.norm(Si, axis=1)
                weights = 1 / distances
                weighted_directions = np.sum(weights[:, np.newaxis] * Si, axis=0)
                norm_weighted_directions = np.linalg.norm(weighted_directions)

                s_min = min(norm_weighted_directions, s_imax)
                if norm_weighted_directions > 0:
                    vi = (weighted_directions / norm_weighted_directions) * s_min
                else:
                    vi = np.zeros_like(weightedirections)

                new_positions.append(pi + vi)

        # Update agent positions
        for agent, pos in zip(self.bioCrowds, new_positions):
            agent.setPos(Vec3(*pos))

        return Task.cont

w = World()
run()
