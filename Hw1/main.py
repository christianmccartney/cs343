# OpenNero will execute ModMain when this mod is loaded
import Maze.main
from Maze.module import getMod, MazeEnvironment
import Hw1.agent

def start_idastar():
    """ start the IDA* search demo """
    getMod().start_agent("data/shapes/character/SydneyIDAStar.xml", MazeEnvironment)

def ModMain():
    agent_desc = ('IDA* Search', start_idastar, False, "")
    getMod().AGENTS.append(agent_desc)
    Maze.main.ModMain()

