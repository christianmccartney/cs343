from OpenNero import *
from common import *

import Maze
from Maze.constants import *
import Maze.agent
from Maze.agent import *


class IdaStarSearchAgent(SearchAgent):

    """
    Depth first search implementation
    """

    def __init__(self):
        """
        A new Agent
        """
        # this line is crucial, otherwise the class is not recognized as an
        # AgentBrainPtr by C++
        SearchAgent.__init__(self)
        # from both Maze/agent.py class DFSSearchAgent and class
        # GenericSearchAlgorithm
        self.visited = set([])
        self.adjlist = {}
        self.parents = {}
        self.queue = []  # queue of cells to visit (front)
        self.backpointers = {}
        self.starting_pos = None
        self.goal = None  # we have no idea where to go at first
        self.heuristic = {(0, 0): manhattan_heuristic(0, 0)}
        self.length = {}
        self.limit = 16

    # from Maze/agent.py class GenericSearchAlgorithm
    def get_next_step(self, r1, c1, r2, c2):
        """
        return the next step when trying to get from current r1,c1 to target r2,c2
        """
        back2 = []  # cells between origin and r2,c2
        r, c = r2, c2  # back track from target
        while (r, c) in self.backpointers:
            r, c = self.backpointers[(r, c)]
            if (r1, c1) == (r, c):  # if we find starting point, we need to move forward
                return back2[-1]  # return the last step
            back2.append((r, c))
        return self.backpointers[(r1, c1)]

    # from Maze/agent.py class DFSSearchAgent and class GenericSearchAlgorithm
    def dfs_action(self, observations):
        r = observations[0]
        c = observations[1]
        current_cell = (r, c)
        goal_cell = None
        cost = sys.maxsize
        # if we have not been here before, build a list of other places we can
        # go
        if current_cell not in self.visited:
            tovisit = []
            for m, (dr, dc) in enumerate(MAZE_MOVES):
                r2, c2 = r + dr, c + dc
                if not observations[2 + m]:  # can we go that way?
                    if (r2, c2) not in self.visited:
                        tovisit.append((r2, c2))
                        self.parents[(r2, c2)] = current_cell
                        self.backpointers[(r2, c2)] = current_cell
                        self.queue.insert(0, (r2, c2))
                        self.length[(r2, c2)] = self.length[current_cell] + 1
                        self.heuristic[(r2, c2)] = manhattan_heuristic(
                            r2, c2) + self.length[(r2, c2)]
                        get_environment().mark_maze_yellow(r2, c2)

            # remember the cells that are adjacent to this one
            self.adjlist[current_cell] = tovisit
        # if we have been here before, check if we have other places to visit
        adjlist = self.adjlist[current_cell]
        if self.goal == current_cell:
            self.goal = None
        if not self.goal:
            for cell in self.queue:
                goal_cell = self.queue.pop(0)
                if(cost >= self.heuristic[(goal_cell[0], goal_cell[1])]):
                    cost = self.heuristic[(goal_cell[0], goal_cell[1])]
                if self.heuristic[(goal_cell[0], goal_cell[1])] <= self.limit:
                    self.goal = goal_cell
                    break
                else:
                    self.queue.append(goal_cell)
            self.limit = cost

        if not self.goal:
            self.limit = cost
            for cell in self.queue:
                goal_cell = self.queue.pop(0)
                if self.heuristic[(goal_cell[0], goal_cell[1])] <= self.limit:
                        self.goal = goal_cell
                        break
                else:
                    self.queue.append(goal_cell)

        self.visited.add(current_cell)
        if current_cell != self.starting_pos:
            get_environment().mark_maze_blue(
                r, c)  # mark it as blue on the maze

        dr, dc = self.goal[0] - r, self.goal[1] - c

        # from GenericSearchAlgorithm
        action = get_action_index((dr, dc))
        v = self.constraints.get_instance()  # make the action vector to return
        # first, is the node reachable in one action?
        if action is not None and observations[2 + action] == 0:
            v[0] = action  # if yes, do that action!
        else:
            # if not, we have to figure out a path to get there from the
            # backtracking dictionary
            (r2, c2) = self.get_next_step(r, c, self.goal[0], self.goal[1])
            dr, dc = r2 - r, c2 - c  # how to get back there?
            v[0] = get_action_index((dr, dc))  # what action is that?

        get_environment().mark_maze_white(
            self.starting_pos[0], self.starting_pos[1])
        return v

    def initialize(self, init_info):
        self.constraints = init_info.actions
        return True

    def start(self, time, observations):
        # return action
        self.starting_pos = (observations[0], observations[1])
        self.length = {(observations[0], observations[1]): 0}
        return self.dfs_action(observations)

    def reset(self):
        self.visited = set([])
        self.parents = {}
        self.backpointers = {}
        self.starting_pos = None
        self.adjlist = {}
        self.queue = []
        self.goal = None
        self.heuristic = {(0, 0): manhattan_heuristic(0, 0)}
        self.length = {}
        self.limit = 16

    def act(self, time, observations, reward):
        # return action
        return self.dfs_action(observations)

    def end(self, time, reward):
        print "Final reward: %f, cumulative: %f" % (reward[0], self.fitness[0])
        self.reset()
        return True

    def mark_path(self, r, c):
        get_environment().mark_maze_white(r, c)
