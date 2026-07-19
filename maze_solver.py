import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- MAZE CONFIGURATION ---
# 0 = Empty Path, 1 = Wall/Obstacle
MAZE = np.array([
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 0, 1, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 1, 1, 0, 1, 0],
    [0, 1, 1, 1, 0, 1, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1, 1, 1, 1, 0],
    [1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 1, 1, 1, 0, 0, 0]
])

START = (0, 0)
GOAL = (9, 9)

class AStarMazeSolver:
    def __init__(self, maze, start, goal):
        self.maze = maze
        self.rows, self.cols = maze.shape
        self.start = start
        self.goal = goal
        
        # Track steps for visualization history
        self.visited_history = []
        self.final_path = []

    def heuristic(self, a, b):
        """Manhattan distance heuristic function."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def solve(self):
        """Executes the A* Search algorithm and logs history."""
        # priority queue elements: (f_score, g_score, current_node)
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(self.start, self.goal), 0, self.start))
        
        came_from = {}
        g_score = {self.start: 0}
        closed_set = set()

        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            if current in closed_set:
                continue
            
            closed_set.add(current)
            self.visited_history.append(current)

            # Check if goal is reached
            if current == self.goal:
                self.reconstruct_path(came_from, current)
                return True

            # Explore 4 cardinal neighbors (Up, Down, Left, Right)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)

                # Validate boundaries and check for obstacles
                if 0 <= neighbor[0] < self.rows and 0 <= neighbor[1] < self.cols:
                    if self.maze[neighbor[0]][neighbor[1]] == 1:
                        continue
                    
                    tentative_g = current_g + 1
                    
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + self.heuristic(neighbor, self.goal)
                        heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                        
        return False

    def reconstruct_path(self, came_from, current):
        """Backtracks from the goal to the start to compile the path."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        self.final_path = path[::-1]

    def visualize(self):
        """Creates an animated simulation of the search and path mapping."""
        # 0: Empty, 1: Wall, 2: Explored, 3: Final Path, 4: Start/Goal
        display_grid = np.copy(self.maze).astype(float)
        
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_title("Maze Solver: A* Search Simulation")
        
        # Color mapping configuration
        # Walls = Black, Paths = White, Exploration = Light Blue, Final Path = Gold
        cmap = plt.cm.colors.ListedColormap(['#FFFFFF', '#1A1A1A', '#A0CED9', '#FFD700', '#FF4D4D'])
        bounds = [0, 0.9, 1.9, 2.9, 3.9, 5]
        norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
        
        # Mark target reference bounds
        display_grid[self.start] = 4
        display_grid[self.goal] = 4
        
        img = ax.imshow(display_grid, cmap=cmap, norm=norm)
        
        # Total frames match search iterations plus path construction steps
        total_frames = len(self.visited_history) + len(self.final_path)

        def update(frame):
            if frame < len(self.visited_history):
                # Search Phase: color the node being explored
                node = self.visited_history[frame]
                if node != self.start and node != self.goal:
                    display_grid[node] = 2
            else:
                # Path Mapping Phase: trace out the final shortest path
                path_frame = frame - len(self.visited_history)
                node = self.final_path[path_frame]
                if node != self.start and node != self.goal:
                    display_grid[node] = 3
            
            img.set_array(display_grid)
            return [img]

        ani = animation.FuncAnimation(
            fig, update, frames=total_frames, interval=80, blit=True, repeat=False
        )
        plt.grid(True, which='both', color='#CCCCCC', linestyle='-', linewidth=0.5)
        ax.set_xticks(np.arange(-0.5, self.cols, 1), [])
        ax.set_yticks(np.arange(-0.5, self.rows, 1), [])
        plt.savefig("maze_output.png", dpi=300, bbox_inches="tight") 
        plt.show()

# --- RUN SOLVER ---
solver = AStarMazeSolver(MAZE, START, GOAL)
if solver.solve():
    print(f"Path Successfully Found! Length: {len(solver.final_path)} steps.")
    solver.visualize()
else:
    print("No valid path exists through this maze.")
