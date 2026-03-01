import tkinter as tk
from tkinter import ttk, messagebox
import random
import math
import time
import heapq                 # priority queue – part of Python standard library

CLR_BG          = "#1a1a2e"
CLR_PANEL       = "#16213e"
CLR_ACCENT      = "#0f3460"
CLR_BTN         = "#e94560"
CLR_BTN_TXT     = "#ffffff"
CLR_EMPTY       = "#0d1117"
CLR_WALL        = "#4a4a6a"
CLR_START       = "#00d4ff"
CLR_GOAL        = "#ff6b35"
CLR_FRONTIER    = "#ffd700"   # yellow  – in priority queue
CLR_VISITED     = "#6c63ff"   # purple  – already expanded
CLR_PATH        = "#39ff14"   # green   – final path
CLR_AGENT       = "#00d4ff"
CLR_GRID_LINE   = "#1e1e3a"
CLR_TEXT        = "#e0e0e0"
CLR_SUBTEXT     = "#8888aa"


#  HEURISTIC FUNCTIONS
def manhattan(a, b):
    """
    D = |x1 - x2| + |y1 - y2|
    Good for 4-directional grids.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a, b):
    """
    D = sqrt( (x1-x2)^2 + (y1-y2)^2 )
    Straight-line distance.
    """
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


#  SEARCH ALGORITHMS
def greedy_bfs(grid, start, goal, heuristic_fn, rows, cols):
    # Each entry in priority queue: (priority, node)
    open_set = []
    heapq.heappush(open_set, (heuristic_fn(start, goal), start))

    came_from = {start: None}   # to reconstruct path
    visited   = []              # order of expansion

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, goal), visited, len(visited)

        visited.append(current)

        for neighbor in get_neighbors(current, grid, rows, cols):
            if neighbor not in came_from:
                came_from[neighbor] = current
                priority = heuristic_fn(neighbor, goal)
                heapq.heappush(open_set, (priority, neighbor))

    return None, visited, len(visited)   # no path found


def a_star(grid, start, goal, heuristic_fn, rows, cols):
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {start: None}
    g_cost    = {start: 0}     # actual cost from start
    visited   = []

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, goal), visited, len(visited)

        visited.append(current)

        for neighbor in get_neighbors(current, grid, rows, cols):
            new_g = g_cost[current] + 1   # each step has cost 1

            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                g_cost[neighbor]    = new_g
                came_from[neighbor] = current
                f = new_g + heuristic_fn(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))

    return None, visited, len(visited)


def get_neighbors(node, grid, rows, cols):
    """Return valid (non-wall) 4-directional neighbors."""
    r, c = node
    directions = [(-1,0),(1,0),(0,-1),(0,1)]   # up, down, left, right
    neighbors  = []
    for dr, dc in directions:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != 1:
            neighbors.append((nr, nc))
    return neighbors


def reconstruct_path(came_from, goal):
    """Trace back from goal to start using came_from dict."""
    path    = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


#  MAIN APPLICATION
class PathfindingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dynamic Pathfinding Agent")
        self.root.configure(bg=CLR_BG)
        self.root.resizable(True, True)

        # ── default settings ──
        self.rows     = 20
        self.cols     = 20
        self.cell_sz  = 30         # pixel size of each cell

        # grid[r][c]:  0 = empty,  1 = wall
        self.grid     = [[0]*self.cols for _ in range(self.rows)]

        self.start    = (0, 0)
        self.goal     = (self.rows-1, self.cols-1)

        # algorithm / heuristic selections
        self.algo_var      = tk.StringVar(value="A*")
        self.heuristic_var = tk.StringVar(value="Manhattan")

        # dynamic mode
        self.dynamic_mode     = False
        self.dynamic_interval = 300    # ms between obstacle spawns
        self.obs_prob         = 0.05   # 5 % chance per cell per tick

        # agent animation
        self.agent_path       = []
        self.agent_idx        = 0
        self.agent_running    = False
        self.dynamic_job      = None
        self.move_job         = None

        # placing mode for start/goal
        self.place_mode = None   # None | "start" | "goal"

        # metrics
        self.nodes_visited  = 0
        self.path_cost      = 0
        self.exec_time_ms   = 0

        # cell colour state for canvas drawing
        self.cell_state     = {}   # (r,c) -> colour tag

        self._build_ui()
        self._init_grid()

    # ─────────────────── UI CONSTRUCTION ────────────────────
    def _build_ui(self):
        # ── left: canvas ──
        self.canvas_frame = tk.Frame(self.root, bg=CLR_BG)
        self.canvas_frame.pack(side=tk.LEFT, padx=10, pady=10)

        canvas_w = self.cols * self.cell_sz
        canvas_h = self.rows * self.cell_sz
        self.canvas = tk.Canvas(
            self.canvas_frame, width=canvas_w, height=canvas_h,
            bg=CLR_EMPTY, highlightthickness=2,
            highlightbackground=CLR_ACCENT
        )
        self.canvas.pack()
        self.canvas.bind("<Button-1>",        self._on_click)
        self.canvas.bind("<B1-Motion>",       self._on_drag)

        # ── right: control panel ──
        panel = tk.Frame(self.root, bg=CLR_PANEL, width=260)
        panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(0,10), pady=10)
        panel.pack_propagate(False)

        self._section(panel, "⚙  GRID SETTINGS")

        # rows / cols inputs
        rc_frame = tk.Frame(panel, bg=CLR_PANEL)
        rc_frame.pack(fill=tk.X, padx=10, pady=2)
        tk.Label(rc_frame, text="Rows:", bg=CLR_PANEL, fg=CLR_TEXT,
                 font=("Courier",10)).grid(row=0, column=0, sticky="w")
        self.rows_var = tk.IntVar(value=self.rows)
        tk.Spinbox(rc_frame, from_=5, to=50, textvariable=self.rows_var,
                   width=5, bg=CLR_ACCENT, fg=CLR_TEXT,
                   buttonbackground=CLR_ACCENT).grid(row=0, column=1, padx=5)
        tk.Label(rc_frame, text="Cols:", bg=CLR_PANEL, fg=CLR_TEXT,
                 font=("Courier",10)).grid(row=0, column=2, sticky="w")
        self.cols_var = tk.IntVar(value=self.cols)
        tk.Spinbox(rc_frame, from_=5, to=50, textvariable=self.cols_var,
                   width=5, bg=CLR_ACCENT, fg=CLR_TEXT,
                   buttonbackground=CLR_ACCENT).grid(row=0, column=3, padx=5)

        self._btn(panel, "Apply Grid Size",   self._apply_grid_size)

        # obstacle density
        tk.Label(panel, text="Obstacle Density (%):", bg=CLR_PANEL,
                 fg=CLR_SUBTEXT, font=("Courier",9)).pack(anchor="w", padx=12)
        self.density_var = tk.IntVar(value=30)
        tk.Scale(panel, from_=0, to=70, orient=tk.HORIZONTAL,
                 variable=self.density_var, bg=CLR_PANEL, fg=CLR_TEXT,
                 troughcolor=CLR_ACCENT, highlightthickness=0
                 ).pack(fill=tk.X, padx=10)

        self._btn(panel, "🎲 Random Map",     self._random_map)
        self._btn(panel, "🗑  Clear Map",      self._clear_map)

        self._section(panel, "📍 PLACE START / GOAL")
        self._btn(panel, "Set Start (click cell)", self._mode_start, color="#00d4ff")
        self._btn(panel, "Set Goal  (click cell)", self._mode_goal,  color=CLR_GOAL)

        self._section(panel, "🧠 ALGORITHM")
        for algo in ["A*", "Greedy BFS"]:
            tk.Radiobutton(panel, text=algo, variable=self.algo_var,
                           value=algo, bg=CLR_PANEL, fg=CLR_TEXT,
                           selectcolor=CLR_ACCENT, activebackground=CLR_PANEL,
                           font=("Courier",10)).pack(anchor="w", padx=16)

        self._section(panel, "📐 HEURISTIC")
        for h in ["Manhattan", "Euclidean"]:
            tk.Radiobutton(panel, text=h, variable=self.heuristic_var,
                           value=h, bg=CLR_PANEL, fg=CLR_TEXT,
                           selectcolor=CLR_ACCENT, activebackground=CLR_PANEL,
                           font=("Courier",10)).pack(anchor="w", padx=16)

        self._section(panel, "▶  RUN")
        self._btn(panel, "▶ Find Path (Static)",  self._run_static)
        self._btn(panel, "🤖 Run Agent (Animate)", self._run_agent)

        self._section(panel, "⚡ DYNAMIC MODE")
        self.dyn_btn = self._btn(panel, "Enable Dynamic Obstacles",
                                  self._toggle_dynamic, color="#ff6b35")
        tk.Label(panel, text="Spawn Probability (%):", bg=CLR_PANEL,
                 fg=CLR_SUBTEXT, font=("Courier",9)).pack(anchor="w", padx=12)
        self.prob_var = tk.IntVar(value=5)
        tk.Scale(panel, from_=1, to=30, orient=tk.HORIZONTAL,
                 variable=self.prob_var, bg=CLR_PANEL, fg=CLR_TEXT,
                 troughcolor=CLR_ACCENT, highlightthickness=0
                 ).pack(fill=tk.X, padx=10)

        self._btn(panel, "⏹  Stop",  self._stop_agent)

        self._section(panel, "📊 METRICS")
        self.lbl_nodes = self._metric(panel, "Nodes Visited", "0")
        self.lbl_cost  = self._metric(panel, "Path Cost",     "0")
        self.lbl_time  = self._metric(panel, "Time (ms)",     "0.00")

        self._section(panel, "🗺  LEGEND")
        legend = [
            (CLR_START,    "Start"),
            (CLR_GOAL,     "Goal"),
            (CLR_WALL,     "Wall"),
            (CLR_FRONTIER, "Frontier"),
            (CLR_VISITED,  "Visited"),
            (CLR_PATH,     "Path"),
        ]
        for clr, lbl in legend:
            row = tk.Frame(panel, bg=CLR_PANEL)
            row.pack(anchor="w", padx=12, pady=1)
            tk.Label(row, bg=clr, width=3).pack(side=tk.LEFT)
            tk.Label(row, text=f" {lbl}", bg=CLR_PANEL, fg=CLR_TEXT,
                     font=("Courier",9)).pack(side=tk.LEFT)

        self.status_var = tk.StringVar(value="Ready")
        tk.Label(panel, textvariable=self.status_var, bg=CLR_PANEL,
                 fg=CLR_BTN, font=("Courier",9,"bold"),
                 wraplength=230).pack(pady=6, padx=10)

    def _section(self, parent, title):
        tk.Label(parent, text=title, bg=CLR_PANEL, fg=CLR_SUBTEXT,
                 font=("Courier",9,"bold")).pack(anchor="w", padx=10, pady=(10,2))
        tk.Frame(parent, bg=CLR_ACCENT, height=1).pack(fill=tk.X, padx=10)

    def _btn(self, parent, text, cmd, color=None):
        color = color or CLR_BTN
        b = tk.Button(parent, text=text, command=cmd, bg=color,
                      fg=CLR_BTN_TXT, font=("Courier",9,"bold"),
                      relief=tk.FLAT, cursor="hand2",
                      activebackground=CLR_BG, activeforeground=CLR_BTN_TXT,
                      padx=4, pady=4)
        b.pack(fill=tk.X, padx=10, pady=3)
        return b

    def _metric(self, parent, label, val):
        f = tk.Frame(parent, bg=CLR_PANEL)
        f.pack(fill=tk.X, padx=10, pady=1)
        tk.Label(f, text=label+":", bg=CLR_PANEL, fg=CLR_SUBTEXT,
                 font=("Courier",9)).pack(side=tk.LEFT)
        var = tk.StringVar(value=val)
        tk.Label(f, textvariable=var, bg=CLR_PANEL, fg=CLR_TEXT,
                 font=("Courier",10,"bold")).pack(side=tk.RIGHT)
        return var

    # ─────────────────── GRID MANAGEMENT ────────────────────
    def _init_grid(self):
        """Draw empty grid on canvas."""
        self.canvas.delete("all")
        self.cell_state.clear()
        self._draw_all_cells()

    def _draw_all_cells(self):
        for r in range(self.rows):
            for c in range(self.cols):
                self._draw_cell(r, c)

    def _draw_cell(self, r, c, colour=None):
        """Draw a single cell with the appropriate colour."""
        if colour is None:
            colour = self._cell_colour(r, c)
        x1 = c * self.cell_sz
        y1 = r * self.cell_sz
        x2 = x1 + self.cell_sz
        y2 = y1 + self.cell_sz
        tag = f"cell_{r}_{c}"
        self.canvas.delete(tag)
        self.canvas.create_rectangle(
            x1+1, y1+1, x2-1, y2-1,
            fill=colour, outline=CLR_GRID_LINE, width=1, tags=tag
        )
        # draw S / G labels
        if (r, c) == self.start:
            self._draw_label(r, c, "S", "#000000")
        elif (r, c) == self.goal:
            self._draw_label(r, c, "G", "#000000")

    def _draw_label(self, r, c, text, fg):
        cx = c * self.cell_sz + self.cell_sz // 2
        cy = r * self.cell_sz + self.cell_sz // 2
        self.canvas.create_text(cx, cy, text=text, fill=fg,
                                 font=("Courier", max(8, self.cell_sz//3), "bold"),
                                 tags=f"cell_{r}_{c}")

    def _cell_colour(self, r, c):
        if (r, c) == self.start:
            return CLR_START
        if (r, c) == self.goal:
            return CLR_GOAL
        if self.grid[r][c] == 1:
            return CLR_WALL
        return self.cell_state.get((r,c), CLR_EMPTY)

    def _apply_grid_size(self):
        self._stop_agent()
        self.rows    = self.rows_var.get()
        self.cols    = self.cols_var.get()
        self.grid    = [[0]*self.cols for _ in range(self.rows)]
        self.start   = (0, 0)
        self.goal    = (self.rows-1, self.cols-1)
        # resize canvas
        self.canvas.config(width=self.cols*self.cell_sz,
                           height=self.rows*self.cell_sz)
        self._init_grid()
        self.status_var.set("Grid resized.")

    def _random_map(self):
        self._stop_agent()
        density = self.density_var.get() / 100.0
        self.grid = [[0]*self.cols for _ in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                if (r,c) not in (self.start, self.goal):
                    self.grid[r][c] = 1 if random.random() < density else 0
        self.cell_state.clear()
        self._draw_all_cells()
        self.status_var.set("Random map generated.")

    def _clear_map(self):
        self._stop_agent()
        self.grid = [[0]*self.cols for _ in range(self.rows)]
        self.cell_state.clear()
        self._draw_all_cells()
        self.status_var.set("Map cleared.")

    # ─────────────────── MOUSE INTERACTION ──────────────────
    def _pixel_to_cell(self, x, y):
        r = y // self.cell_sz
        c = x // self.cell_sz
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return r, c
        return None

    def _on_click(self, event):
        cell = self._pixel_to_cell(event.x, event.y)
        if cell is None:
            return
        r, c = cell

        if self.place_mode == "start":
            self.start      = (r, c)
            self.grid[r][c] = 0
            self.place_mode = None
            self.status_var.set(f"Start set to {(r,c)}")
            self._draw_all_cells()
            return

        if self.place_mode == "goal":
            self.goal       = (r, c)
            self.grid[r][c] = 0
            self.place_mode = None
            self.status_var.set(f"Goal set to {(r,c)}")
            self._draw_all_cells()
            return

        # toggle wall
        if (r,c) not in (self.start, self.goal):
            self.grid[r][c] = 1 - self.grid[r][c]
            self._draw_cell(r, c)

    def _on_drag(self, event):
        """Drag to paint walls."""
        if self.place_mode:
            return
        cell = self._pixel_to_cell(event.x, event.y)
        if cell and cell not in (self.start, self.goal):
            r, c = cell
            self.grid[r][c] = 1
            self._draw_cell(r, c)

    def _mode_start(self):
        self.place_mode = "start"
        self.status_var.set("Click a cell to set Start")

    def _mode_goal(self):
        self.place_mode = "goal"
        self.status_var.set("Click a cell to set Goal")

    # ─────────────────── SEARCH RUNNER ──────────────────────
    def _get_heuristic(self):
        if self.heuristic_var.get() == "Manhattan":
            return manhattan
        return euclidean

    def _run_search(self, start=None, show_animation=True):
        """
        Run the selected algorithm from 'start' (default: self.start).
        Returns (path, visited).
        Also updates metrics.
        """
        if start is None:
            start = self.start
        h_fn = self._get_heuristic()
        algo = self.algo_var.get()

        t0 = time.time()
        if algo == "A*":
            path, visited, nv = a_star(
                self.grid, start, self.goal, h_fn, self.rows, self.cols)
        else:
            path, visited, nv = greedy_bfs(
                self.grid, start, self.goal, h_fn, self.rows, self.cols)
        elapsed = (time.time() - t0) * 1000

        self.nodes_visited = nv
        self.path_cost     = len(path) - 1 if path else 0
        self.exec_time_ms  = elapsed

        self.lbl_nodes.set(str(nv))
        self.lbl_cost.set(str(self.path_cost))
        self.lbl_time.set(f"{elapsed:.2f}")

        return path, visited

    def _run_static(self):
        """Find path and colour visited + path immediately (no animation)."""
        self._stop_agent()
        self.cell_state.clear()
        self._draw_all_cells()

        path, visited = self._run_search()

        # colour visited nodes
        for (r,c) in visited:
            if (r,c) not in (self.start, self.goal):
                self.cell_state[(r,c)] = CLR_VISITED
                self._draw_cell(r, c)

        if path:
            for (r,c) in path:
                if (r,c) not in (self.start, self.goal):
                    self.cell_state[(r,c)] = CLR_PATH
                    self._draw_cell(r, c)
            self.status_var.set(
                f"Path found! Cost={self.path_cost} | Nodes={self.nodes_visited}")
        else:
            self.status_var.set("No path found!")

    # ─────────────────── ANIMATED AGENT ─────────────────────
    def _run_agent(self):
        self._stop_agent()
        self.cell_state.clear()
        self._draw_all_cells()

        path, visited = self._run_search()
        if not path:
            self.status_var.set("No path found!")
            return

        # first show exploration animation
        self._animate_exploration(visited, path)

    def _animate_exploration(self, visited, path, idx=0):
        """Animate frontier expansion, then move agent along path."""
        if idx < len(visited):
            r, c = visited[idx]
            if (r,c) not in (self.start, self.goal):
                self.cell_state[(r,c)] = CLR_VISITED
                self._draw_cell(r, c)
            self.move_job = self.root.after(
                10, self._animate_exploration, visited, path, idx+1)
        else:
            # draw path on top
            for (r,c) in path:
                if (r,c) not in (self.start, self.goal):
                    self.cell_state[(r,c)] = CLR_PATH
                    self._draw_cell(r, c)
            # now animate agent movement
            self.agent_path    = path
            self.agent_idx     = 0
            self.agent_running = True
            self._move_agent()

    def _move_agent(self):
        if not self.agent_running:
            return
        if self.agent_idx >= len(self.agent_path):
            self.status_var.set(
                f"✅ Goal reached! Cost={self.path_cost} | Nodes={self.nodes_visited} | Time={self.exec_time_ms:.2f}ms")
            self.agent_running = False
            return

        r, c = self.agent_path[self.agent_idx]
        self._draw_agent(r, c)
        self.agent_idx += 1
        self.move_job = self.root.after(80, self._move_agent)

    def _draw_agent(self, r, c):
        """Draw a circle for the agent on the given cell."""
        # redraw full grid first to remove old agent marker
        # (only redraw around previous cell for speed)
        if self.agent_idx > 1:
            pr, pc = self.agent_path[self.agent_idx - 2]
            self._draw_cell(pr, pc)
        # draw agent circle
        x1 = c * self.cell_sz + 4
        y1 = r * self.cell_sz + 4
        x2 = (c+1) * self.cell_sz - 4
        y2 = (r+1) * self.cell_sz - 4
        self.canvas.delete("agent")
        self.canvas.create_oval(x1, y1, x2, y2, fill=CLR_AGENT,
                                 outline="#ffffff", width=2, tags="agent")
        self.status_var.set(f"Agent at {(r,c)}...")

    # ─────────────────── DYNAMIC MODE ───────────────────────
    def _toggle_dynamic(self):
        self.dynamic_mode = not self.dynamic_mode
        if self.dynamic_mode:
            self.dyn_btn.config(text="⚡ Dynamic ON  (click to disable)",
                                bg="#27ae60")
            self.status_var.set("Dynamic mode ON. Run agent to start.")
        else:
            self.dyn_btn.config(text="Enable Dynamic Obstacles", bg=CLR_GOAL)
            if self.dynamic_job:
                self.root.after_cancel(self.dynamic_job)
                self.dynamic_job = None
            self.status_var.set("Dynamic mode OFF.")

    def _spawn_obstacles(self):
        """Randomly add walls; if any land on current path → replan."""
        if not self.agent_running or not self.dynamic_mode:
            return
        prob = self.prob_var.get() / 100.0
        path_cells = set(self.agent_path[self.agent_idx:])  # remaining path
        replanning_needed = False

        for r in range(self.rows):
            for c in range(self.cols):
                if (r,c) in (self.start, self.goal):
                    continue
                if self.grid[r][c] == 0 and random.random() < prob:
                    self.grid[r][c] = 1
                    self.cell_state.pop((r,c), None)
                    self._draw_cell(r, c)
                    if (r,c) in path_cells:
                        replanning_needed = True

        if replanning_needed:
            self._replan()

        if self.agent_running and self.dynamic_mode:
            self.dynamic_job = self.root.after(
                self.dynamic_interval, self._spawn_obstacles)

    def _replan(self):
        """
        Re-plan from agent's current position.
        Called when a new obstacle blocks the current path.
        """
        if self.agent_idx >= len(self.agent_path):
            return
        current_pos = self.agent_path[self.agent_idx]
        self.status_var.set(f"⚠ Obstacle on path! Re-planning from {current_pos}...")

        # cancel current movement
        if self.move_job:
            self.root.after_cancel(self.move_job)

        # run fresh search from current position
        path, visited = self._run_search(start=current_pos)

        if path:
            # update path colours
            for (r,c) in self.agent_path:
                if (r,c) not in (self.start, self.goal) and self.grid[r][c] == 0:
                    self.cell_state.pop((r,c), None)
                    self._draw_cell(r, c)
            for (r,c) in path:
                if (r,c) not in (self.start, self.goal):
                    self.cell_state[(r,c)] = CLR_PATH
                    self._draw_cell(r, c)
            self.agent_path  = path
            self.agent_idx   = 0
            self.status_var.set("Re-planning successful! Resuming...")
            self._move_agent()
        else:
            self.agent_running = False
            self.status_var.set("❌ No path available after re-planning!")

    def _run_agent_dynamic(self):
        """Start agent + dynamic spawning together."""
        self._stop_agent()
        self.cell_state.clear()
        self._draw_all_cells()

        path, visited = self._run_search()
        if not path:
            self.status_var.set("No path found!")
            return

        self.agent_path    = path
        self.agent_idx     = 0
        self.agent_running = True

        for (r,c) in path:
            if (r,c) not in (self.start, self.goal):
                self.cell_state[(r,c)] = CLR_PATH
                self._draw_cell(r, c)

        self._move_agent()
        if self.dynamic_mode:
            self.dynamic_job = self.root.after(
                self.dynamic_interval, self._spawn_obstacles)

    def _stop_agent(self):
        self.agent_running = False
        if self.move_job:
            self.root.after_cancel(self.move_job)
            self.move_job = None
        if self.dynamic_job:
            self.root.after_cancel(self.dynamic_job)
            self.dynamic_job = None
        self.canvas.delete("agent")

    # override Run Agent button to also start dynamic
    # We patch this in __init__ by redirecting the button
    def _run_agent_wrapper(self):
        if self.dynamic_mode:
            self._run_agent_dynamic()
        else:
            self._run_agent()


#  ENTRY POINT
if __name__ == "__main__":
    root = tk.Tk()
    app  = PathfindingApp(root)
    app._run_agent = app._run_agent_wrapper
    root.mainloop()