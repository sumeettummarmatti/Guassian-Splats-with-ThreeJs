import open3d as o3d
import numpy as np
import json
import os

# --- CONFIGURATION ---
PLY_FILE = "/Users/sumeet/Desktop/export_last.ply"
OUTPUT_JSON = "collision_map.json"
GRID_SIZE = 0.10  # 10cm resolution (High fidelity)
PLAYER_HEIGHT = 1.5  # Meters
ANKLE_HEIGHT = 0.2   # Lowered to 20cm to catch smaller obstacles
MAX_SLOPE = 0.5      # Max height difference to be considered "walkable"

def generate_collision_map():
    if not os.path.exists(PLY_FILE):
        print(f"Error: File not found at {PLY_FILE}")
        return

    print(f"Loading {PLY_FILE}...")
    try:
        pcd = o3d.io.read_point_cloud(PLY_FILE)
        if not pcd.has_points():
            print("Error: Point cloud is empty.")
            return
    except Exception as e:
        print(f"Error loading PLY: {e}")
        return

    points = np.asarray(pcd.points)
    print(f"Loaded {len(points)} points.")
    
    # 1. Find Bounds
    min_bound = points.min(axis=0)
    max_bound = points.max(axis=0)
    
    width_x = max_bound[0] - min_bound[0]
    depth_z = max_bound[2] - min_bound[2]
    
    cols = int(np.ceil(width_x / GRID_SIZE))
    rows = int(np.ceil(depth_z / GRID_SIZE))
    
    print(f"Grid size: {cols} x {rows} (Dimensions: {width_x:.2f}m x {depth_z:.2f}m)")

    # 2. Bucket points into the grid
    # We store just the Y values (heights) for every X,Z coordinate
    grid = {} # Key: "c,r", Value: list of y heights

    for p in points:
        # Convert world x,z to grid indices
        c = int((p[0] - min_bound[0]) / GRID_SIZE)
        r = int((p[2] - min_bound[2]) / GRID_SIZE)
        
        # Clamp indices just in case
        c = max(0, min(c, cols - 1))
        r = max(0, min(r, rows - 1))

        key = f"{c},{r}"
        if key not in grid:
            grid[key] = []
        grid[key].append(p[1])

    # 3. Analyze each cell to determine Floor Height vs Obstacle
    export_map = {}
    
    # Track global floor for the viewer
    global_min_y = min_bound[1]

    for key, heights in grid.items():
        if not heights:
            continue
            
        heights.sort()
        
        # Heuristic: The lowest point in this cell is the floor
        floor_y = heights[0] 
        max_y = heights[-1]
        
        # Check for obstacles
        is_blocked = False
        
        # Rule 1: Vertical spread (Wall detection)
        # If a cell has points spanning a large vertical range, it's likely a wall
        if (max_y - floor_y) > PLAYER_HEIGHT:
            is_blocked = True
        else:
            # Rule 2: Obstacle in body zone
            # Check if there are points that would block movement
            # We iterate through points to see if any fall in the "blocking zone"
            for h in heights:
                # Check if point is in the "body zone" (ankle to head)
                if h > (floor_y + ANKLE_HEIGHT) and h < (floor_y + PLAYER_HEIGHT):
                    is_blocked = True
                    break
        
        export_map[key] = {
            "y": round(floor_y, 3),
            "b": 1 if is_blocked else 0 # b for blocked
        }

    # 4. Save to JSON
    output_data = {
        "metadata": {
            "min_x": min_bound[0],
            "min_z": min_bound[2],
            "min_y": global_min_y, # Export global floor height
            "grid_size": GRID_SIZE,
            "cols": cols,
            "rows": rows
        },
        "map": export_map
    }
    
    try:
        with open(OUTPUT_JSON, "w") as f:
            json.dump(output_data, f)
        print(f"Saved collision map to {OUTPUT_JSON} with {len(export_map)} cells.")
    except Exception as e:
        print(f"Error saving JSON: {e}")

if __name__ == "__main__":
    generate_collision_map()