import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import cv2
from typing import List, Tuple, Set, Optional
from scipy.spatial.distance import cdist

class PNGMapDronePathPlanner:
    def __init__(self, png_path: str, map_bounds: Tuple[Tuple[float, float], Tuple[float, float]], 
                 start_pos: Tuple[float, float], x_rows: int = 3, y_rows: int = 4,
                 obstacle_threshold: int = 127):
        """
        Initialize the drone path planner with PNG map
        
        Args:
            png_path: Path to PNG image file
            map_bounds: ((x_min, x_max), (y_min, y_max)) real-world coordinates
            start_pos: (x, y) starting position in real-world coordinates
            x_rows: number of rows along x-axis (vertical lines)
            y_rows: number of rows along y-axis (horizontal lines)
            obstacle_threshold: pixel intensity threshold (0-255) below which pixel is considered obstacle
        """
        self.png_path = png_path
        self.x_bounds, self.y_bounds = map_bounds
        self.start_pos = start_pos
        self.x_rows = x_rows
        self.y_rows = y_rows
        self.obstacle_threshold = obstacle_threshold
        
        self.original_image = None
        self.processed_image = None
        self.obstacle_mask = None
        self.load_and_process_map()
        
        border_margin = 2.0
        x_min_safe = self.x_bounds[0] + border_margin
        x_max_safe = self.x_bounds[1] - border_margin
        y_min_safe = self.y_bounds[0] + border_margin
        y_max_safe = self.y_bounds[1] - border_margin


        self.x_spacing = (x_max_safe - x_min_safe) / (x_rows - 1) if x_rows > 1 else 0
        self.y_spacing = (y_max_safe - y_min_safe) / (y_rows - 1) if y_rows > 1 else 0


        self.x_positions = [x_min_safe + i * self.x_spacing for i in range(x_rows)]
        self.y_positions = [y_min_safe + i * self.y_spacing for i in range(y_rows)]
            

        self.obstacle_grid_points = set()
        self.check_grid_obstacles()
        
    def load_and_process_map(self):
        """Load PNG image and process it to identify obstacles"""
        try:
            
            self.original_image = Image.open(self.png_path)
            
           
            if self.original_image.mode != 'L':
                self.processed_image = self.original_image.convert('L')
            else:
                self.processed_image = self.original_image.copy()
            
            
            img_array = np.array(self.processed_image)
            
            
            self.obstacle_mask = img_array < self.obstacle_threshold
            
            print(f"Map loaded: {img_array.shape[1]}x{img_array.shape[0]} pixels")
            print(f"Obstacle pixels: {np.sum(self.obstacle_mask)}/{img_array.size} ({100*np.sum(self.obstacle_mask)/img_array.size:.1f}%)")
            
        except Exception as e:
            print(f"Error loading PNG map: {e}")
            
            self.create_dummy_map()
    
    def create_dummy_map(self):
        """Create a dummy map for demonstration purposes"""
        print("Creating dummy map for demonstration...")
        width, height = 400, 200
        img_array = np.ones((height, width), dtype=np.uint8) * 255  
        
        
        img_array[60:100, 150:200] = 0  
        img_array[120:160, 50:120] = 0  
        
        center_y, center_x = 100, 300
        y, x = np.ogrid[:height, :width]
        mask = (x - center_x)**2 + (y - center_y)**2 <= 30**2
        img_array[mask] = 0
        
        self.processed_image = Image.fromarray(img_array, mode='L')
        self.obstacle_mask = img_array < self.obstacle_threshold
        
    def world_to_pixel(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to pixel coordinates"""
        if self.processed_image is None:
            return (0, 0)
            
        img_width, img_height = self.processed_image.size
        
        pixel_x = int((world_x - self.x_bounds[0]) / (self.x_bounds[1] - self.x_bounds[0]) * img_width)
        pixel_y = int((self.y_bounds[1] - world_y) / (self.y_bounds[1] - self.y_bounds[0]) * img_height)
        
        pixel_x = max(0, min(pixel_x, img_width - 1))
        pixel_y = max(0, min(pixel_y, img_height - 1))
        
        return (pixel_x, pixel_y)
    
    def pixel_to_world(self, pixel_x: int, pixel_y: int) -> Tuple[float, float]:
        """Convert pixel coordinates to world coordinates"""
        if self.processed_image is None:
            return (0.0, 0.0)
            
        img_width, img_height = self.processed_image.size
        
        world_x = self.x_bounds[0] + (pixel_x / img_width) * (self.x_bounds[1] - self.x_bounds[0])
        world_y = self.y_bounds[1] - (pixel_y / img_height) * (self.y_bounds[1] - self.y_bounds[0])
        
        return (world_x, world_y)
    
    def is_safe_position(self, world_x: float, world_y: float, safety_radius: float = 0.5) -> bool:
        """Check if a world position is safe (not an obstacle)"""
        if self.obstacle_mask is None:
            return True
            
        test_points = [
            (world_x, world_y),  
            (world_x + safety_radius, world_y),  
            (world_x - safety_radius, world_y),  
            (world_x, world_y + safety_radius),  
            (world_x, world_y - safety_radius), 
        ]
        
        for tx, ty in test_points:
            pixel_x, pixel_y = self.world_to_pixel(tx, ty)
            if self.obstacle_mask[pixel_y, pixel_x]:
                return False
        
        return True
    
    def check_grid_obstacles(self):
        """Check which grid points are obstacles"""
        for x in self.x_positions:
            for y in self.y_positions:
                if not self.is_safe_position(x, y):
                    self.obstacle_grid_points.add((x, y))
    
    def find_nearest_safe_position(self, target_x: float, target_y: float, 
                                 search_radius: float = 2.0) -> Optional[Tuple[float, float]]:
        """Find the nearest safe position to the target within search radius"""
        if self.is_safe_position(target_x, target_y):
            return (target_x, target_y)
        
        
        step_size = 0.05 
        best_pos = None
        min_distance = float('inf')
        
        search_steps = int(search_radius / step_size)
        for dx in range(-search_steps, search_steps + 1):
            for dy in range(-search_steps, search_steps + 1):
                test_x = target_x + dx * step_size
                test_y = target_y + dy * step_size
                
                if (self.x_bounds[0] <= test_x <= self.x_bounds[1] and 
                    self.y_bounds[0] <= test_y <= self.y_bounds[1]):
                    
                    if self.is_safe_position(test_x, test_y):
                        distance = np.sqrt((test_x - target_x)**2 + (test_y - target_y)**2)
                        if distance < min_distance:
                            min_distance = distance
                            best_pos = (test_x, test_y)
        
        return best_pos
    
    def generate_lawnmower_path(self, safety_margin: float = 0.3) -> List[Tuple[float, float]]:
        """Generate a lawnmower pattern avoiding obstacles from PNG map"""
        waypoints = []
        current_pos = self.start_pos
        
        if not self.is_safe_position(current_pos[0], current_pos[1]):
            safe_start = self.find_nearest_safe_position(current_pos[0], current_pos[1])
            if safe_start:
                current_pos = safe_start
            else:
                print("Warning: Could not find safe starting position!")
        
        waypoints.append(current_pos)
        
        direction = 1  # 1 for right, -1 for left
        
        for i, y in enumerate(self.y_positions):
            x_range = self.x_positions[::direction]

            for x in x_range:
                if self.is_safe_position(x, y, safety_radius=0.5):
                    target_pos = (x, y)
                    if target_pos != current_pos:
                        waypoints.append(target_pos)
                        current_pos = target_pos
                else:
                    safe_pos = self.find_nearest_safe_position(x, y, search_radius=2.0)
                    if safe_pos and safe_pos != current_pos and self.is_safe_position(safe_pos[0], safe_pos[1], safety_radius=0.5):
                        waypoints.append(safe_pos)
                        current_pos = safe_pos

            direction *= -1
        
        safe_waypoints = []
        for wp in waypoints:
            if self.is_safe_position(wp[0], wp[1], safety_radius=0.5):
                safe_waypoints.append(wp)
            else:
                safe_pos = self.find_nearest_safe_position(wp[0], wp[1], search_radius=2.0)
                if safe_pos and self.is_safe_position(safe_pos[0], safe_pos[1], safety_radius=0.5):
                    safe_waypoints.append(safe_pos)

        return safe_waypoints
    
    
    def visualize_map_and_path(self, waypoints: List[Tuple[float, float]], 
                              title: str = "PNG Map Coverage Path"):
        """Visualize the PNG map with the generated path"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        if self.processed_image is not None:
            ax1.imshow(self.processed_image, cmap='gray', extent=[
                self.x_bounds[0], self.x_bounds[1], 
                self.y_bounds[0], self.y_bounds[1]
            ])
            ax1.set_title("PNG Map (Dark = Obstacles)")
        
        if self.obstacle_mask is not None:
            ax2.imshow(~self.obstacle_mask, cmap='gray', alpha=0.3, extent=[
                self.x_bounds[0], self.x_bounds[1], 
                self.y_bounds[0], self.y_bounds[1]
            ])
        
        for x in self.x_positions:
            ax2.axvline(x, color='lightgray', linestyle='--', alpha=0.5)
        for y in self.y_positions:
            ax2.axhline(y, color='lightgray', linestyle='--', alpha=0.5)

        for obs_x, obs_y in self.obstacle_grid_points:
            ax2.plot(obs_x, obs_y, 'rs', markersize=12, alpha=0.7)
        

        if waypoints:
            path_x = [p[0] for p in waypoints]
            path_y = [p[1] for p in waypoints]
            
            ax2.plot(path_x, path_y, 'b-', linewidth=2, alpha=0.8, label='Path')
            ax2.plot(path_x, path_y, 'bo', markersize=6, label='Waypoints')
            
            ax2.plot(waypoints[0][0], waypoints[0][1], 'go', markersize=12, label='Start')
            ax2.plot(waypoints[-1][0], waypoints[-1][1], 'ro', markersize=12, label='End')
            
   
            for i, (x, y) in enumerate(waypoints[::max(1, len(waypoints)//10)]):  
                ax2.annotate(str(i * max(1, len(waypoints)//10)), (x, y), 
                           xytext=(5, 5), textcoords='offset points', fontsize=8)
        
        for ax in [ax1, ax2]:
            ax.set_xlim(self.x_bounds[0], self.x_bounds[1])
            ax.set_ylim(self.y_bounds[0], self.y_bounds[1])
            ax.set_xlabel('X Position')
            ax.set_ylabel('Y Position')
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
        
        ax2.set_title(title)
        ax2.legend()
        
        plt.tight_layout()
        plt.show()
        
        return fig
    
    def get_coverage_statistics(self, waypoints: List[Tuple[float, float]]) -> dict:
        """Calculate coverage statistics"""
        if not waypoints:
            return {}
        
        path_length = 0
        for i in range(len(waypoints) - 1):
            dx = waypoints[i+1][0] - waypoints[i][0]
            dy = waypoints[i+1][1] - waypoints[i][1]
            path_length += np.sqrt(dx*dx + dy*dy)
        
        total_area = (self.x_bounds[1] - self.x_bounds[0]) * (self.y_bounds[1] - self.y_bounds[0])
        if self.obstacle_mask is not None:
            free_area_ratio = 1 - (np.sum(self.obstacle_mask) / self.obstacle_mask.size)
            free_area = total_area * free_area_ratio
        else:
            free_area = total_area
        
        return {
            'total_waypoints': len(waypoints),
            'path_length': path_length,
            'total_area': total_area,
            'free_area': free_area,
            'obstacle_grid_points': len(self.obstacle_grid_points),
            'coverage_efficiency': len(waypoints) / free_area if free_area > 0 else 0
        }

def main():
    png_path = 'your_map.png'  
    
    map_bounds = ((-10, 10), (-5, 5))
    start_pos = (-9.5, -4)
    
    try:
        planner = PNGMapDronePathPlanner(
            png_path="/root/ros2_ws/src/pkg/Drone-Manager/mymap_crop2.png",
            map_bounds=map_bounds, 
            start_pos=start_pos, 
            x_rows=3, 
            y_rows=3,
            obstacle_threshold=127 
        )

        print("Grid positions:")
        print(f"X positions: {planner.x_positions}")
        print(f"Y positions: {planner.y_positions}")
        print(f"Obstacle grid points: {len(planner.obstacle_grid_points)}")

        lawnmower_path = planner.generate_lawnmower_path()

        print(f"\nLawnmower path ({len(lawnmower_path)} waypoints):")
        for i, wp in enumerate(lawnmower_path[:10]):  
            print(f"  {i}: ({wp[0]:.2f}, {wp[1]:.2f})")
        if len(lawnmower_path) > 10:
            print(f"  ... and {len(lawnmower_path) - 10} more waypoints")

        lawnmower_stats = planner.get_coverage_statistics(lawnmower_path)

        print(f"\nPath Statistics:")
        print(f"Lawnmower - Length: {lawnmower_stats['path_length']:.2f}, Waypoints: {lawnmower_stats['total_waypoints']}")

        planner.visualize_map_and_path(lawnmower_path, "Lawnmower Pattern on PNG Map")
        
    except Exception as e:
        print(f"Error in main execution: {e}")
        print("The demo will create a dummy map since no PNG was provided.")

if __name__ == "__main__":
    main()