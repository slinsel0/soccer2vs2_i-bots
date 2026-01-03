import numpy as np

class GeometryTransformer:
    """
    Handles the mathematical transformation from Mirror-Pixel-Coordinates 
    to Real-World-Polar-Coordinates without expensive image unwarping.
    """
    def __init__(self, config):
        self.cx = config['mirror']['center_x']
        self.cy = config['mirror']['center_y']
        self.r_inner = config['mirror']['radius_inner']
        self.r_outer = config['mirror']['radius_outer']

    def pixel_to_polar(self, x, y):
        """
        Vectorized calculation of distance (in pixels) and angle using NumPy.
        Returns: (distance_px, angle_rad)
        """
        # 1. Center coordinates
        dx = x - self.cx
        dy = y - self.cy

        # 2. Pixel radius and Angle (Vectorized if x/y are arrays)
        r_px = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dx, dy) # Result in -pi to +pi

        return r_px, angle

    def get_donut_mask(self, shape):
        """Creates a static boolean mask for the valid mirror area."""
        h, w = shape[:2]
        Y, X = np.ogrid[:h, :w]
        dist_from_center = np.sqrt((X - self.cx)**2 + (Y - self.cy)**2)
        
        mask = (dist_from_center >= self.r_inner) & (dist_from_center <= self.r_outer)
        return mask.astype(np.uint8) * 255

    def get_dynamic_area_threshold(self, r_px, base_area):
        """
        Optional: Adjust expected blob size based on distance from center.
        Objects often appear smaller/squashed near the edges of a convex mirror.
        """
        # Example: Scale factor linear to radius (simplify as needed)
        scale = 1.0 # Implement specific mirror curve logic here
        return base_area * scale