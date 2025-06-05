import numpy as np
from scipy.interpolate import BSpline

class TrajectoryPlanner:
    def __init__(self, start_point, end_point, num_points):
        self.start_point = np.array(start_point, dtype=float)
        self.end_point = np.array(end_point, dtype=float)
        self.num_points = num_points
        
    def generate_polynomial_path(self):
        dp = self.end_point - self.start_point
        
        t = np.linspace(0.0, 1.0, self.num_points)
        blend = 10*t**3 - 15*t**4 + 6*t**5
        
        path = self.start_point + np.outer(blend, dp)
        
        return path
    
    def bspline_path(self, path, degree=3):
        num_points = self.num_points
        N = len(path)
        k = degree
        if N <= k:
            raise ValueError("Number of control points must be greater than the degree.")

        interior = N - (k+1)
        
        inner_knots = np.linspace(0, 1, interior+2)[1:-1] if interior > 0 else []
        
        knots = np.concatenate((np.zeros(k+1), inner_knots, np.ones(k+1)))
        
        spline = BSpline(knots, path, k, axis=0)
        u = np.linspace(0, 1, num_points)
        
        return spline(u)
    
    def plan(self):
        poly = self.generate_polynomial_path()
        smooth = self.bspline_path(poly)
        
        return smooth
    
class CircleTrajectoryPlanner:
    def __init__(self, start_point, center_point, radius, z, num_circle, num_trans):
        self.start_point = start_point
        self.center_point = center_point
        self.r = radius
        self.z = z
        self.num_circle = num_circle
        self.num_trans = num_trans
        
    def plan(self):
        xs, ys, zs = self.start_point
        x0, y0 = self.center_point
        
        angle0 = np.arctan2(ys - y0, xs - x0)
        x_on = x0 + self.r * np.cos(angle0)
        y_on = y0 + self.r * np.sin(angle0)
        z_on = self.z
        
        trans_t = np.linspace(0, 1, self.num_trans)
        x_trans = xs + (x_on - xs) * trans_t
        y_trans = ys + (y_on - ys) * trans_t
        z_trans = zs + (z_on - zs) * trans_t
        trans_path = np.vstack((x_trans, y_trans, z_trans)).T
        
        theta = np.linspace(angle0, angle0 + 2*np.pi, self.num_circle, endpoint=False)
        x_circle = x0 + self.r * np.cos(theta)
        y_circle = y0 + self.r * np.sin(theta)
        z_circle = np.full_like(theta, self.z)
        circle_path = np.vstack((x_circle, y_circle, z_circle)).T

        full_path = np.vstack((trans_path, circle_path))
        return full_path