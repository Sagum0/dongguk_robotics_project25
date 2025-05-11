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
    
    def bspline_path(self, path, degree=3, num_points=100):
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