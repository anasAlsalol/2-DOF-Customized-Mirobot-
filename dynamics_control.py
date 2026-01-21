#!/usr/bin/env python3
"""
dynamics_control ROS 2 NODE - COMPLETE UNIFIED SOLUTION
2-DOF Manipulator Control with Lagrangian & Newton-Euler Models
Communicates with CoppeliaSim via ROS 2 using simROS2
VERSION: 1.2 (مع التعديلات المطلوبة)
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import numpy as np
import math
import json
import csv
import time
from datetime import datetime
from pathlib import Path
from threading import Lock
import matplotlib
import os

# Detect headless / WSL environments: if DISPLAY is not set, use non-interactive backend
HEADLESS_PLOT = False
if os.environ.get('DISPLAY', '') == '' or os.environ.get('WSL_DISTRO_NAME'):
    try:
        matplotlib.use('Agg')
    except Exception:
        pass
    HEADLESS_PLOT = True

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import signal
import sys
import os

from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# ============================================================================
# DYNAMICS MODELS (UNCHANGED)
# ============================================================================

class LagrangianModel:
    """Lagrangian dynamics for 2-DOF planar manipulator"""
    
    def __init__(self, masses=[1.0, 1.0], lengths=[0.3, 0.3], gravity=9.81):
        self.m1, self.m2 = masses
        self.l1, self.l2 = lengths
        self.g = gravity
        
        # Center of mass distances
        self.lc1 = self.l1 / 2
        self.lc2 = self.l2 / 2
        
        # Moments of inertia (simplified for uniform rods)
        self.I1 = self.m1 * self.l1**2 / 12
        self.I2 = self.m2 * self.l2**2 / 12
    
    def inertia_matrix(self, q):
        """Compute inertia matrix M(q)"""
        c2 = math.cos(q[1])
        
        M11 = (self.m1 * self.lc1**2 + self.I1 + 
               self.m2 * (self.l1**2 + self.lc2**2 + 2*self.l1*self.lc2*c2) + self.I2)
        
        M12 = self.m2 * (self.lc2**2 + self.l1*self.lc2*c2) + self.I2
        M22 = self.m2 * self.lc2**2 + self.I2
        
        return np.array([[M11, M12], [M12, M22]])
    
    def coriolis_matrix(self, q, qd):
        """Compute Coriolis matrix C(q, qd)"""
        s2 = math.sin(q[1])
        h = -self.m2 * self.l1 * self.lc2 * s2
        
        return np.array([
            [h * qd[1], h * (qd[0] + qd[1])],
            [-h * qd[0], 0]
        ])
    
    def gravity_vector(self, q):
        """Compute gravity vector G(q)"""
        g1 = ((self.m1 * self.lc1 + self.m2 * self.l1) * math.cos(q[0]) + 
              self.m2 * self.lc2 * math.cos(q[0] + q[1])) * self.g
        
        g2 = self.m2 * self.lc2 * math.cos(q[0] + q[1]) * self.g
        
        return np.array([g1, g2])
    
    def compute_torques(self, q, qd, qdd):
        """τ = M(q)q'' + C(q,q')q' + G(q)"""
        M = self.inertia_matrix(q)
        C = self.coriolis_matrix(q, qd)
        G = self.gravity_vector(q)
        
        return M @ qdd + C @ qd + G
    
    def forward_kinematics(self, q):
        """Compute end-effector position"""
        x = self.l1 * math.cos(q[0]) + self.l2 * math.cos(q[0] + q[1])
        y = self.l1 * math.sin(q[0]) + self.l2 * math.sin(q[0] + q[1])
        return np.array([x, y])


class NewtonEulerModel:
    """Recursive Newton-Euler dynamics"""
    
    def __init__(self, masses=[1.0, 1.0], lengths=[0.3, 0.3], gravity=9.81):
        self.m = masses
        self.l = lengths
        self.g = gravity
        
        # COM positions relative to joints
        self.rc = [[lengths[0]/2, 0, 0],
                   [lengths[1]/2, 0, 0]]
        
        # Inertia tensors (simplified for planar)
        self.Ic = [
            np.diag([0, 0, masses[0] * lengths[0]**2 / 12]),
            np.diag([0, 0, masses[1] * lengths[1]**2 / 12])
        ]
    
    def rotation_z(self, theta):
        """Rotation matrix about Z axis"""
        c, s = math.cos(theta), math.sin(theta)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    
    def compute_torques(self, q, qd, qdd):
        """Recursive Newton-Euler algorithm"""
        # Initialize
        w = [np.zeros(3), np.zeros(3)]
        wd = [np.zeros(3), np.zeros(3)]
        vd = [np.zeros(3), np.zeros(3)]
        
        # Gravity in base frame (Y down in CoppeliaSim)
        g_vec = np.array([0, -self.g, 0])
        
        # OUTWARD PASS: Base to Tip
        # Link 1 (connected to base)
        w[0] = np.array([0, 0, qd[0]])
        wd[0] = np.array([0, 0, qdd[0]])
        vd[0] = np.cross(wd[0], self.rc[0]) + np.cross(w[0], np.cross(w[0], self.rc[0])) - g_vec
        
        # Link 2
        R_01 = self.rotation_z(q[1])
        w[1] = R_01 @ w[0] + np.array([0, 0, qd[1]])
        
        term1 = R_01 @ wd[0]
        term2 = np.array([0, 0, qdd[1]])
        term3 = np.cross(R_01 @ w[0], np.array([0, 0, qd[1]]))
        wd[1] = term1 + term2 + term3
        
        vd_cross = np.cross(wd[1], self.rc[1]) + np.cross(w[1], np.cross(w[1], self.rc[1]))
        R_02 = self.rotation_z(q[0] + q[1])
        vd[1] = R_02 @ vd_cross - g_vec
        
        # INWARD PASS: Tip to Base
        f = [np.zeros(3), np.zeros(3)]
        n = [np.zeros(3), np.zeros(3)]
        tau = np.zeros(2)
        
        # Link 2
        f[1] = self.m[1] * vd[1]
        n[1] = self.Ic[1] @ wd[1] + np.cross(w[1], self.Ic[1] @ w[1])
        tau[1] = (n[1] + np.cross(self.rc[1], f[1]))[2]
        
        # Link 1
        R_10 = self.rotation_z(-q[1])  # Inverse rotation
        f2_in_1 = R_10 @ f[1]
        
        f[0] = self.m[0] * vd[0] + f2_in_1
        
        n1_com = self.Ic[0] @ wd[0] + np.cross(w[0], self.Ic[0] @ w[0])
        moment_arm = np.array([self.l[0], 0, 0])  # From COM1 to joint2
        n_total = n1_com + np.cross(moment_arm, f2_in_1) + R_10 @ n[1]
        
        tau[0] = n_total[2]
        
        return tau


class TrajectoryGenerator:
    """Generates reference trajectories"""
    
    def __init__(self, traj_type='sine', link_lengths=[0.3, 0.3]):
        self.type = traj_type
        self.l1, self.l2 = link_lengths
        
        # Trajectory parameters
        if traj_type == 'sine':
            self.A = [0.5, 0.3]      # Amplitudes
            self.f = [0.5, 0.8]      # Frequencies
            self.phi = [0, math.pi/4] # Phases
        elif traj_type == 'circle':
            self.radius = 0.1
            self.freq = 0.3
            self.center = [0.45, 0.0]
        elif traj_type == 'point':
            self.waypoints = [
                [0.0, 0.0],
                [0.8, 0.4],
                [0.4, 0.6],
                [-0.3, 0.2],
                [0.0, 0.0]
            ]
            self.durations = [1.0, 1.0, 1.0, 1.0]
            self.total_time = sum(self.durations)
    
    def generate(self, t):
        """Generate reference at time t"""
        if self.type == 'sine':
            return self._sine_trajectory(t)
        elif self.type == 'circle':
            return self._circle_trajectory(t)
        else:  # point-to-point
            return self._point_trajectory(t)
    
    def _sine_trajectory(self, t):
        q = np.array([
            self.A[0] * math.sin(2*math.pi*self.f[0]*t + self.phi[0]),
            self.A[1] * math.sin(2*math.pi*self.f[1]*t + self.phi[1])
        ])
        
        qd = np.array([
            self.A[0] * 2*math.pi*self.f[0] * math.cos(2*math.pi*self.f[0]*t + self.phi[0]),
            self.A[1] * 2*math.pi*self.f[1] * math.cos(2*math.pi*self.f[1]*t + self.phi[1])
        ])
        
        qdd = np.array([
            -self.A[0] * (2*math.pi*self.f[0])**2 * math.sin(2*math.pi*self.f[0]*t + self.phi[0]),
            -self.A[1] * (2*math.pi*self.f[1])**2 * math.sin(2*math.pi*self.f[1]*t + self.phi[1])
        ])
        
        return q, qd, qdd
    
    def _circle_trajectory(self, t):
        # Cartesian circle
        x = self.center[0] + self.radius * math.cos(2*math.pi*self.freq*t)
        y = self.center[1] + self.radius * math.sin(2*math.pi*self.freq*t)
        
        # Inverse kinematics
        c2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        c2 = max(-0.9999, min(0.9999, c2))
        
        q2 = math.acos(c2)
        q1 = math.atan2(y, x) - math.atan2(self.l2*math.sin(q2), self.l1 + self.l2*math.cos(q2))
        
        # Jacobian for velocities
        J11 = -self.l1*math.sin(q1) - self.l2*math.sin(q1+q2)
        J12 = -self.l2*math.sin(q1+q2)
        J21 = self.l1*math.cos(q1) + self.l2*math.cos(q1+q2)
        J22 = self.l2*math.cos(q1+q2)
        
        # Cartesian velocities
        xd = -self.radius * 2*math.pi*self.freq * math.sin(2*math.pi*self.freq*t)
        yd = self.radius * 2*math.pi*self.freq * math.cos(2*math.pi*self.freq*t)
        
        # Joint velocities
        J = np.array([[J11, J12], [J21, J22]])
        qd = np.linalg.inv(J) @ np.array([xd, yd])
        
        # Approximate accelerations
        qdd = np.zeros(2)
        
        return np.array([q1, q2]), qd, qdd
    
    def _point_trajectory(self, t):
        t_mod = t % self.total_time
        
        # Find current segment
        seg_time = 0
        seg_idx = 0
        for i, dur in enumerate(self.durations):
            if t_mod < seg_time + dur:
                seg_idx = i
                break
            seg_time += dur
        
        # Normalized time in segment
        s = (t_mod - seg_time) / self.durations[seg_idx]
        
        # Cubic spline interpolation
        p0 = self.waypoints[seg_idx]
        p1 = self.waypoints[seg_idx + 1]
        
        # Cubic coefficients
        h00 = 2*s**3 - 3*s**2 + 1
        h10 = s**3 - 2*s**2 + s
        h01 = -2*s**3 + 3*s**2
        h11 = s**3 - s**2
        
        # Zero velocity at waypoints
        v0 = [0, 0]
        v1 = [0, 0]
        
        q = np.array([
            h00*p0[0] + h10*v0[0] + h01*p1[0] + h11*v1[0],
            h00*p0[1] + h10*v0[1] + h01*p1[1] + h11*v1[1]
        ])
        
        # Velocities
        dh00 = 6*s**2 - 6*s
        dh10 = 3*s**2 - 4*s + 1
        dh01 = -6*s**2 + 6*s
        dh11 = 3*s**2 - 2*s
        
        qd = np.array([
            dh00*p0[0] + dh10*v0[0] + dh01*p1[0] + dh11*v1[0],
            dh00*p0[1] + dh10*v0[1] + dh01*p1[1] + dh11*v1[1]
        ]) / self.durations[seg_idx]
        
        qdd = np.zeros(2)
        
        return q, qd, qdd


class DataLogger:
    """Handles data logging and visualization"""
    
    def __init__(self, log_dir='./logs', plot_realtime=True):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = self.log_dir / f'dynamics_log_{timestamp}.csv'
        self.json_file = self.log_dir / f'metadata_{timestamp}.json'
        
        self.data_lock = Lock()
        self.data = {
            'time': [],
            'q_actual': [], 'qd_actual': [],
            'q_ref': [], 'qd_ref': [], 'qdd_ref': [],
            'tau_lag': [], 'tau_ne': [], 'tau_control': [],
            'errors': [], 'computation_times': []
        }
        
        # Plotting enabled flag (may be disabled in headless environments)
        self.plot_realtime = plot_realtime

        # Setup real-time plotting (skip in headless)
        if self.plot_realtime and not HEADLESS_PLOT:
            self.setup_plots()
    
    def setup_plots(self):
        """Setup matplotlib figures for real-time plotting"""
        plt.ion()  # Interactive mode
        self.fig, self.axes = plt.subplots(3, 2, figsize=(12, 10))
        self.fig.suptitle('2-DOF Manipulator Dynamics - Real-time Monitoring', fontsize=14)
        
        # Initialize plot lines
        self.lines = {}
        colors = ['b', 'r', 'g', 'm', 'c', 'y']
        
        # Plot 1: Joint Positions
        self.axes[0, 0].set_title('Joint Positions')
        self.axes[0, 0].set_xlabel('Time (s)')
        self.axes[0, 0].set_ylabel('Position (rad)')
        self.axes[0, 0].grid(True)
        self.lines['q1_act'] = self.axes[0, 0].plot([], [], colors[0] + '-', label='q1 actual')[0]
        self.lines['q1_ref'] = self.axes[0, 0].plot([], [], colors[0] + '--', label='q1 ref')[0]
        self.lines['q2_act'] = self.axes[0, 0].plot([], [], colors[1] + '-', label='q2 actual')[0]
        self.lines['q2_ref'] = self.axes[0, 0].plot([], [], colors[1] + '--', label='q2 ref')[0]
        self.axes[0, 0].legend()
        
        # Plot 2: Tracking Errors
        self.axes[0, 1].set_title('Tracking Errors')
        self.axes[0, 1].set_xlabel('Time (s)')
        self.axes[0, 1].set_ylabel('Error (rad)')
        self.axes[0, 1].grid(True)
        self.lines['error1'] = self.axes[0, 1].plot([], [], colors[0] + '-', label='Error q1')[0]
        self.lines['error2'] = self.axes[0, 1].plot([], [], colors[1] + '-', label='Error q2')[0]
        self.axes[0, 1].legend()
        
        # Plot 3: Torques Comparison
        self.axes[1, 0].set_title('Torques: Lagrangian vs Newton-Euler')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].set_ylabel('Torque (N·m)')
        self.axes[1, 0].grid(True)
        self.lines['tau_lag1'] = self.axes[1, 0].plot([], [], colors[0] + '-', label='τ1 Lag')[0]
        self.lines['tau_ne1'] = self.axes[1, 0].plot([], [], colors[0] + '--', label='τ1 NE')[0]
        self.lines['tau_lag2'] = self.axes[1, 0].plot([], [], colors[1] + '-', label='τ2 Lag')[0]
        self.lines['tau_ne2'] = self.axes[1, 0].plot([], [], colors[1] + '--', label='τ2 NE')[0]
        self.axes[1, 0].legend()
        
        # Plot 4: Torque Differences
        self.axes[1, 1].set_title('Torque Model Differences')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].set_ylabel('Difference (N·m)')
        self.axes[1, 1].grid(True)
        self.lines['diff1'] = self.axes[1, 1].plot([], [], colors[0] + '-', label='τ1 diff')[0]
        self.lines['diff2'] = self.axes[1, 1].plot([], [], colors[1] + '-', label='τ2 diff')[0]
        self.axes[1, 1].legend()
        
        # Plot 5: Control Torques
        self.axes[2, 0].set_title('Control Torques')
        self.axes[2, 0].set_xlabel('Time (s)')
        self.axes[2, 0].set_ylabel('Torque (N·m)')
        self.axes[2, 0].grid(True)
        self.lines['tau_ctrl1'] = self.axes[2, 0].plot([], [], colors[0] + '-', label='τ1 control')[0]
        self.lines['tau_ctrl2'] = self.axes[2, 0].plot([], [], colors[1] + '-', label='τ2 control')[0]
        self.axes[2, 0].legend()
        
        # Plot 6: End-effector Path
        self.axes[2, 1].set_title('End-effector Path')
        self.axes[2, 1].set_xlabel('X (m)')
        self.axes[2, 1].set_ylabel('Y (m)')
        self.axes[2, 1].grid(True)
        self.axes[2, 1].set_aspect('equal')
        self.lines['ee_path'] = self.axes[2, 1].plot([], [], colors[0] + '-', alpha=0.5, label='Path')[0]
        self.lines['ee_current'] = self.axes[2, 1].plot([], [], 'ro', markersize=8, label='Current')[0]
        self.axes[2, 1].legend()
        
        plt.tight_layout()
        try:
            plt.draw()
        except Exception:
            pass
    
    def log(self, t, q_act, qd_act, q_ref, qd_ref, qdd_ref, tau_lag, tau_ne, tau_ctrl, comp_time):
        """Log data point"""
        with self.data_lock:
            self.data['time'].append(t)
            self.data['q_actual'].append(q_act.copy())
            self.data['qd_actual'].append(qd_act.copy())
            self.data['q_ref'].append(q_ref.copy())
            self.data['qd_ref'].append(qd_ref.copy())
            self.data['qdd_ref'].append(qdd_ref.copy())
            self.data['tau_lag'].append(tau_lag.copy())
            self.data['tau_ne'].append(tau_ne.copy())
            self.data['tau_control'].append(tau_ctrl.copy())
            self.data['errors'].append((q_ref - q_act).copy())
            self.data['computation_times'].append(comp_time)
    
    def update_plots(self):
        """Update plots with latest data"""
        if not self.plot_realtime or len(self.data['time']) < 2:
            return
        
        with self.data_lock:
            t = np.array(self.data['time'])
            q_act = np.array(self.data['q_actual'])
            q_ref = np.array(self.data['q_ref'])
            tau_lag = np.array(self.data['tau_lag'])
            tau_ne = np.array(self.data['tau_ne'])
            tau_ctrl = np.array(self.data['tau_control'])
            
            # Limit data points for performance
            max_points = 500
            if len(t) > max_points:
                indices = np.linspace(0, len(t)-1, max_points, dtype=int)
                t = t[indices]
                q_act = q_act[indices]
                q_ref = q_ref[indices]
                tau_lag = tau_lag[indices]
                tau_ne = tau_ne[indices]
                tau_ctrl = tau_ctrl[indices]
        
        # Update plot 1: Joint Positions
        self.lines['q1_act'].set_data(t, q_act[:, 0])
        self.lines['q1_ref'].set_data(t, q_ref[:, 0])
        self.lines['q2_act'].set_data(t, q_act[:, 1])
        self.lines['q2_ref'].set_data(t, q_ref[:, 1])
        self.axes[0, 0].relim()
        self.axes[0, 0].autoscale_view()
        
        # Update plot 2: Errors
        errors = q_ref - q_act
        self.lines['error1'].set_data(t, errors[:, 0])
        self.lines['error2'].set_data(t, errors[:, 1])
        self.axes[0, 1].relim()
        self.axes[0, 1].autoscale_view()
        
        # Update plot 3: Torques Comparison
        self.lines['tau_lag1'].set_data(t, tau_lag[:, 0])
        self.lines['tau_ne1'].set_data(t, tau_ne[:, 0])
        self.lines['tau_lag2'].set_data(t, tau_lag[:, 1])
        self.lines['tau_ne2'].set_data(t, tau_ne[:, 1])
        self.axes[1, 0].relim()
        self.axes[1, 0].autoscale_view()
        
        # Update plot 4: Differences
        diff1 = tau_lag[:, 0] - tau_ne[:, 0]
        diff2 = tau_lag[:, 1] - tau_ne[:, 1]
        self.lines['diff1'].set_data(t, diff1)
        self.lines['diff2'].set_data(t, diff2)
        self.axes[1, 1].relim()
        self.axes[1, 1].autoscale_view()
        
        # Update plot 5: Control Torques
        self.lines['tau_ctrl1'].set_data(t, tau_ctrl[:, 0])
        self.lines['tau_ctrl2'].set_data(t, tau_ctrl[:, 1])
        self.axes[2, 0].relim()
        self.axes[2, 0].autoscale_view()
        
        # Update plot 6: End-effector Path (using forward kinematics)
        lag_model = LagrangianModel()
        ee_points = []
        for q in q_act[-100:]:  # Last 100 points
            ee = lag_model.forward_kinematics(q)
            ee_points.append(ee)
        
        if ee_points:
            ee_points = np.array(ee_points)
            self.lines['ee_path'].set_data(ee_points[:, 0], ee_points[:, 1])
            current_ee = ee_points[-1]
            self.lines['ee_current'].set_data([current_ee[0]], [current_ee[1]])
            self.axes[2, 1].relim()
            self.axes[2, 1].autoscale_view()
        
        try:
            plt.draw()
            plt.pause(0.001)
        except Exception:
            pass
    
    def save_metadata(self, params):
        """Save simulation metadata"""
        metadata = {
            'timestamp': datetime.now().isoformat(),
            'ros_version': 'ROS 2',
            'parameters': params,
            'data_points': len(self.data['time']),
            'simulation_time': self.data['time'][-1] if self.data['time'] else 0
        }
        
        with open(self.json_file, 'w') as f:
            json.dump(metadata, f, indent=2)
    
    def save_csv(self):
        """Save data to CSV file"""
        if len(self.data['time']) == 0:
            print("⚠️ No data to save")
            return
            
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Write header
            header = ['time']
            header += [f'q{i+1}' for i in range(2)]
            header += [f'qd{i+1}' for i in range(2)]
            header += [f'q_ref{i+1}' for i in range(2)]
            header += [f'qd_ref{i+1}' for i in range(2)]
            header += [f'qdd_ref{i+1}' for i in range(2)]
            header += [f'tau_lag{i+1}' for i in range(2)]
            header += [f'tau_ne{i+1}' for i in range(2)]
            header += [f'tau_control{i+1}' for i in range(2)]
            header += [f'error{i+1}' for i in range(2)]
            header += ['computation_time']
            
            writer.writerow(header)
            
            # Write data
            for i in range(len(self.data['time'])):
                row = [self.data['time'][i]]
                row += self.data['q_actual'][i].tolist()
                row += self.data['qd_actual'][i].tolist()
                row += self.data['q_ref'][i].tolist()
                row += self.data['qd_ref'][i].tolist()
                row += self.data['qdd_ref'][i].tolist()
                row += self.data['tau_lag'][i].tolist()
                row += self.data['tau_ne'][i].tolist()
                row += self.data['tau_control'][i].tolist()
                row += self.data['errors'][i].tolist()
                row += [self.data['computation_times'][i]]
                
                writer.writerow(row)
        
        print(f"✓ Data saved to: {self.csv_file}")
    
    def generate_final_report(self):
        """Generate final analysis report"""
        if len(self.data['time']) == 0:
            print("No data to analyze")
            return
        
        # Load data for analysis
        t = np.array(self.data['time'])
        q_act = np.array(self.data['q_actual'])
        q_ref = np.array(self.data['q_ref'])
        tau_lag = np.array(self.data['tau_lag'])
        tau_ne = np.array(self.data['tau_ne'])
        tau_ctrl = np.array(self.data['tau_control'])
        errors = np.array(self.data['errors'])
        
        # Create analysis figure
        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        fig.suptitle('Dynamics Analysis - Final Report', fontsize=16)
        
        # 1. RMS Errors over time
        window_size = min(50, len(t))
        rms_errors = []
        for i in range(len(t) - window_size):
            window_errors = errors[i:i+window_size]
            rms = np.sqrt(np.mean(window_errors**2, axis=0))
            rms_errors.append(rms)
        
        rms_errors = np.array(rms_errors)
        t_window = t[window_size//2:-window_size//2+1]
        
        axes[0, 0].plot(t_window, rms_errors[:, 0], 'b-', label='Joint 1 RMS')
        axes[0, 0].plot(t_window, rms_errors[:, 1], 'r-', label='Joint 2 RMS')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('RMS Error (rad)')
        axes[0, 0].set_title('RMS Tracking Error')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # 2. Model correlation
        axes[0, 1].scatter(tau_lag[:, 0], tau_ne[:, 0], alpha=0.5, label='Joint 1', s=10)
        axes[0, 1].scatter(tau_lag[:, 1], tau_ne[:, 1], alpha=0.5, label='Joint 2', s=10)
        
        # Add ideal line
        max_tau = max(np.max(np.abs(tau_lag)), np.max(np.abs(tau_ne)))
        axes[0, 1].plot([-max_tau, max_tau], [-max_tau, max_tau], 'k--', alpha=0.5)
        
        axes[0, 1].set_xlabel('Lagrangian Torque (N·m)')
        axes[0, 1].set_ylabel('Newton-Euler Torque (N·m)')
        axes[0, 1].set_title('Model Correlation')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        axes[0, 1].axis('equal')
        
        # 3. Error histogram
        axes[0, 2].hist(errors[:, 0], bins=50, alpha=0.7, label=f'Joint 1 (μ={np.mean(errors[:, 0]):.4f})')
        axes[0, 2].hist(errors[:, 1], bins=50, alpha=0.7, label=f'Joint 2 (μ={np.mean(errors[:, 1]):.4f})')
        axes[0, 2].set_xlabel('Tracking Error (rad)')
        axes[0, 2].set_ylabel('Frequency')
        axes[0, 2].set_title('Error Distribution')
        axes[0, 2].legend()
        axes[0, 2].grid(True)
        
        # 4. Torque frequency analysis
        from scipy import fftpack
        
        for i in range(2):
            torque_fft = fftpack.fft(tau_ctrl[:, i])
            freqs = fftpack.fftfreq(len(t), t[1]-t[0])
            pos_freqs = freqs[freqs > 0]
            torque_fft_pos = np.abs(torque_fft[freqs > 0])
            
            axes[1, 0].plot(pos_freqs[:50], torque_fft_pos[:50], 
                           label=f'Joint {i+1}', alpha=0.7)
        
        axes[1, 0].set_xlabel('Frequency (Hz)')
        axes[1, 0].set_ylabel('Amplitude')
        axes[1, 0].set_title('Torque Frequency Spectrum')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # 5. Energy comparison
        # Approximate power = torque * velocity
        qd_actual = np.array(self.data['qd_actual'])
        power_lag = np.abs(tau_lag * qd_actual)
        power_ne = np.abs(tau_ne * qd_actual)
        
        energy_lag = np.cumsum(power_lag, axis=0) * (t[1] - t[0])
        energy_ne = np.cumsum(power_ne, axis=0) * (t[1] - t[0])
        
        axes[1, 1].plot(t, energy_lag[:, 0], 'b-', label='Joint 1 (Lag)', alpha=0.7)
        axes[1, 1].plot(t, energy_ne[:, 0], 'b--', label='Joint 1 (NE)', alpha=0.7)
        axes[1, 1].plot(t, energy_lag[:, 1], 'r-', label='Joint 2 (Lag)', alpha=0.7)
        axes[1, 1].plot(t, energy_ne[:, 1], 'r--', label='Joint 2 (NE)', alpha=0.7)
        
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Energy (J)')
        axes[1, 1].set_title('Cumulative Energy')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        # 6. Statistics table
        axes[1, 2].axis('off')
        
        stats_text = f"""
        FINAL STATISTICS
        {'='*40}
        Simulation Time: {t[-1]:.2f} s
        Data Points: {len(t)}
        
        TRACKING PERFORMANCE:
        Joint 1 RMS Error: {np.sqrt(np.mean(errors[:, 0]**2)):.6f} rad
        Joint 2 RMS Error: {np.sqrt(np.mean(errors[:, 1]**2)):.6f} rad
        Max Error: [{np.max(np.abs(errors[:, 0])):.6f}, {np.max(np.abs(errors[:, 1])):.6f}] rad
        
        MODEL COMPARISON:
        τ1 Correlation: {np.corrcoef(tau_lag[:, 0], tau_ne[:, 0])[0,1]:.6f}
        τ2 Correlation: {np.corrcoef(tau_lag[:, 1], tau_ne[:, 1])[0,1]:.6f}
        Mean Difference: [{np.mean(tau_lag[:, 0]-tau_ne[:, 0]):.6f}, {np.mean(tau_lag[:, 1]-tau_ne[:, 1]):.6f}] N·m
        
        COMPUTATION:
        Avg Time: {np.mean(self.data['computation_times'])*1000:.3f} ms
        Max Time: {np.max(self.data['computation_times'])*1000:.3f} ms
        """
        
        axes[1, 2].text(0.1, 0.95, stats_text, transform=axes[1, 2].transAxes,
                       fontfamily='monospace', fontsize=9, verticalalignment='top')
        
        plt.tight_layout()
        
        # Save report
        report_file = self.log_dir / f'analysis_report_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png'
        plt.savefig(report_file, dpi=300, bbox_inches='tight')
        if not HEADLESS_PLOT:
            try:
                plt.show()
            except Exception:
                pass
        
        print(f"✓ Analysis report saved to: {report_file}")
        
        return stats_text


# ============================================================================
# MAIN ROS 2 NODE - MODIFIED FOR simROS2 COMPATIBILITY
# ============================================================================

class UnifiedDynamicsNode(Node):
    """Unified ROS 2 node for dynamics control and simulation"""
    
    def __init__(self):
        super().__init__('unified_dynamics_node')
        
        # Signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Initialize parameters
        self.init_parameters()
        
        # Initialize models
        self.init_models()
        
        # Initialize data logger
        self.init_logger()
        
        # Initialize ROS 2 components
        self.init_ros_components()
        
        # Initialize control variables
        self.init_control_variables()
        
        # Send handshake to Lua script
        self.publish_handshake()
        
        # Start control loop
        self.start_control_loop()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("UNIFIED DYNAMICS CONTROL NODE INITIALIZED")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Robot: {self.robot_name}")
        self.get_logger().info(f"Trajectory: {self.trajectory_type}")
        self.get_logger().info(f"Control Frequency: {self.control_freq} Hz")
        self.get_logger().info(f"Control Mode: {self.control_mode}")
        self.get_logger().info(f"Models: Lagrangian={self.use_lagrangian}, Newton-Euler={self.use_newton_euler}")
        self.get_logger().info(f"Data Logging: {self.log_data}")
        self.get_logger().info(f"ROS Domain ID: {os.environ.get('ROS_DOMAIN_ID', '0')}")
        self.get_logger().info("=" * 60)
    
    def init_parameters(self):
        """Initialize all parameters"""
        # Robot parameters
        self.declare_parameter('robot_name', 'planar_2dof')
        self.declare_parameter('joint_names', ['joint1', 'joint2'])
        self.declare_parameter('link_lengths', [0.3, 0.3])
        self.declare_parameter('masses', [1.0, 1.0])
        self.declare_parameter('gravity', 9.81)
        
        # Control parameters
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('control_mode', 'external_ros')  # 'internal', 'external_ros', 'hybrid'
        self.declare_parameter('use_lagrangian', True)
        self.declare_parameter('use_newton_euler', True)
        self.declare_parameter('kp', [150.0, 120.0])
        self.declare_parameter('kd', [25.0, 20.0])
        self.declare_parameter('max_torque', 50.0)
        
        # Trajectory parameters
        self.declare_parameter('trajectory_type', 'sine')
        
        # ROS topics - UPDATED TO MATCH LUA SCRIPT
        self.declare_parameter('joint_state_topic', '/robot/joint_states')
        self.declare_parameter('torque_command_topic', '/robot/joint_torque_cmd')
        self.declare_parameter('reference_trajectory_topic', '/robot/reference_trajectory')
        self.declare_parameter('control_mode_topic', '/sim/control_mode')
        self.declare_parameter('heartbeat_topic', '/ros/heartbeat')
        self.declare_parameter('sim_status_topic', '/sim/status')
        self.declare_parameter('handshake_topic', '/dynamics/handshake')
        
        # Data logging
        self.declare_parameter('log_data', True)
        self.declare_parameter('log_directory', './logs')
        self.declare_parameter('plot_realtime', True)
        self.declare_parameter('save_interval', 10.0)  # seconds
        
        # Connection monitoring
        self.declare_parameter('connection_timeout', 2.0)  # seconds
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.joint_names = self.get_parameter('joint_names').value
        self.link_lengths = self.get_parameter('link_lengths').value
        self.masses = self.get_parameter('masses').value
        self.gravity = self.get_parameter('gravity').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.control_mode = self.get_parameter('control_mode').value
        self.use_lagrangian = self.get_parameter('use_lagrangian').value
        self.use_newton_euler = self.get_parameter('use_newton_euler').value
        self.kp = np.array(self.get_parameter('kp').value)
        self.kd = np.array(self.get_parameter('kd').value)
        self.max_torque = self.get_parameter('max_torque').value
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.torque_command_topic = self.get_parameter('torque_command_topic').value
        self.reference_trajectory_topic = self.get_parameter('reference_trajectory_topic').value
        self.control_mode_topic = self.get_parameter('control_mode_topic').value
        self.heartbeat_topic = self.get_parameter('heartbeat_topic').value
        self.sim_status_topic = self.get_parameter('sim_status_topic').value
        self.handshake_topic = self.get_parameter('handshake_topic').value
        self.log_data = self.get_parameter('log_data').value
        self.log_directory = self.get_parameter('log_directory').value
        self.plot_realtime = self.get_parameter('plot_realtime').value
        self.save_interval = self.get_parameter('save_interval').value
        self.connection_timeout = self.get_parameter('connection_timeout').value
        
        # Add parameter callback for runtime changes
        self.add_on_set_parameters_callback(self.parameters_callback)
    
    def init_models(self):
        """Initialize dynamics models"""
        self.lagrangian_model = LagrangianModel(
            masses=self.masses,
            lengths=self.link_lengths,
            gravity=self.gravity
        )
        
        self.newton_euler_model = NewtonEulerModel(
            masses=self.masses,
            lengths=self.link_lengths,
            gravity=self.gravity
        )
        
        self.trajectory_generator = TrajectoryGenerator(
            traj_type=self.trajectory_type,
            link_lengths=self.link_lengths
        )
    
    def init_logger(self):
        """Initialize data logger"""
        if self.log_data:
            self.logger = DataLogger(self.log_directory)
            
            # Save initial metadata
            params = {
                'robot_name': self.robot_name,
                'joint_names': self.joint_names,
                'link_lengths': self.link_lengths,
                'masses': self.masses,
                'gravity': self.gravity,
                'control_frequency': self.control_freq,
                'control_mode': self.control_mode,
                'trajectory_type': self.trajectory_type,
                'kp': self.kp.tolist(),
                'kd': self.kd.tolist(),
                'max_torque': self.max_torque
            }
            self.logger.save_metadata(params)
        else:
            self.logger = None
    
    def init_ros_components(self):
        """Initialize ROS 2 publishers and subscribers"""
        # QoS profile for simROS2 compatibility - USING BEST_EFFORT
        sim_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT for simROS2 compatibility
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers to CoppeliaSim (use sim_qos_profile)
        # Publishers use simple String payloads for arrays (CSV)
        self.torque_pub = self.create_publisher(
            String,
            self.torque_command_topic,
            sim_qos_profile
        )

        # Reference trajectory published as CSV in String
        self.reference_trajectory_pub = self.create_publisher(
            String,
            self.reference_trajectory_topic,
            sim_qos_profile
        )
        
        self.control_mode_pub = self.create_publisher(
            String,
            self.control_mode_topic,
            sim_qos_profile
        )
        
        self.heartbeat_pub = self.create_publisher(
            Bool,
            self.heartbeat_topic,
            sim_qos_profile
        )
        
        self.handshake_pub = self.create_publisher(
            String,
            self.handshake_topic,
            sim_qos_profile
        )
        
        # Subscribers from CoppeliaSim
        # CoppeliaSim publishes joint-state as CSV string on String topic
        self.joint_state_sub = self.create_subscription(
            String,
            self.joint_state_topic,
            self.joint_state_callback,
            sim_qos_profile
        )
        
        self.sim_status_sub = self.create_subscription(
            String,
            self.sim_status_topic,
            self.sim_status_callback,
            sim_qos_profile
        )
        
        # TF Broadcaster for visualization (uses default QoS)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Status publisher for console (reliable QoS)
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.status_pub = self.create_publisher(
            String, 
            '/dynamics/status', 
            status_qos
        )
        
        self.get_logger().info("ROS 2 components initialized with BEST_EFFORT QoS")
    
    def init_control_variables(self):
        """Initialize control variables"""
        # State variables
        self.joint_positions = np.zeros(2)
        self.joint_velocities = np.zeros(2)
        self.joint_accelerations = np.zeros(2)
        
        # Timing
        self.last_time = self.get_clock().now()
        self.sim_time = 0.0
        self.last_save_time = 0.0
        self.last_heartbeat_time = 0.0
        
        # Connection monitoring
        self.last_joint_msg_time = 0.0
        self.sim_connected = False
        self.received_first_message = False
        
        # Previous values for differentiation
        self.prev_positions = np.zeros(2)
        self.prev_velocities = np.zeros(2)
        
        # Control flags
        self.is_running = True
        self.handshake_sent = False
    
    def start_control_loop(self):
        """Start the main control loop timer"""
        control_period = 1.0 / self.control_freq
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        # Status update timer
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Heartbeat timer (1 Hz)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        
        # Plot update timer (if realtime plotting is enabled)
        if self.log_data and self.plot_realtime:
            self.plot_timer = self.create_timer(0.5, self.update_plots)
        
        self.get_logger().info(f"Control loop started at {self.control_freq} Hz")
    
    def publish_handshake(self):
        """Publish handshake to Lua script"""
        msg = String()
        msg.data = "READY"
        self.handshake_pub.publish(msg)
        self.handshake_sent = True
        self.get_logger().info("✓ Handshake sent to CoppeliaSim")
    
    def joint_state_callback(self, msg):
        """Callback for joint state updates from CoppeliaSim"""
        try:
            # Update connection time
            self.last_joint_msg_time = time.time()

            # Expect String CSV with data = "pos1,pos2,vel1,vel2,effort1,effort2"
            data = []
            if hasattr(msg, 'data'):
                raw = msg.data
                if isinstance(raw, str):
                    parts = [p for p in raw.split(',') if p != '']
                    conv = []
                    for p in parts:
                        try:
                            conv.append(float(p))
                        except Exception:
                            conv.append(0.0)
                    data = conv
                else:
                    # Fallback if a sequence-like object arrives
                    try:
                        data = list(raw)
                    except Exception:
                        data = []

            if len(data) >= 2:
                self.joint_positions = np.array([data[0], data[1]])

            # Velocities
            if len(data) >= 4 and data[2] is not None and data[3] is not None:
                self.joint_velocities = np.array([data[2], data[3]])
            else:
                # Estimate velocities
                current_time = self.get_clock().now()
                dt = (current_time - self.last_time).nanoseconds * 1e-9
                if dt > 0.001:
                    self.joint_velocities = (self.joint_positions - self.prev_positions) / dt
                    self.joint_accelerations = (self.joint_velocities - self.prev_velocities) / dt
                    self.prev_velocities = self.joint_velocities.copy()
                self.prev_positions = self.joint_positions.copy()
                self.last_time = current_time

            if not self.received_first_message:
                self.received_first_message = True
                self.sim_connected = True
                self.get_logger().info("✓ First joint state received from CoppeliaSim")
                self.get_logger().info(f"  Initial positions: [{self.joint_positions[0]:.3f}, {self.joint_positions[1]:.3f}] rad")
                
        except Exception as e:
            self.get_logger().error(f'Error in joint state callback: {str(e)}')
    
    def sim_status_callback(self, msg):
        """Callback for simulation status messages"""
        if msg and msg.data:
            self.get_logger().info(f"Simulation status: {msg.data}")
    
    def control_loop(self):
        """Main control loop - called at control frequency"""
        # Check connection
        current_time = time.time()
        if current_time - self.last_joint_msg_time > self.connection_timeout:
            if self.sim_connected:
                self.sim_connected = False
                self.get_logger().warn(f"No joint states for {self.connection_timeout}s. Simulation may be paused.")
            return
        
        if not self.received_first_message:
            if self.sim_time > 5.0:
                self.get_logger().warn("No joint state messages received. Check CoppeliaSim connection.")
            return
        
        start_time = time.time()
        
        # Update simulation time
        current_ros_time = self.get_clock().now()
        if self.sim_time == 0:
            self.sim_time = 0.001
        else:
            dt = (current_ros_time - self.last_time).nanoseconds * 1e-9
            self.sim_time += dt
        
        # Generate reference trajectory
        q_ref, qd_ref, qdd_ref = self.trajectory_generator.generate(self.sim_time)
        
        # Publish reference trajectory to CoppeliaSim
        self.publish_reference_trajectory(q_ref, qd_ref, qdd_ref)
        
        # Calculate tracking errors
        error = q_ref - self.joint_positions
        error_dot = qd_ref - self.joint_velocities
        
        # Calculate feedforward torques using dynamics models
        tau_lagrangian = np.zeros(2)
        tau_newton_euler = np.zeros(2)
        
        if self.use_lagrangian:
            tau_lagrangian = self.lagrangian_model.compute_torques(
                self.joint_positions,
                self.joint_velocities,
                qdd_ref
            )
        
        if self.use_newton_euler:
            tau_newton_euler = self.newton_euler_model.compute_torques(
                self.joint_positions,
                self.joint_velocities,
                qdd_ref
            )
        
        # PD feedback control
        tau_pd = self.kp * error + self.kd * error_dot
        
        # Combine feedforward and feedback based on control mode
        # IMPORTANT: Don't send torques in 'internal' mode
        if self.control_mode == 'internal':
            # In internal mode, Lua script controls itself
            # Don't publish any torque commands
            tau_control = np.zeros(2)
            if self.sim_time % 5.0 < 0.01:  # Log every 5 seconds
                self.get_logger().info("Internal control mode - Python node not sending torques")
        else:
            # external_ros or hybrid mode
            if self.use_lagrangian and self.use_newton_euler:
                tau_ff = (tau_lagrangian + tau_newton_euler) / 2
            elif self.use_lagrangian:
                tau_ff = tau_lagrangian
            else:
                tau_ff = tau_newton_euler
            
            tau_control = tau_pd + tau_ff
            
            # Apply torque limits
            tau_control = np.clip(tau_control, -self.max_torque, self.max_torque)
            
            # Publish torque commands
            self.publish_torque_commands(tau_control)
        
        # Publish TF frames for visualization
        self.publish_tf_frames()
        
        # Log data
        if self.log_data and self.logger:
            comp_time = time.time() - start_time
            self.logger.log(
                self.sim_time,
                self.joint_positions,
                self.joint_velocities,
                q_ref,
                qd_ref,
                qdd_ref,
                tau_lagrangian,
                tau_newton_euler,
                tau_control,
                comp_time
            )
            
            # Auto-save data periodically
            if self.sim_time - self.last_save_time > self.save_interval:
                self.logger.save_csv()
                self.last_save_time = self.sim_time
        
        # Update timing
        self.last_time = current_ros_time
    
    def publish_torque_commands(self, torques):
        """Publish torque commands to CoppeliaSim"""
        # Publish torques as CSV string on String topic
        msg = String()
        msg.data = f"{float(torques[0]):.6f},{float(torques[1]):.6f}"
        self.torque_pub.publish(msg)
    
    def publish_reference_trajectory(self, positions, velocities, accelerations):
        """Publish reference trajectory to CoppeliaSim"""
        # Publish trajectory as CSV string: pos1,pos2,vel1,vel2,acc1,acc2
        vals = list(positions.tolist()) + list(velocities.tolist()) + list(accelerations.tolist())
        msg = String()
        msg.data = ','.join([f"{float(v):.6f}" for v in vals])
        self.reference_trajectory_pub.publish(msg)
    
    def publish_control_mode(self, mode):
        """Publish control mode to CoppeliaSim"""
        msg = String()
        msg.data = mode
        self.control_mode_pub.publish(msg)
        self.get_logger().info(f"Published control mode: {mode}")
    
    def publish_heartbeat(self):
        """Publish heartbeat signal to CoppeliaSim"""
        msg = Bool()
        msg.data = True
        self.heartbeat_pub.publish(msg)
    
    def publish_tf_frames(self):
        """Publish TF frames for visualization"""
        try:
            # Base frame
            t_base = TransformStamped()
            t_base.header.stamp = self.get_clock().now().to_msg()
            t_base.header.frame_id = 'world'
            t_base.child_frame_id = 'base_link'
            t_base.transform.translation.x = 0.0
            t_base.transform.translation.y = 0.0
            t_base.transform.translation.z = 0.0
            t_base.transform.rotation.x = 0.0
            t_base.transform.rotation.y = 0.0
            t_base.transform.rotation.z = 0.0
            t_base.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t_base)
            
            # Joint 1 frame
            t1 = TransformStamped()
            t1.header.stamp = self.get_clock().now().to_msg()
            t1.header.frame_id = 'base_link'
            t1.child_frame_id = 'link1'
            t1.transform.translation.x = self.link_lengths[0]/2 * math.cos(self.joint_positions[0])
            t1.transform.translation.y = self.link_lengths[0]/2 * math.sin(self.joint_positions[0])
            t1.transform.translation.z = 0.0
            
            q1 = self.euler_to_quaternion(0, 0, self.joint_positions[0])
            t1.transform.rotation.x = q1[0]
            t1.transform.rotation.y = q1[1]
            t1.transform.rotation.z = q1[2]
            t1.transform.rotation.w = q1[3]
            self.tf_broadcaster.sendTransform(t1)
            
            # Joint 2 / End-effector frame
            t2 = TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg()
            t2.header.frame_id = 'link1'
            t2.child_frame_id = 'end_effector'
            
            x = self.link_lengths[0] + self.link_lengths[1]/2 * math.cos(self.joint_positions[1])
            y = self.link_lengths[1]/2 * math.sin(self.joint_positions[1])
            
            t2.transform.translation.x = x
            t2.transform.translation.y = y
            t2.transform.translation.z = 0.0
            
            q2 = self.euler_to_quaternion(0, 0, self.joint_positions[1])
            t2.transform.rotation.x = q2[0]
            t2.transform.rotation.y = q2[1]
            t2.transform.rotation.z = q2[2]
            t2.transform.rotation.w = q2[3]
            self.tf_broadcaster.sendTransform(t2)
        except Exception as e:
            self.get_logger().error(f"Error publishing TF frames: {str(e)}")
    
    def publish_status(self):
        """Periodic status publication"""
        status_msg = String()
        
        if self.received_first_message:
            if self.logger and len(self.logger.data['time']) > 0:
                errors = self.logger.data['errors']
                if errors:
                    # Calculate statistics from recent data
                    recent_errors = errors[-min(100, len(errors)):]
                    avg_error = np.mean(np.abs(recent_errors))
                    rms_error = np.sqrt(np.mean(np.array(recent_errors)**2))
                    
                    status = (f"Time: {self.sim_time:.2f}s | "
                             f"Data Points: {len(self.logger.data['time'])} | "
                             f"Avg Error: {avg_error:.4f}rad | "
                             f"RMS Error: {rms_error:.4f}rad | "
                             f"Control Mode: {self.control_mode}")
                else:
                    status = (f"Time: {self.sim_time:.2f}s | "
                             f"Control Mode: {self.control_mode} | "
                             f"Sim Connected: {self.sim_connected}")
            else:
                status = (f"Time: {self.sim_time:.2f}s | "
                         f"Control Mode: {self.control_mode} | "
                         f"Waiting for data...")
            
            # Log to console every 10 seconds
            if int(self.sim_time) % 10 == 0 and int(self.sim_time) != int(self.sim_time - 0.1):
                self.get_logger().info(status)
        else:
            status = "Waiting for joint states from CoppeliaSim..."
            if self.sim_time > 2.0 and int(self.sim_time) % 5 == 0:
                self.get_logger().warn(status)
        
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def update_plots(self):
        """Update real-time plots"""
        if self.logger and self.plot_realtime:
            self.logger.update_plots()
    
    def parameters_callback(self, params):
        """Handle parameter changes at runtime"""
        for param in params:
            if param.name == 'trajectory_type':
                self.trajectory_type = param.value
                self.trajectory_generator = TrajectoryGenerator(
                    traj_type=self.trajectory_type,
                    link_lengths=self.link_lengths
                )
                self.get_logger().info(f"Trajectory changed to: {self.trajectory_type}")
            
            elif param.name == 'control_mode':
                old_mode = self.control_mode
                self.control_mode = param.value
                self.publish_control_mode(self.control_mode)
                self.get_logger().info(f"Control mode changed from {old_mode} to {self.control_mode}")
            
            elif param.name == 'kp':
                self.kp = np.array(param.value)
                self.get_logger().info(f"KP gains changed to: {self.kp}")
            
            elif param.name == 'kd':
                self.kd = np.array(param.value)
                self.get_logger().info(f"KD gains changed to: {self.kd}")
        
        return SetParametersResult(successful=True)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C for graceful shutdown"""
        self.get_logger().info("Shutdown signal received")
        self.shutdown()
    
    def shutdown(self):
        """Clean shutdown procedure"""
        self.is_running = False
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("SHUTTING DOWN DYNAMICS CONTROL NODE")
        self.get_logger().info("="*60)
        
        if self.logger:
            # Save final data
            self.logger.save_csv()
            
            # Generate final report
            try:
                report = self.logger.generate_final_report()
                
                # Log report to file
                report_file = Path(self.log_directory) / f'final_report_{datetime.now().strftime("%Y%m%d_%H%M%S")}.txt'
                with open(report_file, 'w') as f:
                    f.write(report)
                
                print(f"✓ Final report saved to: {report_file}")
            except Exception as e:
                self.get_logger().error(f"Error generating report: {str(e)}")
        
        # Stop ROS 2
        self.destroy_node()
        rclpy.shutdown()
        
        print("\n" + "="*60)
        print("DYNAMICS CONTROL NODE SHUTDOWN COMPLETE")
        print("="*60)
        
        # Keep plots open (if not headless)
        if self.logger and self.plot_realtime and not HEADLESS_PLOT:
            try:
                plt.ioff()
                plt.show()
            except Exception:
                pass
        
        sys.exit(0)


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main(args=None):
    """Main function"""
    print("\n" + "="*60)
    print("UNIFIED 2-DOF DYNAMICS CONTROL NODE")
    print("COMPATIBLE WITH COPPELIASIM simROS2")
    print("VERSION: 1.2 (مع التعديلات المطلوبة)")
    print("="*60)
    print("Features:")
    print("  • Lagrangian Dynamics Model")
    print("  • Newton-Euler Recursive Algorithm")
    print("  • Real-time Control (100 Hz)")
    print("  • Multiple Trajectories (sine, circle, point-to-point)")
    print("  • Real-time Plotting and Data Logging")
    print("  • ROS 2 Integration with CoppeliaSim simROS2")
    print("  • Automatic Analysis and Report Generation")
    print("="*60)
    print("Important Notes:")
    print("  1. Make sure CoppeliaSim is running with simROS2 plugin")
    print("  2. Load the Lua script in CoppeliaSim")
    print("  3. Topics match the Lua script configuration")
    print("  4. ROS_DOMAIN_ID must match between CoppeliaSim and ROS")
    print("="*60)
    
    # Initialize ROS 2
    rclpy.init(args=args)
    
    try:
        # Create and run node
        node = UnifiedDynamicsNode()
        
        print("\nStarting control node...")
        print("Press Ctrl+C to stop and generate final report")
        print("="*60 + "\n")
        
        # Spin node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt received")
        if 'node' in locals():
            node.shutdown()
    
    except Exception as e:
        print(f"\nError: {str(e)}")
        import traceback
        traceback.print_exc()
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()
    
    finally:
        print("\nProgram terminated")


if __name__ == '__main__':
    import os
    main()