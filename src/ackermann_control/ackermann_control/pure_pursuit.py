#!/usr/bin/env python3
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def lemniscate(a, step):
    n = max(int(5.3 * a / step), 100)
    t0 = math.asin(1.0 / math.sqrt(3.0))
    pts = []
    for i in range(n + 1):
        t = t0 + 2.0 * math.pi * i / n
        s, c = math.sin(t), math.cos(t)
        d = 1.0 + s * s
        pts.append((a * c / d, a * s * c / d))
    x0, y0 = pts[0]
    return [(-(x - x0), -(y - y0)) for x, y in pts]


def build_path(kind, R, S, A, step=0.05):
    """Trayectoria en el marco inicial del robot (arranca en 0,0 mirando a +X)."""
    pts = []
    if kind == 'infinity':
        return lemniscate(A, step)
    if kind == 'circle':
        n = max(int(2 * math.pi * R / step), 20)
        for i in range(n + 1):
            t = 2 * math.pi * i / n
            pts.append((R * math.sin(t), R * (1 - math.cos(t))))
    else:  # 'oval': dos rectas de largo S unidas por dos semicircunferencias de radio R
        n = max(int(S / step), 2)
        m = max(int(math.pi * R / step), 10)
        for i in range(n):
            pts.append((S * i / n, 0.0))
        for i in range(m):
            a = -math.pi / 2 + math.pi * i / m
            pts.append((S + R * math.cos(a), R + R * math.sin(a)))
        for i in range(n):
            pts.append((S - S * i / n, 2 * R))
        for i in range(m + 1):
            a = math.pi / 2 + math.pi * i / m
            pts.append((R * math.cos(a), R + R * math.sin(a)))
    return pts


def max_curvature(pts):
    """Curvatura maxima de la trayectoria (formula de Menger con 3 puntos)."""
    kmax = 0.0
    for i in range(1, len(pts) - 1):
        ax, ay = pts[i - 1]
        bx, by = pts[i]
        cx, cy = pts[i + 1]
        cross = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
        den = math.hypot(bx - ax, by - ay) * math.hypot(cx - bx, cy - by) * math.hypot(cx - ax, cy - ay)
        if den > 1e-9:
            kmax = max(kmax, 2.0 * cross / den)
    return kmax


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('path_type', 'infinity')  # infinity | circle | oval
        self.declare_parameter('size', 3.0)              # infinity: semi-largo a (radio minimo = a/3)
        self.declare_parameter('radius', 1.5)            # circle / oval
        self.declare_parameter('straight', 2.0)          # oval
        self.declare_parameter('speed', 0.4)
        self.declare_parameter('lookahead', 0.6)
        self.declare_parameter('wheelbase', 0.24)
        self.declare_parameter('rear_offset', 0.12)      # eje trasero -> origen del odom
        self.declare_parameter('max_steer', 0.28)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('out_dir', os.path.expanduser('~/ackermann_results'))

        g = lambda k: self.get_parameter(k).value
        self.kind = 'infinity' if g('path_type') == 'lemniscate' else g('path_type')
        self.A = g('size')
        self.R = g('radius')
        self.S = g('straight')
        self.v = g('speed')
        self.Ld = g('lookahead')
        self.L = g('wheelbase')
        self.off = g('rear_offset')
        self.kmax = math.tan(g('max_steer')) / self.L
        self.out_dir = g('out_dir')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(1.0 / g('rate'), self.control)

        self.pose = None
        self.path = None
        self.idx = 0
        self.t0 = None
        self.done = False
        self.saved = False
        self.rows = []
        self.get_logger().info(
            f'Pure Pursuit listo: {self.kind}, v={self.v} m/s, Ld={self.Ld} m, '
            f'curvatura max del robot={self.kmax:.2f} 1/m')

    def odom_cb(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def init_path(self):
        x0, y0, th0 = self.pose
        c, s = math.cos(th0), math.sin(th0)
        raw = build_path(self.kind, self.R, self.S, self.A)
        self.path = [(x0 + px * c - py * s, y0 + px * s + py * c) for px, py in raw]
        self.t0 = self.now()

        length = sum(math.hypot(self.path[i + 1][0] - self.path[i][0],
                                self.path[i + 1][1] - self.path[i][1])
                     for i in range(len(self.path) - 1))
        kp = max_curvature(raw)
        self.get_logger().info(
            f'Trayectoria {self.kind}: {len(self.path)} puntos, longitud {length:.1f} m, '
            f'curvatura max {kp:.2f} 1/m (radio min {1.0 / max(kp, 1e-6):.2f} m), '
            f'inicio en ({x0:.2f}, {y0:.2f})')
        if kp > self.kmax:
            self.get_logger().warn(
                'La trayectoria exige mas curvatura de la que el robot puede dar: '
                'aumenta size/radius o el robot no podra seguirla.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def control(self):
        if self.pose is None or self.done:
            return
        if self.path is None:
            self.init_path()

        x, y, yaw = self.pose
        rx = x - self.off * math.cos(yaw)   # Pure Pursuit se define en el eje trasero
        ry = y - self.off * math.sin(yaw)
        n = len(self.path)

        # punto mas cercano: ventana hacia adelante para no saltar entre ramas en el cruce del 8
        best, bd = self.idx, float('inf')
        for i in range(self.idx, min(self.idx + 60, n)):
            d = math.hypot(self.path[i][0] - rx, self.path[i][1] - ry)
            if d < bd:
                best, bd = i, d
        self.idx = best
        cte = bd

        # punto objetivo a distancia Ld, siempre hacia adelante en la trayectoria
        j = self.idx
        while j < n - 1 and math.hypot(self.path[j][0] - rx, self.path[j][1] - ry) < self.Ld:
            j += 1
        tx, ty = self.path[j]
        dx, dy = tx - rx, ty - ry
        dist = math.hypot(dx, dy)

        if (j == n - 1 and dist < 0.15) or self.idx >= n - 2:
            self.finish()
            return

        alpha = wrap(math.atan2(dy, dx) - yaw)
        kappa = 2.0 * math.sin(alpha) / max(dist, 1e-3)
        kappa = max(-self.kmax, min(self.kmax, kappa))
        w = self.v * kappa
        delta = math.atan(self.L * kappa)

        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = w
        self.pub.publish(msg)

        self.rows.append([self.now() - self.t0, x, y, yaw, self.v, w, delta, tx, ty, cte])

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())

    def finish(self):
        if self.saved:
            return
        self.saved = True
        self.done = True
        self.stop()
        os.makedirs(self.out_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        run = os.path.join(self.out_dir, f'run_{self.kind}_{stamp}.csv')
        with open(run, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw', 'v_cmd', 'w_cmd', 'delta_est', 'target_x', 'target_y', 'cte'])
            w.writerows(self.rows)
        if self.path:
            with open(os.path.join(self.out_dir, f'path_{self.kind}_{stamp}.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['x', 'y'])
                w.writerows(self.path)
        if self.rows:
            ctes = [r[9] for r in self.rows]
            rmse = math.sqrt(sum(c * c for c in ctes) / len(ctes))
            self.get_logger().info(
                f'FIN. Duracion {self.rows[-1][0]:.1f} s | RMSE cte = {rmse:.3f} m | '
                f'cte max = {max(ctes):.3f} m | CSV: {run}')


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrumpido: guardando resultados parciales')
        node.finish()
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
