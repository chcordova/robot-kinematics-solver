import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from abc import ABC, abstractmethod

# ------------------------------------------------------------------------------
# 1. LIBRARY IMPORTS
# ------------------------------------------------------------------------------
"""
Se realizan las importaciones de RTB aquí, con un try-except para robustez
"""
try:
    from roboticstoolbox import DHRobot, RevoluteDH
    from spatialmath import SE3
    RTB_INSTALLED = True
except ImportError:
    RTB_INSTALLED = False

# ------------------------------------------------------------------------------
# 2. ABSTRACT BASE CLASS (THE INTERFACE / THE "CONTRACT")
# ------------------------------------------------------------------------------
class KinematicsSolver(ABC):
    """
    Interfaz base para definir operaciones de cinemática directa e inversa
    para un brazo robótico con 3 grados de libertad.
    """
    def __init__(self, lengths):
        if len(lengths) != 3:
            raise ValueError("Se requieren 3 longitudes para los eslabones [L1, L2, L3].")
        self.lengths = lengths
        self.L1, self.L2, self.L3 = self.lengths
        super().__init__()

    @abstractmethod
    def forward(self, thetas):
        """
        Calcula la posición del efector final a partir de ángulos articulares.
        Parámetros:
            thetas (list): ángulos [θ1, θ2, θ3] en radianes.
        Retorna:
            tuple: posición final y lista de posiciones articulares.
        """
        pass

    @abstractmethod
    def inverse(self, target_pos, elbow_config='up', q0=None):
        """
        Calcula los ángulos articulares necesarios para alcanzar una posición.
        Parámetros:
            target_pos (list): coordenadas objetivo [x, y, z].
            elbow_config (str): configuración del codo ('up' o 'down').
            q0 (list, opcional): estimación inicial para métodos iterativos.
        Retorna:
            list: ángulos articulares si hay solución, sino None.
        """
        pass

# ------------------------------------------------------------------------------
# 3. CONCRETE STRATEGIES (IMPLEMENTATIONS)
# ------------------------------------------------------------------------------
class GeometricSolver(KinematicsSolver):
    """
    Solver basado en trigonometría para CD/CI. Se asume un modelo articulado
    tipo RRR con planos de movimiento XY y XZ.
    """
    def forward(self, thetas):
        theta1, theta2, theta3 = thetas
        p0 = np.array([0, 0, 0])
        p1 = np.array([0, 0, self.L1])

        r2 = self.L2 * np.cos(theta2)
        x2 = r2 * np.cos(theta1)
        y2 = r2 * np.sin(theta1)
        z2 = self.L1 + self.L2 * np.sin(theta2)
        p2 = np.array([x2, y2, z2])

        theta23 = theta2 + theta3
        r3 = self.L3 * np.cos(theta23)
        x3 = (r2 + r3) * np.cos(theta1)
        y3 = (r2 + r3) * np.sin(theta1)
        z3 = self.L1 + self.L2 * np.sin(theta2) + self.L3 * np.sin(theta23)
        p3 = np.array([x3, y3, z3])

        return p3, [p0, p1, p2, p3]

    def inverse(self, target_pos, elbow_config='up', q0=None):
        x, y, z = target_pos
        theta1 = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2)
        z_prime = z - self.L1
        d = np.sqrt(r**2 + z_prime**2)

        if d > self.L2 + self.L3 or d < abs(self.L2 - self.L3):
            return None

        alpha = np.arctan2(z_prime, r)
        beta = np.arccos(np.clip((d**2 + self.L2**2 - self.L3**2)/(2*d*self.L2), -1.0, 1.0))
        cos_theta3 = (d**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)

        if elbow_config == 'up':
            theta2 = alpha + beta
            theta3 = -np.arccos(np.clip(cos_theta3, -1.0, 1.0))
        else:
            theta2 = alpha - beta
            theta3 = np.arccos(np.clip(cos_theta3, -1.0, 1.0))

        return [theta1, theta2, theta3]

class MatrixDHSolver(KinematicsSolver):
    """
    Solver que utiliza la convención de Denavit-Hartenberg y matrices homogéneas
    para resolver CD y CI.
    """
    def __init__(self, lengths):
        super().__init__(lengths)
        self.dh_params = np.array([
            [0, self.L1, 0, 90],
            [0, 0, self.L2, 0],
            [0, 0, self.L3, 0]
        ])

    def _dh_matrix(self, theta, d, a, alpha):
        alpha_rad = np.deg2rad(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha_rad), np.sin(alpha_rad)
        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def _inv_transform(self, T):
        R_t = T[:3, :3].T
        p = T[:3, 3]
        p_inv = -R_t @ p
        T_inv = np.identity(4)
        T_inv[:3, :3] = R_t
        T_inv[:3, 3] = p_inv
        return T_inv

    def forward(self, thetas):
        T = np.identity(4)
        joint_positions = [T[:3, 3]]
        for i in range(len(thetas)):
            d, a, alpha = self.dh_params[i, 1:]
            T_i = self._dh_matrix(thetas[i], d, a, alpha)
            T = T @ T_i
            joint_positions.append(T[:3, 3])
        return joint_positions[-1], joint_positions

    def inverse(self, target_pos, elbow_config='up', q0=None):
        x, y, z = target_pos
        theta1 = np.arctan2(y, x)
        d1, a1, alpha1 = self.dh_params[0, 1:]
        T_0_1 = self._dh_matrix(theta1, d1, a1, alpha1)
        T_1_0 = self._inv_transform(T_0_1)
        P_in_S0 = np.array([x, y, z, 1])
        P_in_S1 = T_1_0 @ P_in_S0
        r = P_in_S1[0]
        z_prime = P_in_S1[1]
        d = np.sqrt(r ** 2 + z_prime ** 2)
        if d > self.L2 + self.L3 or d < abs(self.L2 - self.L3):
            return None

        alpha = np.arctan2(z_prime, r)
        beta = np.arccos(np.clip((d**2 + self.L2**2 - self.L3**2)/(2*d*self.L2), -1.0, 1.0))
        cos_theta3 = (d**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)

        if elbow_config == 'up':
            theta2 = alpha + beta
            theta3 = -np.arccos(np.clip(cos_theta3, -1.0, 1.0))
        else:
            theta2 = alpha - beta
            theta3 = np.arccos(np.clip(cos_theta3, -1.0, 1.0))

        return [theta1, theta2, theta3]

class RTBSolver(KinematicsSolver):
    """
    Solver que emplea Robotics Toolbox para cálculos de CD/CI usando modelos DH
    y funciones avanzadas como ikine_LM.
    """
    def __init__(self, lengths):
        super().__init__(lengths)
        if not RTB_INSTALLED:
            raise ImportError("Se requiere 'roboticstoolbox-python'.")
        self.robot = DHRobot([
            RevoluteDH(d=self.L1, a=0, alpha=np.pi/2),
            RevoluteDH(d=0, a=self.L2, alpha=0),
            RevoluteDH(d=0, a=self.L3, alpha=0)],
            name="RobotRRR")

    def forward(self, thetas):
        matrix_solver = MatrixDHSolver(self.lengths)
        return matrix_solver.forward(thetas)

    def inverse(self, target_pos, elbow_config='up', q0=None):
        T_target = SE3(target_pos)
        mask = np.array([1, 1, 1, 0, 0, 0])
        q0 = q0 if q0 is not None else np.zeros(self.robot.n)
        sol = self.robot.ikine_LM(T_target, q0=q0, mask=mask)
        return sol.q if sol.success else None

# ------------------------------------------------------------------------------
# 4. MAIN ROBOT CLASS
# ------------------------------------------------------------------------------
class RobotArm:
    """
    Contenedor principal del brazo robótico. Permite seleccionar el enfoque de
    resolución de cinemática e invocar CD/CI según la estrategia definida.
    """
    def __init__(self, lengths, solver_strategy='geometric'):
        self.lengths = lengths
        solver_map = {
            'geometric': GeometricSolver,
            'matrix': MatrixDHSolver,
            'rtb': RTBSolver
        }
        if solver_strategy not in solver_map:
            raise ValueError(f"Solver no reconocido: {solver_strategy}")
        self.solver = solver_map[solver_strategy](lengths)

    def forward_kinematics(self, thetas):
        """
        Ejecuta cinemática directa utilizando la estrategia seleccionada.
        """
        return self.solver.forward(thetas)

    def inverse_kinematics(self, target_pos, elbow_config='up', q0=None):
        """
        Ejecuta cinemática inversa utilizando la estrategia seleccionada.
        """
        return self.solver.inverse(target_pos, elbow_config, q0)
    
# ------------------------------------------------------------------------------
# 5. VISUALIZATION FUNCTION
# ------------------------------------------------------------------------------
def plot_arm(ax, joint_positions, lengths, angles_deg=None, target=None, title=''):
    """
    Renderiza una visualización 3D del brazo robótico en matplotlib.
    Parámetros:
        ax (Axes3D): eje 3D sobre el que se dibuja.
        joint_positions (list): lista de posiciones [P0, P1, P2, P3].
        lengths (list): longitudes [L1, L2, L3] del robot.
        angles_deg (list): ángulos articulares en grados, opcional.
        target (list): coordenada objetivo, opcional.
        title (str): título del gráfico.
    """
    points = np.array(joint_positions)
    ax.plot(points[:, 0], points[:, 1], points[:, 2], '-', color='purple', lw=5, label='Eslabones', zorder=1)
    ax.scatter(points[:-1, 0], points[:-1, 1], points[:-1, 2], c='blue', s=120, label='Articulaciones', zorder=2)

    if target is not None:
        ax.scatter(target[0], target[1], target[2], c='gold', s=450, marker='*', label='Objetivo', zorder=3, edgecolor='black')
    else:
        final_joint = points[-1]
        ax.scatter(final_joint[0], final_joint[1], final_joint[2], s=400, facecolors='none', edgecolors='darkviolet', lw=2, zorder=4, label='Efector Final')

    legend1 = ax.legend(loc='upper right', labelspacing=1.8, fontsize='medium')

    info_labels = ["Parámetros del Robot:",
                   f" L₁ = {lengths[0]}", f" L₂ = {lengths[1]}", f" L₃ = {lengths[2]}"]

    if angles_deg is not None:
        info_labels += ["\\nÁngulos de la Pose:",
                        f" θ₁ = {angles_deg[0]:.1f}°",
                        f" θ₂ = {angles_deg[1]:.1f}°",
                        f" θ₃ = {angles_deg[2]:.1f}°"]

    info_handles = [mpatches.Patch(color='none') for _ in info_labels]
    legend2 = ax.legend(handles=info_handles, labels=info_labels, loc='upper left', labelspacing=1.0, fontsize='medium', title="Información de la Simulación", title_fontsize='large')
    ax.add_artist(legend1)
    # Etiquetas de texto para los ESLABONES
    link_labels = ["L1", "L2", "L3"]
    for i in range(len(link_labels)):
        mid_point = (points[i] + points[i+1]) / 2
        ax.text(mid_point[0], mid_point[1], mid_point[2] + 1.5, link_labels[i], 
                color='black', fontweight='bold', fontsize=9, ha='center',
                bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=1))
                
    # Etiquetas de texto para las ARTICULACIONES
    if angles_deg is not None:
        joint_labels = ["θ₁", "θ₂", "θ₃"]
        # Puntos correspondientes a las articulaciones donde se aplican los ángulos (P0, P1, P2)
        angle_points = points[:3]
        for i, label in enumerate(joint_labels):
            p = angle_points[i]
            # Se añade un pequeño desfase en X y se alinea el texto a la izquierda del punto.
            ax.text(p[0] + 0.5, p[1], p[2], label,
                    color='darkred', fontweight='bold', fontsize=10, ha='left', va='center',
                    bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=1)) 

    ax.set_xlabel('Eje X (m)', fontweight='bold')
    ax.set_ylabel('Eje Y (m)', fontweight='bold')
    ax.set_zlabel('Eje Z (m)', fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')

    max_range = sum(lengths)
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([0, max_range])
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass
    ax.view_init(elev=25, azim=-45)
    ax.grid(True)

# ------------------------------------------------------------------------------
# 6. TEST AND DEMONSTRATION FUNCTIONS
# ------------------------------------------------------------------------------
def run_demonstration(strategy, lengths, angles_deg):
    """
    Ejecuta y compara CD/CI para un conjunto de ángulos usando una estrategia específica.
    Parámetros:
        strategy (str): nombre del solver ('matrix', 'geometric', 'rtb').
        lengths (list): longitudes de eslabones.
        angles_deg (list): ángulos articulares en grados.
    """
    print(f"\n{'='*50}\nSOLVER: {strategy.upper()}\n{'='*50}")

    try:
        robot = RobotArm(lengths=lengths, solver_strategy=strategy)
        angles_rad = np.radians(angles_deg)

        final_pos, joints_fk = robot.forward_kinematics(angles_rad)
        print(f"PCD para {angles_deg}° -> Posición: {np.round(final_pos, 3)}")

        target_angles = robot.inverse_kinematics(final_pos, elbow_config='up', q0=angles_rad)
        if target_angles is None:
            print("El cálculo de la cinemática inversa no encontró una solución.")
            return

        target_angles_deg = [np.rad2deg(angle) for angle in target_angles]
        print(f"PCI -> Ángulos calculados: {np.round(target_angles_deg, 3)}°")

        if np.allclose(angles_rad, target_angles):
            print("VERIFICACIÓN SUPERADA: Los ángulos de la IK coinciden con los originales.")
        else:
            print("AVISO: Los ángulos de la IK no coinciden exactamente (solución alternativa válida).")

        final_pos_ik, joints_ik = robot.forward_kinematics(target_angles)
        print(f"VERIFICACIÓN: Posición alcanzada con ángulos de IK: {np.round(final_pos_ik, 3)}")

        fig, axs = plt.subplots(1, 2, figsize=(16, 8), subplot_kw={'projection': '3d'})
        fig.suptitle(f'Análisis Comparativo - Enfoque {strategy.capitalize()}', fontsize=16)

        plot_arm(axs[0], joints_fk, lengths, angles_deg=angles_deg, title='Cinemática Directa')
        plot_arm(axs[1], joints_ik, lengths, angles_deg=target_angles_deg, target=final_pos, title='Cinemática Inversa')

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show()

    except Exception as e:
        print(f"[Error] No se pudo ejecutar el solver '{strategy}': {e}")
# ------------------------------------------------------------------------------
# 7. MAIN EXECUTION
# ------------------------------------------------------------------------------
if __name__ == "__main__":
    robot_lengths_main = [10, 12, 8]
    angles_deg_main = [40, 60, -50]
    for strategy in ['matrix', 'geometric', 'rtb']:
        run_demonstration(strategy, robot_lengths_main, angles_deg_main)
