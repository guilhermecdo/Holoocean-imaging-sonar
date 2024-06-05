import numpy as np
from scipy.spatial.transform import Rotation as R

def load_xyz_file(file_path):
    """
    Lê um arquivo .xyz e retorna os pontos como um array numpy.
    """
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.strip():  # Ignorar linhas vazias
                x, y, z,_,_,_ = map(float, line.split())
                points.append([x, y, (z-0.5)])
    #print(np.array(points))
    return np.array(points)

def generate_frustum_planes(fov_horizontal, fov_vertical, min_distance, max_distance, position, orientation):
    # Convertendo ângulos de campo de visão de graus para radianos
    h_angle = np.radians(fov_horizontal / 2)
    v_angle = np.radians(fov_vertical / 2)
    
    # Criação dos pontos no plano near
    t_near = np.tan(v_angle) * min_distance
    r_near = np.tan(h_angle) * min_distance
    
    # Criação dos pontos no plano far
    t_far = np.tan(v_angle) * max_distance
    r_far = np.tan(h_angle) * max_distance
    
    # Criação dos vértices do frustum no espaço da câmera
    frustum_near = np.array([
        [r_near, t_near, min_distance],
        [-r_near, t_near, min_distance],
        [r_near, -t_near, min_distance],
        [-r_near, -t_near, min_distance]
    ])
    
    frustum_far = np.array([
        [r_far, t_far, max_distance],
        [-r_far, t_far, max_distance],
        [r_far, -t_far, max_distance],
        [-r_far, -t_far, max_distance]
    ])
    
    # Aplicar a orientação e posição da câmera
    rotation_matrix = R.from_euler('xyz', orientation, degrees=True).as_matrix()
    frustum_near = frustum_near @ rotation_matrix.T + position
    frustum_far = frustum_far @ rotation_matrix.T + position

    return frustum_near, frustum_far

def point_in_frustum(point, frustum_near, frustum_far, position, orientation):
    # Convertendo os pontos do frustum para o sistema de coordenadas da câmera
    rotation_matrix = R.from_euler('xyz', orientation, degrees=True).as_matrix()
    point_cam = (point - position) @ rotation_matrix

    # Parâmetros do frustum no sistema de coordenadas da câmera
    min_distance = frustum_near[0, 2]
    max_distance = frustum_far[0, 2]
    tan_h = np.abs(frustum_near[0, 0] / min_distance)
    tan_v = np.abs(frustum_near[0, 1] / min_distance)

    # Verificar se o ponto está dentro do frustum
    z = point_cam[2]
    if z < min_distance or z > max_distance:
        return False
    x = point_cam[0]
    y = point_cam[1]
    if np.abs(x) > z * tan_h or np.abs(y) > z * tan_v:
        return False

    return True

def filter_points(points, fov_horizontal, fov_vertical, min_distance, max_distance, camera_position, camera_orientation):
    frustum_near, frustum_far = generate_frustum_planes(fov_horizontal, fov_vertical, min_distance, max_distance, camera_position, camera_orientation)
    points_in_view = []

    for point in points:
        if point_in_frustum(point, frustum_near, frustum_far, camera_position, camera_orientation):
            points_in_view.append(point)

    return np.array(points_in_view,dtype=np.float128)

# Exemplo de uso:
file_path = 'prisma.asc'  # Atualize com o caminho para o seu arquivo .xyz
points = load_xyz_file(file_path)

camera_position = np.array([2,0,0])
camera_orientation = [0, 0, 180]  # Euler angles (yaw, pitch, roll)
fov_horizontal = 130  # graus
fov_vertical = 20  # graus
min_distance = 0.5  # unidades
max_distance = 10  # unidades

filtered_points = filter_points(points, fov_horizontal, fov_vertical, min_distance, max_distance, camera_position, camera_orientation)
print(filtered_points)