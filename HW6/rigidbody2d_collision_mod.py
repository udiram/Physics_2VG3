from panda3d.core import Mat3, NodePath, Vec3


EPSILON = 1.0e-8


def safe_normalized(vec: Vec3, default: Vec3 | None = None) -> Vec3:
    length = vec.length()
    if length <= EPSILON:
        if default is None:
            return Vec3(1.0, 0.0, 0.0)
        return Vec3(default)
    return vec / length


def vec2_add(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    return (a[0] + b[0], a[1] + b[1])


def vec2_sub(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    return (a[0] - b[0], a[1] - b[1])


def vec2_dot(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


def vec2_cross(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[1] - a[1] * b[0]


def vec2_mul(a: tuple[float, float], s: float) -> tuple[float, float]:
    return (a[0] * s, a[1] * s)


def vec2_len(a: tuple[float, float]) -> float:
    return (a[0] * a[0] + a[1] * a[1]) ** 0.5


def vec2_norm(a: tuple[float, float]) -> tuple[float, float]:
    length = vec2_len(a)
    if length <= EPSILON:
        return (1.0, 0.0)
    return (a[0] / length, a[1] / length)


def vec2_avg(points: list[tuple[float, float]]) -> tuple[float, float]:
    if not points:
        return (0.0, 0.0)
    sx = 0.0
    sz = 0.0
    for px, pz in points:
        sx += px
        sz += pz
    inv_n = 1.0 / len(points)
    return (sx * inv_n, sz * inv_n)


def segment_intersection(
    p1: tuple[float, float],
    p2: tuple[float, float],
    q1: tuple[float, float],
    q2: tuple[float, float],
    tol: float = 1.0e-8,
):
    r = vec2_sub(p2, p1)
    s = vec2_sub(q2, q1)
    denom = vec2_cross(r, s)
    qp = vec2_sub(q1, p1)

    if abs(denom) < tol:
        return None

    t = vec2_cross(qp, s) / denom
    u = vec2_cross(qp, r) / denom
    if -tol <= t <= 1.0 + tol and -tol <= u <= 1.0 + tol:
        return vec2_add(p1, vec2_mul(r, t))
    return None


def point_in_convex_polygon(pt: tuple[float, float], poly: list[tuple[float, float]], tol: float = 1.0e-8) -> bool:
    has_pos = False
    has_neg = False
    count = len(poly)
    for i in range(count):
        a = poly[i]
        b = poly[(i + 1) % count]
        edge = vec2_sub(b, a)
        rel = vec2_sub(pt, a)
        cross_val = vec2_cross(edge, rel)
        if cross_val > tol:
            has_pos = True
        elif cross_val < -tol:
            has_neg = True
        if has_pos and has_neg:
            return False
    return True


def dedupe_points(points: list[tuple[float, float]], tol: float = 1.0e-6) -> list[tuple[float, float]]:
    out = []
    tol2 = tol * tol
    for point in points:
        duplicate = False
        for existing in out:
            dx = point[0] - existing[0]
            dz = point[1] - existing[1]
            if dx * dx + dz * dz <= tol2:
                duplicate = True
                break
        if not duplicate:
            out.append(point)
    return out


def reduce_contact_points(
    points: list[tuple[float, float]],
    normal: tuple[float, float],
    verts_i: list[tuple[float, float]],
    verts_j: list[tuple[float, float]],
    penetration: float,
) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points

    nx, nz = vec2_norm(normal)
    tangent = (-nz, nx)

    max_i = max(vec2_dot(v, (nx, nz)) for v in verts_i)
    min_j = min(vec2_dot(v, (nx, nz)) for v in verts_j)
    contact_plane = 0.5 * (max_i + min_j)

    plane_tol = max(1.0e-4, 2.0 * penetration + 1.0e-6)
    near_plane = [p for p in points if abs(vec2_dot(p, (nx, nz)) - contact_plane) <= plane_tol]
    candidates = near_plane if near_plane else points

    if len(candidates) <= 2:
        return candidates

    min_p = min(candidates, key=lambda p: vec2_dot(p, tangent))
    max_p = max(candidates, key=lambda p: vec2_dot(p, tangent))
    reduced = dedupe_points([min_p, max_p], tol=1.0e-7)
    return reduced[:2]


def square_vertices_xz(x: Vec3, radius: float, angle: float) -> list[tuple[float, float]]:
    square = [
        Vec3(-1.0, 0.0, -1.0),
        Vec3(-1.0, 0.0, 1.0),
        Vec3(1.0, 0.0, 1.0),
        Vec3(1.0, 0.0, -1.0),
    ]
    rotation = Mat3().rotateMatNormaxis(angle * 180.0 / 3.141592653589793238462, Vec3(0.0, 1.0, 0.0))
    vertices = []
    for vertex in square:
        world = x + rotation.xform(vertex * radius)
        vertices.append((world.x, world.z))
    return vertices


def sat_axes(verts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    axes = []
    for i in range(len(verts)):
        a = verts[i]
        b = verts[(i + 1) % len(verts)]
        edge = vec2_sub(b, a)
        axis = vec2_norm((-edge[1], edge[0]))
        axes.append(axis)
    return axes


def project_poly(verts: list[tuple[float, float]], axis: tuple[float, float]) -> tuple[float, float]:
    values = [vec2_dot(v, axis) for v in verts]
    return (min(values), max(values))


def collideSquareSquare(scene, x1: Vec3, x2: Vec3, radius1: float, radius2: float, angle1: float, angle2: float):
    verts_i = square_vertices_xz(x1, radius1, angle1)
    verts_j = square_vertices_xz(x2, radius2, angle2)

    axes = sat_axes(verts_i) + sat_axes(verts_j)
    min_overlap = float("inf")
    best_axis = (1.0, 0.0)

    for axis in axes:
        i_min, i_max = project_poly(verts_i, axis)
        j_min, j_max = project_poly(verts_j, axis)
        overlap = min(i_max, j_max) - max(i_min, j_min)
        if overlap < 0.0:
            return []
        if overlap < min_overlap:
            min_overlap = overlap
            best_axis = axis

    center_i = (x1.x, x1.z)
    center_j = (x2.x, x2.z)
    center_delta = vec2_sub(center_j, center_i)
    if vec2_dot(center_delta, best_axis) < 0.0:
        best_axis = (-best_axis[0], -best_axis[1])

    contacts_xz = []

    for vertex in verts_i:
        if point_in_convex_polygon(vertex, verts_j):
            contacts_xz.append(vertex)
    for vertex in verts_j:
        if point_in_convex_polygon(vertex, verts_i):
            contacts_xz.append(vertex)

    for i in range(4):
        ai = verts_i[i]
        bi = verts_i[(i + 1) % 4]
        for j in range(4):
            aj = verts_j[j]
            bj = verts_j[(j + 1) % 4]
            hit = segment_intersection(ai, bi, aj, bj)
            if hit is not None:
                contacts_xz.append(hit)

    contacts_xz = dedupe_points(contacts_xz, tol=1.0e-6)
    if not contacts_xz:
        contacts_xz = [vec2_avg([center_i, center_j])]
    else:
        contacts_xz = reduce_contact_points(contacts_xz, best_axis, verts_i, verts_j, min_overlap)

    normal = Vec3(best_axis[0], 0.0, best_axis[1])
    y_contact = 0.5 * (x1.y + x2.y)
    contacts = []
    for cx, cz in contacts_xz:
        xclose = Vec3(cx, y_contact, cz)
        markerAdd(scene, xclose, "collision")
        contacts.append((float(min_overlap), xclose, normal))
    return contacts


def collideShapes(scene, pi, pj):
    return collideSquareSquare(scene, pi.getPos(), pj.getPos(), pi.radius, pj.radius, pi.theta, pj.theta)


class Marker(NodePath):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.iFrame = 0


def markerInit(scene, modelfile):
    try:
        scene.markermodel = loader.loadModel(modelfile)
    except OSError:
        scene.markermodel = loader.loadModel("models/misc/sphere")
    scene.markermodel.setDepthTest(False)
    scene.markermodel.setScale(0.05)
    scene.markers = []


def markerClean(scene):
    if hasattr(scene, "markers"):
        old_markers = scene.markers
        scene.markers = []
        for marker in old_markers:
            if marker.iFrame < scene.iFrame - 1:
                marker.removeNode()
            else:
                scene.markers.append(marker)


def markerAdd(scene, x: Vec3, marker_type: str):
    if not getattr(scene, "showMarkers", True):
        return
    if hasattr(scene, "markers"):
        marker = Marker(marker_type)
        if marker_type == "corner":
            marker.setColor(Vec3(1.0, 1.0, 1.0))
            marker.setScale(0.04)
        elif marker_type == "closest":
            marker.setColor(Vec3(1.0, 0.0, 0.0))
            marker.setScale(0.04)
        else:
            marker.setColor(Vec3(0.0, 0.9, 0.2))
            marker.setScale(0.04)
        marker.setPos(Vec3(x.x, x.y, x.z))
        marker.iFrame = scene.iFrame
        marker.reparentTo(render)
        scene.markermodel.instanceTo(marker)
        scene.markers.append(marker)
