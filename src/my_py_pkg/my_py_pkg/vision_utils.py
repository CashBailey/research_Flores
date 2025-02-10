import numpy as np
import cv2
from numpy.linalg import inv, svd

def unitize(x, y):
    """Normalize 2D vector"""
    magnitude = np.sqrt(x**2 + y**2)
    return x/magnitude, y/magnitude

def vex(A):
    """Convert skew-symmetric matrix to vector"""
    if A.shape != (3, 3):
        raise ValueError("vex: expects 3x3 matrices as input")
    return np.array([A[2,1]-A[1,2],
                     A[0,2]-A[2,0],
                     A[1,0]-A[0,1]]) * 0.5

def compute_homography(ref_pts, curr_pts, K):
    """Compute normalized homography"""
    H, _ = cv2.findHomography(ref_pts, curr_pts, cv2.RANSAC)
    return inv(K) @ H @ K  # Use matrix operator @ instead of dot()

def recover_from_homography(H, K, iter_count, prev_solution):
    """Recover pose from homography matrix"""
    U, S, V_T = svd(H) # Use svd directly from numpy.linalg
    V = V_T.T
    s = S[0] + S[1]
    t1 = np.array([U[:, 2]]).T * s / 2.0
    t2 = -np.array([U[:, 2]]).T * s / 2.0
    W = np.diag([1, 1, (S[0] / S[1])])
    W_inv = W.T
    R1 = U @ W @ V_T
    R2 = U @ W_inv @ V_T
    n1 = np.array([V[:, 2]]).T
    n2 = - np.array([V[:, 2]]).T # Normal vector n2 = -n1

    d = 1 # Since Zref = Zcurrent

    # Ensure rotation matrices are proper rotations (determinant +1) and normals point towards camera.
    if np.linalg.det(R1) < 0:
        R1 = -R1
    if np.linalg.det(R2) < 0:
        R2 = -R2
    if n1[2] < 0:
        n1 = -n1
    if n2[2] < 0:
        n2 = -n2

    possible_solutions = [{'R': R1, 't': t1, 'n': n1, 'solution_tag': 1},
                           {'R': R2, 't': t1, 'n': n2, 'solution_tag': 2},
                           {'R': R1, 't': t2, 'n': n2, 'solution_tag': 3},
                           {'R': R2, 't': t2, 'n': n1, 'solution_tag': 4}]

    # In a real scenario, you would have to implement a more robust solution selection
    # based on reprojection error and/or cheirality checks.
    # Here, for simplicity, we just return the first valid solution.
    # A basic heuristic could be to check if the translation is not NaN.
    # And also check if the recovered rotation is close to a rotation matrix.

    best_solution = None
    min_error = float('inf')

    for sol in possible_solutions:
        R_candidate = sol['R']
        t_candidate = sol['t']
        n_candidate = sol['n']

        if not np.isnan(np.sum(t_candidate)): # Basic NaN check for translation
            if best_solution is None:
                 best_solution = sol # Select the first valid solution as best for now.


    if best_solution is not None:
        R = best_solution['R']
        t = best_solution['t']
        n = best_solution['n']
        solution_tag = best_solution['solution_tag']
    else:
        R = np.eye(3)
        t = np.zeros((3, 1))
        n = np.zeros(3)
        d = 0.0
        solution_tag = 0 # Indicate no valid solution found

    return R, t, n, d, solution_tag


def estimate_homography(ref_img, curr_img, K, counter):
    """Feature-based homography estimation"""
    orb = cv2.ORB_create(600)
    kp1, des1 = orb.detectAndCompute(ref_img, None)
    kp2, des2 = orb.detectAndCompute(curr_img, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = sorted(bf.match(des1, des2), key=lambda x: x.distance)[:50]

    # Convert keypoints to numpy arrays
    ref_pts = np.float32([kp1[m.queryIdx].pt for m in matches])
    curr_pts = np.float32([kp2[m.trainIdx].pt for m in matches])

    if len(ref_pts) < 4 or len(curr_pts) < 4:
        return None

    H, _ = cv2.findHomography(ref_pts, curr_pts, cv2.RANSAC, 5.0)
    return (inv(K) @ H @ K) / H[1,1]  # Modern matrix operator

def rodriguez(R):
    u, _ = cv2.Rodrigues(R)
    return u

def pbvs_controller(R, t, u, lambdav, lambdaw):
    Uv = -lambdav * np.dot(R.T, t)
    Uw = -lambdaw * u
    return Uv, Uw

def normalize_point(point, Kinv):
    norm_point = np.dot(Kinv, np.hstack((point, [1])))
    return norm_point[:2] / norm_point[2]
