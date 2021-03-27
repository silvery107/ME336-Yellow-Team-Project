import numpy as np
import random
from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense


def augment(xyzs):
    axyz = np.ones((len(xyzs), 4))
    axyz[:, :3] = xyzs
    return axyz


def estimate(xyzs):
    axyz = augment(xyzs[:3])
    return np.linalg.svd(axyz)[-1][-1, :]


def is_inlier(coeffs, xyz, threshold):
    return np.abs(coeffs.dot(augment([xyz]).T)) < threshold


def run_ransac(data, estimate, is_inlier, sample_size, goal_inliers, max_iterations, stop_at_goal=True,
               random_seed=None):
    best_ic = 0
    best_model = None
    random.seed(random_seed)
    # random.sample cannot deal with "data" being a numpy array
    data = list(data)
    for i in range(max_iterations):
        s = random.sample(data, int(sample_size))
        m = estimate(s)
        ic = 0
        for j in range(len(data)):
            if is_inlier(m, data[j]):
                ic += 1

        print(s)
        print('estimate:', m, )
        print('# inliers:', ic)

        if ic > best_ic:
            best_ic = ic
            best_model = m
            if ic > goal_inliers and stop_at_goal:
                break
    print('took iterations:', i + 1, 'best model:', best_model, 'explains:', best_ic)
    return best_model, best_ic


if __name__ == '__main__':
    import matplotlib
    import matplotlib.pyplot as plt
    from mpl_toolkits import mplot3d

    fig = plt.figure()
    ax = mplot3d.Axes3D(fig)


    def plot_plane(a, b, c, d):
        xx, yy = np.mgrid[:10, :10]
        return xx, yy, (-d - a * xx - b * yy) / c


    n = 4
    max_iterations = 100
    goal_inliers = n * 0.3

    # Todo: test data
    xyzs = [[-0.868, -0.163, 1.251], [0.582, 0.016, 1.256], [-0.794, 0.152, 1.19], [0.594, 0.138, 1.198]]
    xyzs = np.array(xyzs)

    ax.scatter3D(xyzs.T[0], xyzs.T[1], xyzs.T[2])

    # RANSAC
    m, b = run_ransac(xyzs, estimate, lambda x, y: is_inlier(x, y, 0.005), 3, goal_inliers, max_iterations)
    a, b, c, d = m
    xx, yy, zz = plot_plane(a, b, c, d)
    ax.plot_surface(xx, yy, zz, color=(0, 1, 0, 0.5))

    plt.show()
    camera = Realsense('./configs/basic_config/camera_rs_d435.yaml')

    frame = camera.get_frame()
    color = frame.color_image[0]
    depth_img = frame.depth_image[0]
    width = 1280
    hight = 720
    fx = 640.983
    fy = 640.983
    cx = 641.114
    cy = 368.461
    pc = []
    for i in range(0, 720, 4):
        temp_pc = []
        for j in range(0, 1280, 4):
            z = depth_img[i][j]
            x = (j - cx) * z / fx
            y = (i - cy) * z / fy
            if z == 0:
                temp_pc.append([np.nan, np.nan, np.nan])
            else:
                temp_pc.append([x, y, z])
        pc.append(temp_pc)
    pc = np.array(pc).reshape((-1, 3))
    # todo: filter plane
    print('model para', -1 * m)
    # plane_model = [-0.01060143, 0.17056028, 0.61998797, -0.76577524]
    plane_model = -1 * m
    new_pc = []
    for i in range(len(pc)):
        if pc[i][0] * plane_model[0] + pc[i][1] * plane_model[1] + pc[i][2] * plane_model[2] + plane_model[3] < 0:
            new_pc.append(pc[i])

    new_pc = np.array(new_pc)
    show_flag = True
    if show_flag:
        fig = plt.figure("pcs")
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(new_pc[:, 0], new_pc[:, 1], new_pc[:, 2], s=1, )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_zlim(0.5, 2)
        ax.view_init(elev=-90, azim=-90)
        plt.show()
