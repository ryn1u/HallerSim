import cv2
import numpy as np
import matplotlib.pyplot as plt


def get_pixel_to_distance_map(depth_image, distance_measures):
    pixel_to_distance_map = list()
    shape = np.shape(depth_image)
    for coords, distance in distance_measures.items():
        if distance == np.inf:
            continue

        y = int(np.round((1 - coords[0]) * shape[0]))
        x = int(np.round(coords[1] * shape[1]))

        # debug_image = cv2.circle(depth_image, (x, y), 5, (255, 255, 255))
        # cv2.putText(debug_image, str(int(distance)), (x + 10, y + 10), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2, cv2.LINE_AA)
        pixel_to_distance_map.append((depth_image[y, x], distance))
    return pixel_to_distance_map


def approx_distance(x):
    a = 28125.11707
    b = -2.03966
    return a * np.power(x, b)


if __name__ == '__main__':
    import pickle
    with open("depth_records.pkl", 'rb') as file:
        depth_data = pickle.load(file)

    x_data = [t[0] for t in depth_data]
    y_data = [t[1] for t in depth_data]

    x2 = np.arange(0, 150, 1)
    y2 = approx_distance(x2)

    plt.scatter(x_data, y_data)
    plt.plot(x2, y2, 'r')
    plt.show()