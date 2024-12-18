import cv2
import numpy as np

image_path = 'black/image_3.jpg'
image = cv2.imread(image_path)
window_width = 0.73
window_height = 0.57
if image is None:
    print("Error: Unable to load the image.")
    exit(1)

hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

lower_black = np.array([0, 0, 0])
upper_black = np.array([200, 290, 78])

black_mask = cv2.inRange(hsv_image, lower_black, upper_black)

blurred_mask = cv2.GaussianBlur(black_mask, (5, 5), 0)

edges = cv2.Canny(blurred_mask, 30, 120)

kernel = np.ones((3, 3), np.uint8)
closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)

contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

max_area = 0
max_contour = None
valid_contours = []
for contour in contours:
    hull = cv2.convexHull(contour)
    epsilon = 0.02 * cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, epsilon, True)
    area = cv2.contourArea(hull)
    if 4 <= len(approx) <= 6 and cv2.contourArea(hull) > 500:
        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h
        if 0.4 < aspect_ratio < 1.2:
            valid_contours.append(approx)
        if area > max_area:
            max_area = area
            max_contour = approx

image_with_all_contours = image.copy()
cv2.drawContours(image_with_all_contours, contours, -1, (255, 0, 0), 2)

image_with_quads = image.copy()
cv2.drawContours(image_with_quads, [max_contour], -1, (0, 255, 0), 2)

cv2.namedWindow('All Contours (Blue)', cv2.WINDOW_NORMAL)
cv2.namedWindow('Valid Quadrilaterals (Green)', cv2.WINDOW_NORMAL)

cv2.imshow('All Contours (Blue)', image_with_all_contours)
cv2.imshow('Valid Quadrilaterals (Green)', image_with_quads)

if max_contour is not None and len(max_contour) == 4:
    model_points = np.array([
        [0, 0, 0],
        [window_width, 0, 0],
        [window_width, window_height, 0],
        [0, window_height, 0]
    ], dtype="double")

    image_points = np.array([point[0] for point in max_contour], dtype="double")

    camera_matrix = np.array([
        [722.90283793, 0, 351.36461658],
        [0, 722.90283793, 241.00635105],
        [0, 0, 1]
    ], dtype="double")

    dist_coeffs = np.array([[-0.54819263, 1.01679325, -0.00645402, -0.0107541,-1.85955179]])

    success, rotation_vector, translation_vector = cv2.solvePnP(
        model_points, image_points, camera_matrix, dist_coeffs
    )

    if success:
        (projected_points, _) = cv2.projectPoints(
            model_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs
        )

        projected_points = projected_points.astype(int)
        for i in range(4):
            start_point = tuple(projected_points[i][0])
            end_point = tuple(projected_points[(i + 1) % 4][0])
            cv2.line(image, start_point, end_point, (0, 255, 0), 2)

        position = translation_vector.flatten()
        orientation = rotation_vector.flatten()
        print("Position:", position)
        print("Orientation:", orientation)
        print("Translation Vector (Position in meters):", translation_vector.flatten())
        print("Rotation Vector (Rodrigues):", rotation_vector.flatten())

        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        print("Rotation Matrix:\n", rotation_matrix)

        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        pitch = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

        print("Yaw:", np.degrees(yaw))
        print("Pitch:", np.degrees(pitch))
        print("Roll:", np.degrees(roll))

        cv2.imshow("Window Detection with PnP", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

else:
    print("No valid contour found.")

    cv2.waitKey(0)
    cv2.destroyAllWindows()
