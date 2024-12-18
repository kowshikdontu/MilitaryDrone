from basic_func import *
import cv2
import socket
import numpy as np
import time


def capture_camera_frame(client_socket):
    # Receive the size of the frame
    frame_size = int.from_bytes(client_socket.recv(4), 'big')

    # Receive the frame data
    frame_data = b''
    while len(frame_data) < frame_size:
        packet = client_socket.recv(frame_size - len(frame_data))
        if not packet:
            break
        frame_data += packet

    # Decode the JPEG image
    frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
    return frame


def detect_door(image):

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
    # cv2.namedWindow('All Contours (Blue)', cv2.WINDOW_NORMAL)
    # cv2.namedWindow('Valid Quadrilaterals (Green)', cv2.WINDOW_NORMAL)

    # cv2.imshow('All Contours (Blue)', image_with_all_contours)
    # cv2.imshow('Valid Quadrilaterals (Green)', image_with_quads)
    if max_contour is not None and len(max_contour) == 4:
        image_points = np.array([point[0] for point in max_contour], dtype="double")
        return image_points

    return None



def search_frame(controller, detect_door,picam2 ,max_rotation=360):
    cnt =0
    while cnt<5:

        frame = capture_camera_frame(picam2)

        bbox = detect_door(frame)
        time.sleep(2)
        if bbox is not None:
            print("Bounding box found!")
            return bbox

        print(f"No frame found")
        cnt+=1


    print("Search failed: No frame detected after full rotation.")
    return None


def converge_to_center(controller, image_points, picam2, camera_resolution=(640, 480)):

    frame_center_x = camera_resolution[0] // 2
    frame_center_y = camera_resolution[1] // 2

    while True:
        # Calculate the center of the detected quadrilateral
        bbox_center_x = int(np.mean(image_points[:, 0]))
        bbox_center_y = int(np.mean(image_points[:, 1]))

        # Check if the detected frame is sufficiently centered
        if abs(bbox_center_x - frame_center_x) <= 10 and abs(bbox_center_y - frame_center_y) <= 10:
            print("Bounding box centered!")
            return True

        # Calculate adjustment factors
        move_x = (frame_center_x - bbox_center_x) / frame_center_x
        move_y = (frame_center_y - bbox_center_y) / frame_center_y

        print(f"Adjusting drone position: X={move_x}, Y={move_y}")
        controller.send_velocity(move_x, move_y, 0)
        time.sleep(0.1)

        # Capture a new frame and re-detect the quadrilateral
        frame = capture_camera_frame(picam2)
        image_points = detect_door(frame)

        if image_points is None:
            print("Lost bounding box during convergence.")
            return False
def execute_pnp(controller, position, yaw):
    x, y, z = position
    print(f"Executing based on PnP Output: Position: {position}, Yaw: {yaw}")

    # Step 1: Correct Yaw
    print(f"Aligning Yaw: {yaw}°")
    controller.condition_yaw(yaw, relative=True)
    time.sleep(2)  # Allow time for yaw alignment

    # Step 2: Center the Frame
    if abs(x) > 0.05:  # Allow some tolerance
        if x < 0:
            print(f"Moving Left: {abs(x)} meters")
            controller.move_left(abs(x))
        else:
            print(f"Moving Right: {x} meters")
            controller.move_right(x)

    if abs(y) > 0.05:  # Allow some tolerance
        if y < 0:
            print(f"Moving Down: {abs(y)} meters")
            controller.move_down(abs(y))
        else:
            print(f"Moving Up: {y} meters")
            controller.move_up(y)

    # Step 3: Move Forward to Frame
    print(f"Moving Forward: {z} meters")
    controller.move_forward(z)

    print("Execution Complete!")


def execute(controller, picam2):

    window_width = 0.73
    window_height = 0.57

    print("Capturing image from camera...")
    image = capture_camera_frame(picam2)
    image_2d_points = detect_door(image)

    if image_2d_points is not None and len(image_2d_points) == 4:
        print("Frame detected! Processing PnP data...")

        model_points = np.array([
            [0, 0, 0],
            [window_width, 0, 0],
            [window_width, window_height, 0],
            [0, window_height, 0]
        ], dtype="double")

        #
        camera_matrix = np.array([
            [722.90283793, 0, 351.36461658],
            [0, 722.90283793, 241.00635105],
            [0, 0, 1]
        ], dtype="double")
        dist_coeffs = np.array([[-0.54819263, 1.01679325, -0.00645402, -0.0107541, -1.85955179]])

        # Solve PnP to get position and orientation
        success, rotation_vector, translation_vector = cv2.solvePnP(
            model_points, image_2d_points, camera_matrix, dist_coeffs
        )

        if success:
            print("PnP Solution Found! Processing Position and Orientation...")

            # Extract position and orientation
            position = translation_vector.flatten()
            rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

            # Calculate yaw, pitch, and roll
            yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            pitch = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))
            roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

            print(f"Position (meters): {position}")
            print(f"Orientation (Rodrigues): {rotation_vector.flatten()}")
            print(f"Yaw: {np.degrees(yaw):.2f}°, Pitch: {np.degrees(pitch):.2f}°, Roll: {np.degrees(roll):.2f}°")

            # Visual feedback (optional)
            # projected_points, _ = cv2.projectPoints(
            #     model_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs
            # )
            # projected_points = projected_points.astype(int)
            # for i in range(4):
            #     start_point = tuple(projected_points[i][0])
            #     end_point = tuple(projected_points[(i + 1) % 4][0])
            #     cv2.line(image, start_point, end_point, (0, 255, 0), 2)
            # cv2.imshow("Window Detection with PnP", image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            # Execute PnP-based motion
            execute_pnp(controller, position, np.degrees(yaw))

            if position[2] > 2:
                print("Approaching the frame to finalize position...")
                controller.move_forward(position[2] - 1)

                print("Aligning for final pass...")
                controller.move_forward(1.5)

            print("Frame passed successfully!")
        else:
            print("PnP failed to solve position and orientation. Mission aborted.")
    else:
        print("Contour detection failed! Mission failed: lost the frame.")


def mission(controller, detect_door,picam2):

    print("Starting mission...")
    bbox = search_frame(controller, detect_door,picam2)
    if bbox is None:
        print("Mission failed: No frame detected.")
        return

    print("Converging to center...")
    if not converge_to_center(controller, bbox,picam2):
        print("Mission failed: Could not center the frame.")
        return

    print("Executing final maneuver through the frame...")
    execute(controller, picam2)
    print("Mission completed successfully!")

if __name__ == "__main__":
    HOST = '127.0.0.1'
    PORT = 65432

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))

    mission(controller, detect_door, client_socket)
    vehicle.mode("land")

    



