import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
os.environ["MESA_LOADER_DISABLE_EXTENSIONS"] = "true"
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
import mediapipe as mp
import qi
import sys
import math


class CameraPoseNaoNode(Node):
    def __init__(self):
        super().__init__('camera_pose_nao_node')

        # Parametri connessione NAO
        self.declare_parameter('ip', '192.168.0.102')
        self.declare_parameter('port', 9559)
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        # Connessione sessione NAOqi
        self.session = self.set_connection(ip, port)
        self.video_service = self.session.service("ALVideoDevice")

        # Bridge ROS <-> OpenCV
        self.bridge = CvBridge()
        
        resolution = 1   # kQVGA -> più leggero
        colorSpace = 11  # kRGBColorSpace
        fps = 30
        
        self.topCamera = self.video_service.subscribeCamera(
            "top7Camera",
            0,               # 0 = top camera, 1 = bottom camera
            resolution,
            colorSpace,
            fps
        )

        self.botCamera = self.video_service.subscribeCamera(
            "bot7Camera",
            1,               # 0 = top camera, 1 = bottom camera
            resolution,
            colorSpace,
            fps
        )
        
        print(self.topCamera, self.botCamera)
        self.sub_bottom = self.create_timer(1.0 / fps, self.bottom_callback)
        self.sub_front = self.create_timer(1.0 / fps, self.front_callback)

        self.publisher = self.create_publisher(Float32MultiArray, "/nao_pose_info", 10)
        # Mediapipe Pose
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(
            static_image_mode=False,            # False per video/live
            model_complexity=2,                 # 0, 1, 2 → maggiore complessità
            smooth_landmarks=True,              
            enable_segmentation=False,          
            refine_face_landmarks=False,        
            min_detection_confidence=0.8,
            min_tracking_confidence=0.8
        )
        
        self.mp_draw = mp.solutions.drawing_utils
        
        self.last_front_frame = None
        self.last_bottom_frame = None

    def set_connection(self, ip, port):
        session = qi.Session()
        try:
            session.connect(f"tcp://{ip}:{port}")
        except RuntimeError:
            self.get_logger().error(f"Can't connect to Naoqi at ip \"{ip}\" on port {port}.")
            sys.exit(1)
        return session

    def process_combined_frame(self, frame_top, frame_bottom, window_name="Combined Camera"):
        if frame_top.shape[1] != frame_bottom.shape[1]:
            # Ridimensiona bottom alla larghezza del top
            frame_bottom = cv2.resize(frame_bottom, (frame_top.shape[1], frame_bottom.shape[0]))

        # Concatenazione verticale
        combined_frame = cv2.vconcat([frame_top, frame_bottom])
        h_top = frame_top.shape[0]

        # Conversione in RGB per Mediapipe
        rgb = cv2.cvtColor(combined_frame, cv2.COLOR_BGR2RGB)
        results = self.holistic.process(rgb)

        # Disegna landmarks se presenti
        if results.pose_landmarks:
            self.mp_draw.draw_landmarks(combined_frame, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS)
            
            if results.left_hand_landmarks:
                self.mp_draw.draw_landmarks(combined_frame, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)

            if results.right_hand_landmarks:
                self.mp_draw.draw_landmarks(combined_frame, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            
            
            left_shoulder = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]

            left_elbow = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_ELBOW]
            right_elbow = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_ELBOW]
            
            left_wrist = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_WRIST]
            right_wrist = results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_WRIST]

            lw_x, lw_y, lw_z = left_wrist.x, left_wrist.y, left_wrist.z
            rw_x, rw_y, rw_z = right_wrist.x, right_wrist.y, right_wrist.z
            
            # Stima orientamento utente
            orientation = self.estimate_user_orientation(results.pose_landmarks.landmark, combined_frame.shape)
            print(f"Orientamento utente: {orientation}")
            # Calcola angoli spalla e gomito
            left_roll_angle = self.calculate_shoulder_roll(results.pose_landmarks.landmark, side="left")
            right_roll_angle = self.calculate_shoulder_roll(results.pose_landmarks.landmark, side="right")

            left_pitch_angle = self.calculate_shoulder_pitch(results.pose_landmarks.landmark, side="left")
            right_pitch_angle = self.calculate_shoulder_pitch(results.pose_landmarks.landmark, side="right")
            
            left_elbow_angle = self.calculate_elbow_roll_angle(results.pose_landmarks.landmark, side="left")
            right_elbow_angle = self.calculate_elbow_roll_angle(results.pose_landmarks.landmark, side="right")

            msg = Float32MultiArray()
            orientation_val = 0.0
            if orientation == "lateralmente":
                orientation_val = 1.0
            elif orientation == "girato di spalle":
                orientation_val = 2.0
            msg.data = [
                orientation_val,
                left_roll_angle,
                left_pitch_angle,
                left_elbow_angle,
                right_roll_angle,
                right_pitch_angle,
                right_elbow_angle
            ]
            
            self.publisher.publish(msg)

            h, w, _ = combined_frame.shape
            # Posizione pixel (aggiustiamo Y perché l'immagine è combinata)
            left_pos = (int(left_shoulder.x * w), int(left_shoulder.y * h))
            right_pos = (int(right_shoulder.x * w), int(right_shoulder.y * h))

            left_elbow_pos = (int(left_elbow.x * w), int(left_elbow.y * h))
            right_elbow_pos = (int(right_elbow.x * w), int(right_elbow.y * h))

            # Scrive i gradi sopra le spalle
            #cv2.putText(combined_frame, f"{left_elbow_angle} deg", 
            #    (left_elbow_pos[0], left_elbow_pos[1] - 10),
            #    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            #cv2.putText(combined_frame, f"{right_elbow_angle} deg", 
            #    (right_elbow_pos[0], right_elbow_pos[1] - 10),
            #    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            #cv2.putText(combined_frame, f"{left_roll_angle}", 
            #    (left_pos[0], left_pos[1] - 10),
            #    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            #cv2.putText(combined_frame, f"{right_roll_angle}", 
            #    (right_pos[0], right_pos[1] - 10),
            #    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            #cv2.putText(combined_frame, f"{left_pitch_angle}", 
            #    (left_pos[0], left_pos[1] - 10),
            #    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            #cv2.putText(combined_frame, f"{right_pitch_angle}", 
            #    (right_pos[0], right_pos[1] - 10),
            #    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Mostra l'immagine combinata
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, combined_frame)
        cv2.resizeWindow(window_name, 960, 720)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()
            sys.exit(0)


    def front_callback(self):
        msg = self.video_service.getImageRemote(self.topCamera)
        if msg is None:
            self.get_logger().error("Failed to get image from front camera")
            return
        width  = msg[0]
        height = msg[1]
        array  = msg[6]
        
        frame = np.frombuffer(array, dtype=np.uint8).reshape((height, width, 3))
        self.last_front_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        self.try_process_combined()

    def bottom_callback(self):
        msg = self.video_service.getImageRemote(self.botCamera)
        if msg is None:
            self.get_logger().error("Failed to get image from front camera")
            return
        width  = msg[0]
        height = msg[1]
        array  = msg[6]
        
        frame = np.frombuffer(array, dtype=np.uint8).reshape((height, width, 3))
        self.last_bottom_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        self.try_process_combined()

    def try_process_combined(self):
        if self.last_front_frame is not None and self.last_bottom_frame is not None:
            try:
                self.process_combined_frame(self.last_front_frame, self.last_bottom_frame)
            except Exception as e:
                self.get_logger().error(f"Mediapipe error: {e}")
                
    def estimate_user_orientation(self, landmarks, image_shape):
        orientamento = "None"
        
        LSH = (landmarks[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER.value].x * image_shape[1],
               landmarks[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER.value].y * image_shape[0])
        RSH = (landmarks[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER.value].x * image_shape[1],
               landmarks[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER.value].y * image_shape[0])
        LHP = (landmarks[mp.solutions.pose.PoseLandmark.LEFT_HIP.value].x * image_shape[1],
               landmarks[mp.solutions.pose.PoseLandmark.LEFT_HIP.value].y * image_shape[0])
        RHP = (landmarks[mp.solutions.pose.PoseLandmark.RIGHT_HIP.value].x * image_shape[1],
               landmarks[mp.solutions.pose.PoseLandmark.RIGHT_HIP.value].y * image_shape[0])
        # funzione distanza Euclidea
        def dist(p1, p2):
            return math.hypot(p1[0]-p2[0], p1[1]-p2[1])
        
        shoulder_width = dist(LSH, RSH)
        
        torso_len = dist(((LSH[0]+RSH[0])/2, (LSH[1]+RSH[1])/2),
                         ((LHP[0]+RHP[0])/2, (LHP[1]+RHP[1])/2))
        if torso_len > 1e-3:
            ratio = abs(shoulder_width / torso_len)
            if ratio > 0.4:
                orientamento = "di fronte"
            else:
                orientamento = "lateralmente"
            # controllo spalle invertite → utente di spalle
            if LSH[0] < RSH[0] and ratio > 0.4:
                orientamento = "girato di spalle"

        return orientamento

    def calculate_shoulder_roll(self, landmarks, side="left"):
        if side == "left":
            shoulder = landmarks[self.mp_holistic.PoseLandmark.LEFT_SHOULDER]
            elbow = landmarks[self.mp_holistic.PoseLandmark.LEFT_ELBOW]
        else:
            shoulder = landmarks[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER]
            elbow = landmarks[self.mp_holistic.PoseLandmark.RIGHT_ELBOW]

        # Vettore spalla → gomito (x, y)
        vx = elbow.x - shoulder.x
        vy = elbow.y - shoulder.y

        # Angolo con la verticale
        mag_v = math.sqrt(vx**2 + vy**2)
        if mag_v < 1e-6:
            return 0.0, 0.0  # caso limite

        # Normalizzazione
        nx = vx / mag_v

        angle_rad = math.asin(nx)

        return float(f"{angle_rad:.2f}")

    def calculate_shoulder_pitch(self, landmarks, side="left"):
        if side == "left":
            shoulder = landmarks[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
            elbow = landmarks[mp.solutions.pose.PoseLandmark.LEFT_ELBOW]
            hip = landmarks[mp.solutions.pose.PoseLandmark.LEFT_HIP]
        else:
            shoulder = landmarks[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]
            elbow = landmarks[mp.solutions.pose.PoseLandmark.RIGHT_ELBOW]
            hip = landmarks[mp.solutions.pose.PoseLandmark.RIGHT_HIP]

        # Lunghezza del busto
        bust_length = math.sqrt((shoulder.x - hip.x)**2 + (shoulder.y - hip.y)**2)

        # Stima lunghezza braccio (fattore empirico)
        arm_length = bust_length * 0.95

        vy = elbow.y - shoulder.y

        # Angolo stimato
        sin_theta = vy / (arm_length + 1e-8)
        theta_rad = math.asin(max(-math.pi/2, min(math.pi/2, sin_theta))) * math.pi

        return float(f"{theta_rad:.2f}")

    
    def calculate_elbow_roll_angle(self, landmarks, side="left"):
        if side == "left":
            shoulder = landmarks[self.mp_holistic.PoseLandmark.LEFT_SHOULDER]
            elbow = landmarks[self.mp_holistic.PoseLandmark.LEFT_ELBOW]
            wrist = landmarks[self.mp_holistic.PoseLandmark.LEFT_WRIST]
        else:
            shoulder = landmarks[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER]
            elbow = landmarks[self.mp_holistic.PoseLandmark.RIGHT_ELBOW]
            wrist = landmarks[self.mp_holistic.PoseLandmark.RIGHT_WRIST]

        # Vettori sul piano XY
        v1x, v1y = shoulder.x - elbow.x, shoulder.y - elbow.y
        v2x, v2y = wrist.x - elbow.x, wrist.y - elbow.y

        # Calcolo prodotto scalare e magnitudini
        dot = v1x*v2x + v1y*v2y
        mag_v1 = math.sqrt(v1x**2 + v1y**2)
        mag_v2 = math.sqrt(v2x**2 + v2y**2)

        if mag_v1 < 1e-6 or mag_v2 < 1e-6:
            return 0.0  # caso limite

        # Angolo tra i due vettori
        cos_theta = dot / (mag_v1 * mag_v2)
        cos_theta = max(min(cos_theta, 1.0), -1.0)  # clamp per sicurezza numerica
        angle_rad = math.acos(cos_theta)

        # Invertiamo la scala
        angle_rad = math.pi - angle_rad

        # Per convenzione, se è il sinistro destro possiamo invertire il segno
        if side == "left":
            angle_rad = -angle_rad

        return float(f"{angle_rad:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseNaoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
