import cv2
import mediapipe as mp
import math
import serial
import time

# ==========================================
# 1. USB SERIAL SETTINGS
# ==========================================
COM_PORT = 'COM3'  # Update to your ESP32's COM Port
BAUD_RATE = 115200

try:
    esp_serial = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"Hardware Connected: ESP32 found on {COM_PORT}")
    time.sleep(2) 
except Exception as e:
    print(f"Error: Could not open {COM_PORT}. Is Arduino Serial Monitor closed? \nDetails: {e}")
    exit()

# ==========================================
# 2. MEDIAPIPE EAR CALCULATION
# ==========================================
def euclidean_distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def get_eye_openness(landmarks, eye_indices):
    vertical_1 = euclidean_distance(landmarks[eye_indices[1]], landmarks[eye_indices[5]])
    vertical_2 = euclidean_distance(landmarks[eye_indices[2]], landmarks[eye_indices[4]])
    horizontal = euclidean_distance(landmarks[eye_indices[0]], landmarks[eye_indices[3]])
    
    # SAFETY CATCH: Prevent ZeroDivisionError when looking away
    if horizontal == 0.0:
        return 0.0
        
    return (vertical_1 + vertical_2) / (2.0 * horizontal)

RIGHT_EYE = [33, 160, 158, 133, 153, 144]
LEFT_EYE = [362, 385, 387, 263, 373, 380]

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1)
cap = cv2.VideoCapture(0)

# ==========================================
# 3. TIMING & SENSITIVITY SETTINGS
# ==========================================
blink_threshold = 0.20        # Tune this based on the "Live EAR"
frames_to_confirm_blink = 3   # Short blink (Hover -> Sends '0')
long = 12                     # Long blink (Select -> Sends '1')
cooldown_duration = 1.5       # 1.5s hardware safety delay

frame_counter = 0
last_action_time = 0
last_action_text = "None"



# ==========================================
# 4. MAIN PROCESSING LOOP
# ==========================================
while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_frame)

    time_since_action = time.time() - last_action_time
    in_cooldown = time_since_action < cooldown_duration
    avg_openness = 0.0

    if results.multi_face_landmarks:
        face_landmarks = results.multi_face_landmarks[0]
        h, w, _ = frame.shape
        landmarks = [(int(p.x * w), int(p.y * h)) for p in face_landmarks.landmark]

        left_openness = get_eye_openness(landmarks, LEFT_EYE)
        right_openness = get_eye_openness(landmarks, RIGHT_EYE)
        avg_openness = (left_openness + right_openness) / 2.0

        if not in_cooldown:
            if avg_openness < blink_threshold:
                frame_counter += 1
            else:
                if frame_counter >= frames_to_confirm_blink:
                    if frame_counter >= long:
                        esp_serial.write(b"1")
                        print("TRIGGER: LONG BLINK (Select)")
                        last_action_text = "LONG BLINK (Select)"
                    else:
                        esp_serial.write(b"0")
                        print("TRIGGER: SHORT BLINK (Hover)")
                        last_action_text = "SHORT BLINK (Hover)"
                        
                    last_action_time = time.time()
                
                frame_counter = 0

    # ==========================================
    # 5. LIVE CAMERA HUD
    # ==========================================
    cv2.putText(frame, f"Live EAR: {avg_openness:.3f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(frame, f"Frames Closed: {frame_counter}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    if in_cooldown:
        time_left = cooldown_duration - time_since_action
        cv2.putText(frame, f"STATUS: COOLDOWN ({time_left:.1f}s)", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    else:
        cv2.putText(frame, "STATUS: READY", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.putText(frame, f"Last Action: {last_action_text}", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
    
    cv2.imshow("USB Blink Control", frame)
    
    if cv2.waitKey(1) & 0xFF == 27:  
        break

cap.release()
esp_serial.close()
cv2.destroyAllWindows()