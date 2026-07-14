import cv2
import mediapipe as mp
import math
import serial
import threading
import queue
import time

# ==========================================
# 1. USB SERIAL SETTINGS
# ==========================================
COM_PORT = 'COM8'  # Update to your ESP32's COM Port
BAUD_RATE = 115200

esp_serial = None
serial_status = "OK"                 # last write outcome, shown on the HUD
serial_queue = queue.Queue(maxsize=5)

def serial_writer():
    """
    Owns all communication with the ESP32 and runs on its own thread.
    The camera loop only ever does serial_queue.put_nowait(...), which
    returns instantly - so a slow or blocked write can no longer freeze
    the video feed, no matter what the ESP32 is doing.
    """
    global serial_status
    while True:
        payload = serial_queue.get()
        if payload is None:          # shutdown sentinel
            break
        if esp_serial and esp_serial.is_open:
            try:
                esp_serial.write(payload)
                esp_serial.flush()
                serial_status = "OK"
                time.sleep(0.05)  # brief window for ESP to respond
                if esp_serial.in_waiting:
                    response = esp_serial.read(esp_serial.in_waiting).decode(errors="ignore")
                    print(f"[ESP RESPONSE] {response.strip()}")
            except serial.SerialTimeoutException:
                serial_status = "TIMEOUT"
                print("Serial Write Timeout: ESP32 didn't accept the byte in time")
            except Exception as serial_err:
                serial_status = "ERROR"
                print(f"Serial Write Error: {serial_err}")

try:
    # write_timeout is the actual fix: before, it was set as a stray local
    # variable that never reached Serial(), so writes had NO timeout and
    # could block forever - freezing the whole script, camera included -
    # any time the ESP32 was slow to read (mid-delay(), mid-reset, etc).
    esp_serial = serial.Serial(COM_PORT, BAUD_RATE, timeout=1, write_timeout=0.5)
    print(f"Hardware Connected: ESP32 found on {COM_PORT}")
    time.sleep(2)
except Exception as e:
    print(f"Warning: Could not open {COM_PORT}. Running in simulation mode (no hardware). \nDetails: {e}")
    esp_serial = None  # Allow the script to run anyway for testing

writer_thread = threading.Thread(target=serial_writer, daemon=True)
writer_thread.start()

# ==========================================
# 2. MEDIAPIPE EAR CALCULATION
# ==========================================
def euclidean_distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def get_eye_openness(landmarks, eye_indices):
    if len(landmarks) <= max(eye_indices):
        return 0.0
    vertical_1 = euclidean_distance(landmarks[eye_indices[1]], landmarks[eye_indices[5]])
    vertical_2 = euclidean_distance(landmarks[eye_indices[2]], landmarks[eye_indices[4]])
    horizontal = euclidean_distance(landmarks[eye_indices[0]], landmarks[eye_indices[3]])

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
blink_threshold = 0.20
frames_to_confirm_blink = 3
long = 12
cooldown_duration = 1.5

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
                    # Determine payload
                    payload = b"1" if frame_counter >= long else b"0"
                    action_name = "LONG BLINK (Select)" if frame_counter >= long else "SHORT BLINK (Hover)"

                    print(f"TRIGGER: {action_name}")
                    last_action_text = action_name

                    # Hand the byte to the writer thread and move straight on -
                    # the camera loop never waits on the ESP32 or the serial line.
                    if esp_serial and esp_serial.is_open:
                        try:
                            serial_queue.put_nowait(payload)
                        except queue.Full:
                            print("Serial queue full - ESP32 not keeping up, dropping this command")
                    else:
                        print(f"[Simulated] Sent {payload.decode()} to ESP32")

                    last_action_time = time.time()

                frame_counter = 0
    else:
        # If MediaPipe loses the face mid-blink, don't let the counter get stuck
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

    # Small diagnostic readout - lets you SEE if a write ever times out or
    # errors, instead of only finding out because the feed used to freeze.
    serial_color = (0, 255, 0) if serial_status == "OK" else (0, 0, 255)
    cv2.putText(frame, f"Serial: {serial_status}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, serial_color, 2)

    cv2.imshow("USB Blink Control", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
serial_queue.put(None)           # tell the writer thread to stop
writer_thread.join(timeout=1)
if esp_serial and esp_serial.is_open:
    esp_serial.close()
cv2.destroyAllWindows()