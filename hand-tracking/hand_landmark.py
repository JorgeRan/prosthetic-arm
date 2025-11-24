import cv2
import pyautogui
import mediapipe as mp
import serial
import time

ser = serial.Serial('/dev/cu.usbserial-1110', 9600, timeout=1)
time.sleep(2)

cap = cv2.VideoCapture(0)

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

mp_drawing = mp.solutions.drawing_utils

while True:
    ret, frame = cap.read() 
    if not ret:
        break

    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    results = hands.process(image_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(
                frame, hand_landmarks, mp_hands.HAND_CONNECTIONS
            )

            index_finger_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y
            index_finger_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x

            index_finger_mcp_y = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y
            index_finger_mcp_x = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x
            thumb_y = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y
            thumb_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x


            pinky_y = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y
            pinky_x = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].x
            
            #print(f'Length Index finger:{round(index_finger_mcp_y, 1) - round(index_finger_tip_y, 2)}')

            # if thumb_x > pinky_x:
            #     hand_gesture = 'PALM'
            # elif thumb_x < pinky_x:
            #     hand_gesture = 'BACK'
            distance = round(index_finger_mcp_y, 2) - round(index_finger_tip_y, 2)
           
            print("Sending:", distance)
            ser.write((f"{distance}" + "\n").encode())
            time.sleep(0.05)

    cv2.imshow('Hand Gesture', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ser.close()
cap.release()
cv2.destroyAllWindows()




