import cv2
import pyautogui
import mediapipe as mp # type: ignore
import serial
import time
import json

ser = serial.Serial('/dev/cu.usbserial-2120', 9600, timeout=1)
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
            
            #--------THUMB-----------# 
            thumb_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y
            thumb_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x

            thumb_mcp_y = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y
            thumb_mcp_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].x
            
            #--------INDEX-----------#
            index_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y
            index_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x

            index_mcp_y = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y
            index_mcp_x = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x
            
            #--------MIDDLE----------#
            middle_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y
            middle_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x
            
            middle_mcp_y = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y
            middle_mcp_x = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x
            
            #--------RING------------#
            ring_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y
            ring_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].x
            
            ring_mcp_y = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y
            ring_mcp_x = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].x
            
            #--------PINKY-----------#
            pinky_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y
            pinky_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].x
            
            pinky_mcp_y = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y
            pinky_mcp_x = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x
            
            
            #print(f'Length Index finger:{round(index_finger_mcp_y, 1) - round(index_finger_tip_y, 2)}')

            # if thumb_x > pinky_x:
            #     hand_gesture = 'PALM'
            # elif thumb_x < pinky_x:
            #     hand_gesture = 'BACK'
            thumbDistance = round(thumb_mcp_y, 2) - round(thumb_tip_y, 2)
            indexDistance = round(index_mcp_y, 2) - round(index_tip_y, 2)
            middleDistance = round(middle_mcp_y, 2) - round(middle_tip_y, 2)
            ringDistance = round(ring_mcp_y, 2) - round(ring_tip_y, 2)
            pinkyDistance = round(pinky_mcp_y, 2) - round(pinky_tip_y, 2)

            hand_values = {
                "thumb": thumbDistance,
                "index": indexDistance,
                "middle": middleDistance,
                "ring": ringDistance,
                "pinky": pinkyDistance,
            }
            json_string = json.dumps(hand_values)
            ser.write((json_string + "\n").encode())
            time.sleep(0.02)

    cv2.imshow('Hand Gesture', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ser.close()
cap.release()
cv2.destroyAllWindows()




