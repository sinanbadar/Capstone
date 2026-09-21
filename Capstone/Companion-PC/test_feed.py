from djitellopy import Tello
import cv2

drone = Tello(host="192.168.10.1")
drone.connect()
print(f"Battery: {drone.get_battery()}%")
drone.streamon()

frame_reader = drone.get_frame_read()
print("Stream started, press Q to quit")

while True:
    frame = frame_reader.frame
    if frame is not None:
        cv2.imshow("Tello Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

drone.streamoff()
cv2.destroyAllWindows()