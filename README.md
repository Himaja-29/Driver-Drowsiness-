This is a driver drowsiness detection system using a Raspberry Pi and OpenCV. It uses Haar cascades to detect faces and eyes in real-time from a webcam feed. The Eye Aspect Ratio (EAR) is calculated to determine whether the driver is sleeping, drowsy, or active.

Hardware: Raspberry Pi, buzzer, and LED to alert the driver.

Logic:

If both eyes’ EAR < threshold → driver is sleeping → buzzer and LED ON.

If one eye’s EAR < threshold → driver is drowsy → buzzer and LED ON.

Otherwise → driver is active → buzzer and LED OFF.

Output: Status is displayed on the video frame with color-coded alerts (red for sleep/drowsy, green for active).

Essentially, it monitors driver alertness in real-time and provides visual and auditory warnings to prevent accidents.
