import cv2
from mv_shrimp import find_shrimp_features
from time import perf_counter
from picamera2 import Picamera2

# Main function
def main():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format":'XRGB8888', "size":(720,480)}, controls={'FrameRate':120})
    picam2.configure(config)
    picam2.start()
    frame = picam2.capture_array()
    
    # Get the width and height of the frame
    #ret, frame = cap.read()
    height, width = frame.shape[:2]
    
    # Define ROI: (x1, y1, x2, y2)
    nose_roi = (0, int(0.3 * height), int(0.5 * width), int(0.7 * height))

    # Define ROI: (x1, y1, x2, y2)
    ear_roi = (0, int(0.3 * height), int(0.5 * width), int(0.7 * height))


    # Create an instance of the shrimp
    shrimp = find_shrimp_features()

    # Set the ear template with required parameters
    shrimp.set_ear_template(
        '/home/admin/Conveyor/template1.jpg', 
        low_scale_factor=0.9, 
        high_scale_factor=1.1, 
        scale_step=3
    )

    while True:
        #ret, frame = cap.read()
        # if not ret:
        #     break
        
        # record start time
        time_start = perf_counter()

        # Detect nose
        # Parameters
        nose_dark_threshold = 100  # Settable dark value threshold
        nose_area_threshold = 100  # Minimum area of the detected object to be considered valid

        # # Find the nose in the input image
        # frame = cv2.imread('./Data/dataset_from_setup_20240501/img_1713563896.547093.jpg')
        # found, topleft, lowerright = shrimp.find_nose(frame, nose_roi, nose_dark_threshold, nose_area_threshold)
        # if found:
        #     # Draw bounding box in the original frame
        #     cv2.rectangle(frame, topleft, lowerright, (0, 255, 0), 1)
        # else:
        #     print("found nothing")

        # Find the ear in the input image
        frame = picam2.capture_array()
        found, topleft, lowerright = shrimp.find_ear(frame, ear_roi, match_threshold=0.8)
        
        if found:
            # print(f"Ear found! Top-left corner: {topleft}, Bottom-right corner: {lowerright}")
            cv2.rectangle(frame, topleft, lowerright, (0, 255, 0), 1)
        else:
            print("No match found.")



        # record end time
        time_end = perf_counter()
        # calculate the duration
        time_duration = time_end - time_start
        # report the duration
        print(f'Took {time_duration} seconds')

        # Draw ROI rectangle on the frame
        cv2.rectangle(frame, (nose_roi[0], nose_roi[1]), (nose_roi[2], nose_roi[3]), (255, 0, 0), 2)
        
        # Display the frame
        cv2.imshow('Frame', frame)
        
        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the video capture object and close all windows
    #cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()