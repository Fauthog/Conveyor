import cv2
from mv_shrimp import find_shrimp_features
from time import perf_counter

import random

# Main function
def main():
    # # Capture video from camera
    # cap = cv2.VideoCapture(0)
    # Read the image as color
    frame = cv2.imread('img_1713563943.5467362.jpg')
    
    # Get the width and height of the frame
    #ret, frame = cap.read()
    height, width = frame.shape[:2]
    
    # Define ROI: (x1, y1, x2, y2)
    nose_roi = (0, int(0.3 * height), int(0.5 * width), int(0.7 * height))

    # Define ROI: (x1, y1, x2, y2)
    ear_roi = (0, int(0.3 * height), int(0.5 * width), int(0.7 * height))
    #ear_roi = (100, 150, 280, 280)


    # Create an instance of the shrimp
    shrimp = find_shrimp_features()

    # Set the ear template with required parameters
    # shrimp.set_ear_template(
    #     './Data/dataset_edited_20240506/template1s.jpg', 
    #     low_scale_factor=0.75, 
    #     high_scale_factor=1.25, 
    #     scale_step=10,
    #     scale_divider=3
    # )

    shrimp.generate_scaled_and_rotated_templates(
        'template1s.jpg', 
        low_scale_factor=0.75, 
        high_scale_factor=1.25, 
        scale_step=10,
        scale_divider=3,
        rotation_range=10,
        rotation_steps=2
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
        frame = cv2.imread('img_1713563943.5467362.jpg')
        h, w = frame.shape[:2]
        center = (w / 2, h / 2)

        # # Slightly rotate the image
        angle = random.uniform(-10, 10)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        frame = cv2.warpAffine(frame, rotation_matrix, (w, h), flags=cv2.INTER_CUBIC)

        found, topleft, lowerright = shrimp.find_ear_mt(frame, ear_roi, match_threshold=0.70, scale_divider=3)
        #found, topleft, lowerright = shrimp.find_ear(frame, ear_roi, match_threshold=0.70)
        
        if found:
            #print(f"Ear found! Top-left corner: {topleft}, Bottom-right corner: {lowerright}")
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
        cv2.rectangle(frame, (ear_roi[0], ear_roi[1]), (ear_roi[2], ear_roi[3]), (255, 0, 0), 2)
        
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