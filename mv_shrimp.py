import cv2
import numpy as np

class find_shrimp_features:
    def __init__(self):
        self.ear_template = None
        self.ear_scale_factors = None
        self.ear_scaled_templates = []

    def set_ear_template(self, template_image_path, low_scale_factor, high_scale_factor, scale_step):
        # Load the template and convert to grayscale (if not already grayscale)
        self.template = cv2.imread(template_image_path, 0)
        
        # Apply histogram equalization to the template
        self.template = cv2.equalizeHist(self.template)
        
        # Define scale factors to iterate over
        self.scale_factors = np.linspace(low_scale_factor, high_scale_factor, scale_step)
        
        # Pre-calculate scaled templates
        self.scaled_templates = []
        for scale in self.scale_factors:
            self.scaled_templates.append(cv2.resize(self.template, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC))

    def find_ear(self, frame, roi, match_threshold):
        # Extract the ROI
        x1, y1, x2, y2 = roi
        roi_image = frame[y1:y2, x1:x2]
        
        # Convert color image to grayscale for processing
        roi_image = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
        
        # Apply histogram equalization to the image
        roi_image = cv2.equalizeHist(roi_image)
        
        # Initialize variables to store the best match
        best_match_val = -1
        best_match_loc = None
        best_match_size = None

        # Iterate over pre-calculated scaled templates
        for i, scaled_template in enumerate(self.scaled_templates):
            w, h = scaled_template.shape[::-1]
            
            # Perform template matching within the ROI
            result = cv2.matchTemplate(roi_image, scaled_template, cv2.TM_CCOEFF_NORMED) 
            
            # Find the best match location
            _, max_val, _, max_loc = cv2.minMaxLoc(result)
            
            # Update the best match if the current one is better
            if max_val > best_match_val:
                best_match_val = max_val
                best_match_loc = max_loc
                best_match_size = (w, h)

        # Check if the best match is above the threshold
        if best_match_val >= match_threshold and best_match_loc is not None:
            # Convert ROI coordinates to image coordinates
            top_left = (x1 + best_match_loc[0], y1 + best_match_loc[1])
            bottom_right = (top_left[0] + best_match_size[0], top_left[1] + best_match_size[1])
            return True, top_left, bottom_right
        
        return False, None, None


    # Function to detect dark objects in ROI
    def find_nose(self, frame, roi, dark_threshold, area_threshold):
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Extract the ROI
        x1, y1, x2, y2 = roi
        roi_gray = gray[y1:y2, x1:x2]

        # Threshold the image to get the dark regions
        _, thresholded = cv2.threshold(roi_gray, dark_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # Find contours of the dark regions
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Check each contour to see if it meets the area threshold
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > area_threshold:
                # Get bounding box of the contour
                x, y, w, h = cv2.boundingRect(contour)
                return True, (x1+x, y1+y), (x1+x+w, y1+y+h)  # Object detected
        return False, None, None  # No object detected
