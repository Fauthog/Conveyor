import cv2
import numpy as np
import concurrent.futures
import threading

class find_shrimp_features:
    def __init__(self):
        self.ear_template = None
        self.ear_scale_factors = None
        self.ear_scaled_templates = []

        self.match_count_value = 0  # Initial count of matches

    def set_ear_template(self, template_image_path, low_scale_factor, high_scale_factor, scale_step, scale_divider):
        # Load the template and convert to grayscale (if not already grayscale)
        self.template = cv2.imread(template_image_path, 0)
        
        # Apply histogram equalization to the template
        self.template = cv2.equalizeHist(self.template)
        
        # Define scale factors to iterate over
        self.scale_factors = np.linspace(low_scale_factor, high_scale_factor, scale_step)
        
        # Pre-calculate scaled templates
        self.scaled_templates = []

        for scale in self.scale_factors:
            self.scaled_templates.append(cv2.resize(self.template, None, fx=scale/scale_divider, fy=scale/scale_divider, interpolation=cv2.INTER_CUBIC))

    def generate_rotated_images(self, image, rotation_range, steps):
        h, w = image.shape[:2]
        center = (w / 2, h / 2)
        angles = np.linspace(-rotation_range / 2, rotation_range / 2, steps)
        
        rotated_images = []
        for angle in angles:
            rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
            rotated_image = cv2.warpAffine(image, rotation_matrix, (w, h), flags=cv2.INTER_CUBIC)
            rotated_images.append(rotated_image)
        
        return rotated_images

    def generate_scaled_and_rotated_templates(self, template_image_path, low_scale_factor, high_scale_factor, scale_step, scale_divider, rotation_range, rotation_steps):
        # Load the template and convert to grayscale (if not already grayscale)
        self.template = cv2.imread(template_image_path, 0)
        
        # Apply histogram equalization to the template
        self.template = cv2.equalizeHist(self.template)
        
        # Define scale factors to iterate over
        self.scale_factors = np.linspace(low_scale_factor, high_scale_factor, scale_step)
        
        # Pre-calculate scaled templates
        self.scaled_templates = []

        for scale in self.scale_factors:
            # Scale the template
            scaled_template = cv2.resize(self.template, None, fx=scale/scale_divider, fy=scale/scale_divider, interpolation=cv2.INTER_CUBIC)
            # Generate rotated images for the scaled template
            rotated_templates = self.generate_rotated_images(scaled_template, rotation_range, rotation_steps)
            self.scaled_templates.extend(rotated_templates)

    
    # def set_ear_template_mask(self, template_image_path, template_mask_image_path, low_scale_factor, high_scale_factor, scale_step):
    #     # Load the template and convert to grayscale (if not already grayscale)
    #     self.template = cv2.imread(template_image_path, 0)
    #     self.template_mask = cv2.imread(template_mask_image_path, 0)

        
    #     # Apply histogram equalization to the template
    #     self.template = cv2.equalizeHist(self.template)
        
    #     # Define scale factors to iterate over
    #     self.scale_factors = np.linspace(low_scale_factor, high_scale_factor, scale_step)
        
    #     # Pre-calculate scaled templates
    #     self.scaled_templates = []
    #     self.scaled_mask_templates = []

    #     for scale in self.scale_factors:
    #         self.scaled_templates.append(cv2.resize(self.template, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC))
    #         self.scaled_mask_templates.append(cv2.resize(self.template_mask, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC))


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

    def match_ear_template(self, template, roi_image, threshold, stop_flag, lock_match_count, scale_divider):
        # thread_id = threading.get_ident()
        # print(f"Thread {thread_id} started")

        # Scale down the template and ROI image by 50%
        #template = cv2.resize(template, (template.shape[1] // scale, template.shape[0] // scale))
        roi_image = cv2.resize(roi_image, (roi_image.shape[1] // scale_divider, roi_image.shape[0] // scale_divider))

        w, h = template.shape[::-1]
        result = cv2.matchTemplate(roi_image, template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(result)

        # Since the images were scaled down, the match location needs to be scaled back up
        scaled_max_loc = (max_loc[0] * scale_divider, max_loc[1] * scale_divider)

        # print(f"Thread {thread_id} ended")
        if max_val >= threshold:
            stop_flag.set()
        with lock_match_count:
            self.match_count_value += 1

        return max_val, scaled_max_loc, (w*scale_divider, h*scale_divider)

    def match_ear_template_mask(self, template, mask, roi_image, threshold, stop_flag, lock_match_count):
        # thread_id = threading.get_ident()
        # print(f"Thread {thread_id} started")
        w, h = template.shape[::-1]
        result = cv2.matchTemplate(roi_image, template, cv2.TM_CCOEFF_NORMED, mask=mask)
        _, max_val, _, max_loc = cv2.minMaxLoc(result)
        # print(f"Thread {thread_id} ended")
        if max_val >= threshold:
            stop_flag.set()
        with lock_match_count:
            self.match_count_value += 1

        return max_val, max_loc, (w, h)


    def find_ear_mt(self, frame, roi, match_threshold, scale_divider):
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

        stop_flag = threading.Event()
        counter = 0
        self.match_count_value = 0
        lock_match_count = threading.Lock()

        # Use ThreadPoolExecutor to parallelize the template matching
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = [executor.submit(self.match_ear_template, template, roi_image, match_threshold, stop_flag, lock_match_count, scale_divider) for template in self.scaled_templates]
            for future in concurrent.futures.as_completed(futures):
                counter += 1
                max_val, max_loc, size = future.result()
                if max_val > best_match_val:
                    best_match_val = max_val
                    best_match_loc = max_loc
                    best_match_size = size
                if best_match_val >= match_threshold:
                    #print("Stop at iteration", counter, "with best val", best_match_val, "total thread started", self.match_count_value)
                    break  # Exit the loop once stop_flag is set

        # Check if the best match is above the threshold
        if best_match_val >= match_threshold and best_match_loc is not None:
            # Convert ROI coordinates to image coordinates
            top_left = (x1 + best_match_loc[0], y1 + best_match_loc[1])
            bottom_right = (top_left[0] + best_match_size[0], top_left[1] + best_match_size[1])
            return True, top_left, bottom_right
        
        return False, None, None

    def find_ear_mt_mask(self, frame, roi, match_threshold):
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

        stop_flag = threading.Event()
        counter = 0
        self.match_count_value = 0
        lock_match_count = threading.Lock()

        # Use ThreadPoolExecutor to parallelize the template matching
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = [executor.submit(self.match_ear_template_mask, template, mask, roi_image, match_threshold, stop_flag, lock_match_count) for template, mask in zip(self.scaled_templates, self.scaled_mask_templates)]
            for future in concurrent.futures.as_completed(futures):
                counter += 1
                max_val, max_loc, size = future.result()
                if max_val > best_match_val:
                    best_match_val = max_val
                    best_match_loc = max_loc
                    best_match_size = size
                if best_match_val >= match_threshold:
                    #print("Stop at iteration", counter, "with best val", best_match_val, "total thread started", self.match_count_value)
                    break  # Exit the loop once stop_flag is set

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
