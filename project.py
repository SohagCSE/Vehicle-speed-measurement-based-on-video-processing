import cv2
import numpy as np
import math


cap = cv2.VideoCapture('cars_cmp_1_3rd.mp4')
Line_h = [40,60,80,100,120,140,160,180]


LHS_road = True
path_showing = True
fps = cap.get(cv2.CAP_PROP_FPS)
frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#print(fps)


class Vehicle(object):
    def __init__(self, id, position):
        self.id = id
        self.positions = [position]     #------- here, position is "centroid" of current frame for this vehicle (x,y) or (widht,height)
        self.unseen_counter = 0      #------- unseen frame counter for this vehicle
        self.seen_counter = 0
        self.counted = False
        self.vehicle_dir = 0
        self.line_flag = [False for i in range(8)]
        self.line_crossing = -1
        self.speed = -1


    @property
    def last_position(self):    #------ "[-1]" will give the last element of the array 
        return self.positions[-1]
    
    @property
    def last_position2(self):
        return self.positions[-2]

    def add_position(self, new_position):
        self.positions.append(new_position)
        self.unseen_counter = 0
        self.seen_counter += 1

    def draw(self, output_image):
        if self.speed != -1:
            cv2.putText(output_image, ("%02d" % self.speed), self.positions[-1], cv2.FONT_HERSHEY_PLAIN, 1, (0,255, 0), 2)

        if path_showing == True:
            for point in self.positions:
                cv2.circle(output_image, point, 2, (0, 0, 255), -1)
                cv2.polylines(output_image, [np.int32(self.positions)], False, (0, 0, 255), 1)
            
                


class VehicleStorage(object):
    def __init__(self, shape, divider):

        self.height, self.width = shape
        self.divider = divider      #-------- here, divider is height which is divide by 2, thats why "divider" is mid height

        self.vehicles = []
        self.next_vehicle_id = 0
        self.vehicle_count = 0
        self.vehicle_LHS = 0
        self.vehicle_RHS = 0
        self.max_unseen_frames = 10


    @staticmethod
    def get_vector(a, b):
        dx = float(b[0] - a[0])
        dy = float(b[1] - a[1])

        distance = math.sqrt(dx**2 + dy**2)

        if dy > 0:
            angle = math.degrees(math.atan(-dx/dy))
        elif dy == 0:
            if dx < 0:
                angle = 90.0
            elif dx > 0:
                angle = -90.0
            else:
                angle = 0.0
        else:
            if dx < 0:
                angle = 180 - math.degrees(math.atan(dx/dy))
            elif dx > 0:
                angle = -180 - math.degrees(math.atan(dx/dy))
            else:
                angle = 180.0        

        return distance, angle, dx, dy 


    @staticmethod
    def is_valid_vector(a, b):
        distance, angle, _, _ = a
        threshold_distance = 12.0
        return (distance <= threshold_distance)

    #----- if "current_centroid" and "vehicle_prev_centroid" fulfil threshold, then add "curr_centroid" to "vehicle.positions" and remove from "centroid_list"
    # increase "seen_counter" if match, otherwise increase "unseen_counter" if not match
    def update_vehicle(self, vehicle, centroid_storage):
        
        for i, match in enumerate(centroid_storage):
            contour, centroid = match
            
            vector = self.get_vector(vehicle.last_position, centroid)
            
            if vehicle.seen_counter > 2:
                prevVector = self.get_vector(vehicle.last_position2, vehicle.last_position)
                angleDev = abs(prevVector[1]-vector[1])
            else:
                angleDev = 0
            
            if self.is_valid_vector(vector, angleDev):       
                vehicle.add_position(centroid)
                vehicle.seen_counter += 1

                if vector[3] > 0:        #---------- if 'dy' positive that means vehicle moving "down"
                    vehicle.vehicle_dir = 1
                elif vector[3] < 0:
                    vehicle.vehicle_dir = -1
                return i
       
        vehicle.unseen_counter += 1      #------- if any match not found
        return None


    def update_count(self, frame_no, centroid_storage, output_image = None):                   #-------- here, "matches" are the all stored "centroids"

        # First update all the existing vehicles
        for vehicle in self.vehicles:
            i = self.update_vehicle(vehicle, centroid_storage)       #----- here, 'i' is the position of "blobs" which match with selected vehicle 
            if i is not None:
                del centroid_storage[i]       #--------- remove centroid from centroid_storage if exist in vehicles[]

        # Add new vehicles based on the remaining matches
        for match in centroid_storage:
            contour, centroid = match
            new_vehicle = Vehicle(self.next_vehicle_id, centroid)       #--------- here, new_vehicle is an object of "Vehicle" class
            self.next_vehicle_id += 1
            self.vehicles.append(new_vehicle)   #--------- add new_vehicle for each "detection"

        #-------- Vehicle_counting
        for vehicle in self.vehicles:
            if not vehicle.counted and (((vehicle.last_position[1] > self.divider) and (vehicle.vehicle_dir == 1)) or
                                          ((vehicle.last_position[1] < self.divider) and (vehicle.vehicle_dir == -1))) and (vehicle.seen_counter > 6):

                vehicle.counted = True

                if ((vehicle.last_position[1] > self.divider) and (vehicle.vehicle_dir == 1) ):
                    self.vehicle_RHS += 1
                    self.vehicle_count += 1

                elif ((vehicle.last_position[1] < self.divider) and (vehicle.vehicle_dir == -1) ):
                    self.vehicle_LHS += 1
                    self.vehicle_count += 1


        #----------- Speed_Measuring
        for vehicle in self.vehicles:
            for j in range(0,8):
                if not vehicle.line_flag[j] and (((vehicle.last_position[1] > Line_h[j]) and (vehicle.vehicle_dir == 1)) or
                                          ((vehicle.last_position[1] < Line_h[j]) and (vehicle.vehicle_dir == -1))) and (vehicle.seen_counter > 6):
                    
                    vehicle.line_flag[j] = True
                    
                    if vehicle.line_crossing != -1:
                        frame_dif = math.fabs(vehicle.line_crossing - frame_no)
                        
                        if frame_dif != 0:
                            vehicle.speed = (29.97*10)/frame_dif
                        
                    vehicle.line_crossing = frame_no


                    

        if output_image is not None:
            
            for vehicle in self.vehicles:       #------ drawing path for each vehicle
                vehicle.draw(output_image)
                
            if LHS_road == True:
                LHS_temp = self.vehicle_LHS
                RHS_temp = self.vehicle_RHS
            else:
                LHS_temp = self.vehicle_RHS
                RHS_temp = self.vehicle_LHS
                
            cv2.putText(output_image, ("LH Lane: %02d" % LHS_temp), (12, 20), cv2.FONT_HERSHEY_PLAIN, 1.2, (127,255, 255), 2)
            cv2.putText(output_image, ("RH Lane: %02d" % RHS_temp), (216, 20), cv2.FONT_HERSHEY_PLAIN, 1.2, (127, 255, 255), 2)

            if path_showing:
                for j in range(0,8):
                    cv2.line(output_image,(0,Line_h[j]),(frame_w,Line_h[j]),(255,0,0),1)
                
            
        
        self.vehicles[:] = [ v for v in self.vehicles
                             if not v.unseen_counter >= self.max_unseen_frames ]   #------- store only that vehicle which are "not" crossed the limit of max_unseen


centroid_storage = []
car_counter = None
frame_no = 0
total_cars = 0
LINE_THICKNESS = 1

car_cascade = cv2.CascadeClassifier('cars.xml')


while True:
    frame_no = frame_no + 1
    
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cars = car_cascade.detectMultiScale(gray, 1.05, 3)

    #---------------------------------------------------------
    for (x,y,w,h) in cars:
        center = (int(x + w/2), int(y + h/2))
        centroid_storage.append(((x, y, w, h), center))
		
    for (i, match) in enumerate(centroid_storage):
        contour, centroid = match
        x, y, w, h = contour

        if path_showing == True:
            cv2.rectangle(frame, (x, y), (x + w - 1, y + h - 1), (255, 255, 255), LINE_THICKNESS)

    
    if car_counter is None:
        car_counter = VehicleStorage(frame.shape[:2], frame.shape[0] / 2) # here, frame.shape[0] is "height" and frame.shape[1] is "width"
		
    car_counter.update_count(frame_no, centroid_storage, frame)
    current_count = car_counter.vehicle_RHS + car_counter.vehicle_LHS

    if current_count > total_cars:
        cv2.line(frame, (0, int(frame_h/2)),(frame_w, int(frame_h/2)),(0,255,0), 2*LINE_THICKNESS)
    else:
        cv2.line(frame, (0, int(frame_h/2)),(frame_w, int(frame_h/2)),(0,0,255), LINE_THICKNESS)
		
    total_cars = current_count
	
    cv2.imshow("video", frame)
	
    #out.write(frame)

	

    kkk = cv2.waitKey(25) & 0xFF
    if kkk  == ord('q'):    # when press key 'q' stop video
        break
    elif kkk == ord('s'):    # when press key 's' save that frame
        cv2.imwrite('capture.jpg',frame)
    elif kkk == ord('p'):
        if path_showing == True:
            path_showing = False
        else:
            path_showing = True


cap.release()
cv2.destroyAllWindows()
