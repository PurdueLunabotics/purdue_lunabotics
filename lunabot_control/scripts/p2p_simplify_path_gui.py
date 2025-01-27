import numpy as np
import sys, pygame

pygame.init()
size = width, height = 1500, 900
points = []
simple_points = []
screen = pygame.display.set_mode(size)



### Simplifies a complex path by removing points that are close to colinear with their neighbors
### ensures that gradual changes are still done

def simplify_path(points, difference_threshold=0.99):
    if len(points)==0:
        return points
    print("-------")
    print(points)
    filtered_points = [points[0]]
    last_filtered_index = 0
    if len(points) >= 3:
        for i in range(2,len(points)):
            print("---")
            p1 = points[last_filtered_index]
            p2 = points[last_filtered_index+1]
            p3 = points[i]
            print(filtered_points)
            print(last_filtered_index)
            print(str(p1)+" "+str(p2)+" "+str(p3))
            # find projection of vectors on each other
            #vector 1 is from the last filtered point to the next point in the complex path
            v1 = np.array(p2,'float64') - np.array(p1,'float64')
            v1 /= np.linalg.norm(v1)  # normalize
            #vector 2 is from the last filtered point to the current point in the complex path
            v2 = np.array(p3,'float64') - np.array(p1,'float64')
            v2 /= np.linalg.norm(v2)  # normalize
            print(str(v1)+" "+str(v2))
            
            # calculate projection size
            projection_size = np.dot(v1, v2)
            
            print(projection_size)
            
            # if within difference threshold, keep the point
            if projection_size <= difference_threshold:
                j=0
                #check the points between the indicies of p1 and p3 to find the last p2 that isn't within tolerance
                for j in range(0,i-last_filtered_index):
                    p2 = points[last_filtered_index+1+j]
                    v1 = np.array(p2,'float64') - np.array(p1,'float64')
                    v1 /= np.linalg.norm(v1)  # normalize
                    v2 = np.array(p3,'float64') - np.array(p1,'float64')
                    v2 /= np.linalg.norm(v2)  # normalize
                    projection_size = np.dot(v1, v2)
                    if projection_size>difference_threshold:
                        p2 = points[last_filtered_index+j]
                        break
                
                filtered_points.append(p2)
                last_filtered_index = last_filtered_index+j


            # add last point to end up in same location
            if i == len(points)-1:
                filtered_points.append(points[-1])
    else: 
        filtered_points = points
    if len(filtered_points) == 0:
        filtered_points = points
        
    #uncomment in real code:
    #marker_points = self.__point_list_to_ros_point_list(filtered_points)
    #marker_points.insert(0, marker_points[0])
    #marker_points.insert(0, Point(self.robot_pose[0], self.robot_pose[1], 0))
    #return filtered_points, marker_points
    
    
    return filtered_points

if len(points)!=0:
    simple_points = simplify_path(points)
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: 
            print(simple_points)
            pygame.quit()
            sys.exit()
        if event.type == pygame.MOUSEBUTTONDOWN:
            points.append(event.pos)
            simple_points = simplify_path(points)
    
    screen.fill((140,180,210))
    if len(points)>=2:
        pygame.draw.lines(screen, (240,50,70),False,points,6)
    if len(simple_points)>=2:
        pygame.draw.lines(screen, (50,230,90),False,simple_points,3)
    
    pygame.display.flip()
