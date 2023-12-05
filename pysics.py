import math
import numpy as np

globalObjectList = []
globalWidth = 0
globalHeight = 0
global_gravity = [0, 0]
useGlobalGravity = False

X_POS = 0
Y_POS = 1

class Object :
    x_pos = 0
    y_pos = 0
    mass = 1
    x_max_min = [0,0]
    y_max_min = [0,0]
    width = 0
    height = 0
    gravity_force = [0, 1]
    force = [0,0]
    
    rotate_value = 0
    is_rotated = False #True일 경우 회전중임
    fix_rotate = True # False일 경우 어떠한 경우에도 회전하지 않음
    
    velocity = [0,0]
    max_velocity = [10,10]
    is_gravity = False # True일 경우 중력 활성화
    active_rigidbody = False # True일 경우 리지드바디 활성화
    
    active_elastic_collision = False #True일 경우 탄성충돌
    
    colision_dectect_aabb = False #True일 경우 aabb 충돌 판정
    colision_detect_obb = False #미구현
    
    def __init__(self) :
        globalObjectList.append(self)
        self.active_elastic_collision = False
        self.gravity_force = [0,1]
        self.force = [0,0]
        self.is_move = True
        self.fixed_object = False
        return
    
    def __init__(self, x_pos, y_pos) :
        globalObjectList.append(self)
        self.active_elastic_collision = False
        self.gravity_force = [0,1]
        self.force = [0,0]
        self.x_pos = x_pos
        self.y_pos = y_pos
        
        self.is_move = True
        self.fixed_object = False
        return True
    
    def Position(self) :
        #자식 class에게 넘기는 용
        #자식클래스에서는 무게중심을 반환
        return 0
    
    
    
    def RotateStart(self, radian, is_collied) :
        if (self.fix_rotate) == True :
            return
        if (self.is_rotated == False) :
            if (is_collied != -1) :
                self.is_rotated = True
                self.Rotate(radian)
        else :
            if (is_collied == -1) :
                self.Rotate(self.rotate_value)
            else :
                self.Rotate(radian)
            
    def Rotate(self, radian) :
        self.rotate_value = radian
        angle = math.radians(radian)
        sin, cos = np.sin(angle), np.cos(angle)
        rotation_matrix = np.array([(cos, -sin),(sin, cos)])
        self.originXY = self.originXY @ rotation_matrix
        
        return self.originXY
    
    
    
    def RigidBody(self, target) :
        #실행이 안되는 조건
        if (target == -1) : # 만약 target이 없으면
            return False
        if (self.active_rigidbody == False) or (target.active_rigidbody == False) : #타겟이나 자신이 리지드바디 혀옹을 안 하면
            return False
        
        velo = self.velocity
        
        #허용한다면
        if (self.active_elastic_collision) : #탄성충돌 ( 무한정 충돌)
            if (target.fixed_object == True) :
                self.velocity = [-self.velocity[X_POS], -self.velocity[Y_POS]]
                target.velocity = [0,0]
                
                return True

            self.velocity = [target.velocity[X_POS], target.velocity[Y_POS]]
            target.velocity = [velo[X_POS], velo[Y_POS]]
            return True
        
        #비탄성충돌
        
        if (target.fixed_object == True) or (self.fixed_object == True) :
            self.is_move = False
            self.velocity = [0,0]
            target.velocity = [0,0]
            return True
        
        if (self.is_move == False) : 
            target.is_move = False
            target.y_pos -= self.height/2
            return True
        
        elif (target.is_move == False) :
            self.y_pos -= target.height/2
            self.is_move = False
        
        x_velo , y_velo = (target.velocity[X_POS] + velo[X_POS])/ 2, (target.velocity[Y_POS] + velo[Y_POS]) / 2
        self.velocity = [x_velo, y_velo]
        target.velocity = [x_velo, y_velo]
        
        
        
        return True
    
    def colisionDetect(self, target) :
        if self.colision_dectect_aabb == False :
            if self.colision_detect_obb == False :
                return 0
            else :
                temp = 0
                #obb detect
        else :
            return colisionDetect_AABB(self, target)
            #aabb return    
            
    def getXY(self) :
        return (self.x_pos, self.y_pos)
    def getXYList(self) :
                return [self.x_pos, self.y_pos]
            
    def Addforce(self, force) :
        self.force[X_POS] += force[X_POS]
        self.force[Y_POS] += force[Y_POS]
        return self.force
    
    def Move(self) :
        
        if (self.is_move == False) :
            return 0
        if self.fixed_object == True :
            return 0
        # 
        self.velocity[X_POS] += self.force[X_POS]
        self.velocity[Y_POS] += self.force[Y_POS]
        
        # 중력 관련해서 업데이트
        if (self.is_gravity == True) :
            if(useGlobalGravity == True) :
                self.velocity[X_POS] += global_gravity[X_POS]
                self.velocity[Y_POS] += global_gravity[Y_POS]
            else :
                self.velocity[X_POS] += self.gravity_force[X_POS]
                self.velocity[Y_POS] += self.gravity_force[Y_POS]

        if self.velocity[0] > self.max_velocity[0] :
            self.velocity[0] = self.max_velocity[0]

        if self.velocity[1] > self.max_velocity[1] :
            self.velocity[1] = self.max_velocity[1]
        
        
        
        self.x_pos += self.velocity[X_POS]
        self.y_pos +=  self.velocity[Y_POS]

                
        return self.velocity   
            
    def Update(self) :
        if(self.colision_dectect_aabb or self.colision_detect_obb) :
            for i in globalObjectList[globalObjectList.index(self):] :
                if ( i == self) :
                    self.RotateStart(self.rotate_value, -1)
                if (i != self) :
                    other = self.colisionDetect(i)
                    self.RigidBody(other[1])
                    self.RotateStart(other[0], other[1])
    
    
    
    
    
    
    
    
    
    
    
class Circle(Object) :
    
    def __init__(self, x_pos, y_pos, radius, color = (255,255,255)) :
        super().__init__(x_pos, y_pos)
        self.radius = radius
        self.color = color
        self.width = 2 * radius
        self.height = 2 * radius
        self.x_max_min = [x_pos+radius, x_pos-radius]
        self.y_max_min = [y_pos - radius, y_pos + radius]
        self.velocity = [0,0]
        self.force = [0,0]
        self.mass = 1
        self.is_rotated = False
        self.fix_rotate = True
        self.location = (x_pos, y_pos)

    
    def getCirclePos(self) :
        return [self.x_pos, self.y_pos]
    
    def getCircleRadian(self) :
        return [self.radius]

    def Position(self) :
        return (self.x_pos, self.y_pos)

    def Move(self):
        super().Move()
                    
        if (self.y_pos + self.height/2 >= globalHeight) :
            self.y_pos = globalHeight - self.height/2
            if (self.active_elastic_collision == True) :
                self.velocity[Y_POS] = -self.velocity[Y_POS]
            else :
                self.is_move = False
            
        elif (self.y_pos - self.height/2 <= 0) :
            self.y_pos = globalHeight + self.height/2
            if (self.active_elastic_collision == True) :
                self.velocity[Y_POS] = -self.velocity[Y_POS]
            else :
                self.is_move = False
                
        if (self.x_pos + self.width/2 >= globalWidth) :
            self.x_pos = globalWidth - self.width/2
            if (self.active_elastic_collision == True) :
                self.velocity[X_POS] = -self.velocity[X_POS]
            else :
                self.is_move = False
            
        elif (self.x_pos - self.width/2 <= 0) :
            self.x_pos = self.width/2
            if (self.active_elastic_collision == True) :
                self.velocity[X_POS] = -self.velocity[X_POS]
            else :
                self.is_move = False
                            
    def Update(self):
        super().Update()
        self.Move()
        self.x_max_min = [self.x_pos+self.radius, self.x_pos-self.radius]
        self.y_max_min = [self.y_pos - self.radius, self.y_pos + self.radius]

    
                  
 

 






    
class Rect(Object) :
    def __init__(self, x_pos, y_pos, width, height, color = (255, 255, 255)) :
        super().__init__(x_pos, y_pos)
        self.width = width
        self.height = height
        self.color = color
        self.x_max_min = [x_pos + width, x_pos]
        self.y_max_min = [y_pos, y_pos + height]
        self.velocity = [0,0]
        self.force = [0,0]
        self.mass = 1
        self.originXY =np.array([(-width/2, -height/2),
                                 (width/2, -height/2),
                                 (width/2, height/2),
                                (-width/2, height/2)]) 
        self.location = (x_pos, y_pos)
        self.is_rotated = False
        self.fix_rotate = True
        
        self.now_position = self.originXY + self.location
    
    def Position(self):
        return (self.x_pos, self.y_pos)
    
    
    def Move(self):
        super().Move() 
                
        if (self.is_move == False) :
            return 0
        if self.fixed_object == True :
            return 0   
        
        if (self.x_pos + self.width/2 >= globalWidth) :
            self.x_pos = globalWidth - self.width/2
            if (self.active_elastic_collision == True) :
                self.velocity[X_POS] = -self.velocity[X_POS]
            else :
                self.is_move = False
                
        elif ((self.x_pos - self.width/2 <= 0)) :
            self.x_pos = self.width/2
            if (self.active_elastic_collision == True) :
                self.velocity[X_POS] = -self.velocity[X_POS]
            else :
                self.is_move = False    



        if (self.y_pos + self.height/2 >= globalHeight) :
            self.y_pos = globalHeight - self.height/2
            if (self.active_elastic_collision == True) :
                self.velocity[Y_POS] = -self.velocity[Y_POS]
            else :
                self.is_move = False
        elif (self.y_pos - self.height/2 <= 0) :
            self.y_pos = self.height/2
            if (self.active_elastic_collision == True) :
                self.velocity[Y_POS] = -self.velocity[Y_POS]
            else :
                self.is_move = False
            
    def Update(self):
        super().Update()
        self.Move()
        self.location = (self.x_pos, self.y_pos)
        self.now_position = self.originXY + self.location
        self.x_max_min = [self.x_pos + self.width/2, self.x_pos - self.width/2]
        self.y_max_min = [self.y_pos - self.height/2, self.y_pos + self.height/2]
        
      #  if(self.colision_dectect_aabb or self.colision_detect_obb) :
            #for i in globalObjectList :
            #    if (i != self) :
            #        self.colisionDetect(i)

        


def colisionDetect_AABB(origin : Object, other : Object) :
    MAX = 0
    MIN = 1
    if ((origin.x_max_min[MAX] >= other.x_max_min[MIN] and origin.x_max_min[MIN] < other.x_max_min[MAX]) and (origin.y_max_min[MAX] < other.y_max_min[MIN] and origin.y_max_min[MIN] > other.y_max_min[MAX] )) :
        
        if (origin.Position()[0] == other.Position()[0] or origin.Position()[1] == other.Position()[1]) :
            return (0, other)
        
        ori_vel_vec = np.array([origin.velocity[0], origin.velocity[1]])        
        other_vel_vec = np.array([other.velocity[0], other.velocity[1]])        
        ori_to_other_vec = other_vel_vec - ori_vel_vec
        
        costheta = np.dot(ori_vel_vec, ori_to_other_vec)
        
        costheta = costheta/(np.linalg.norm(ori_vel_vec) + np.linalg.norm(ori_to_other_vec))

        angle = costheta

        other.RotateStart(-angle, 1)
        
        return (angle, other)
    return (0, -1)
    
