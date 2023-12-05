import pysics
import pygame
import sys
from random import randrange


def SetRectObject(num, maxSize, minSize) :
    result = []
    for i in range(1, num) :
        result.append(pysics.Rect(100 + i * 50, i * 50, randrange(minSize, maxSize), randrange(minSize, maxSize)))
    result.append(pysics.Rect(200 , 2 * 100, randrange(minSize, maxSize), randrange(minSize, maxSize), color=(99,99,99)))
    result.append(pysics.Rect(200 , 1 * 50, randrange(minSize, maxSize), randrange(minSize, maxSize), color= (255,0,0)))
    result.append(pysics.Rect(600 , 1 * 50, randrange(minSize, maxSize), randrange(minSize, maxSize), color= (255,0,0)))
    result.append(pysics.Rect(500 , 1 * 50, 10, 100, color= (255,0,0)))

    return result

pygame.init()
is_running = True 
BLACK = (0,0,0)
pygame.key.set_repeat(30, 30)
SCREEN_WIDTH = 600
SCREEN_HEIGHT = 600
pysics.globalWidth = SCREEN_WIDTH
pysics.globalHeight = SCREEN_HEIGHT
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
clock = pygame.time.Clock()

#player = pysics.Rect(300, 300,20,20)
#player.colision_dectect_aabb = True
#player.active_rigidbody = True



rect_list = SetRectObject(5, 21, 20)

for i in rect_list :
    i.gravity_force
    i.is_gravity = True
    i.active_rigidbody = True
    i.active_elastic_collision = False
 #   ran = randrange(-5,5)
 #   i.gravity_force = [0.1 * ran, 0.1 * ran]
    i.gravity_force = [0, 2]
    #i.velocity = [ran, ran]
    i.fix_rotate = True
    i.colision_dectect_aabb = True
    
rect_list[0].active_elastic_collision = True

rect_list[6].active_elastic_collision = True
rect_list[6].gravity_force = [-0.4, -0.2]

rect_list[7].fix_rotate = False
rect_list[7].active_elastic_collision = True
rect_list[7].gravity_force = [0, -0.5]





while(1) :
    clock.tick(60)
    screen.fill(BLACK)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        #if event.type == pygame.KEYDOWN:
        #    if event.key == pygame.K_LEFT:
        #        player.velocity[0] -= 5
        #    if event.key == pygame.K_RIGHT:
        #        player.velocity[0] += 5
        #    if event.key == pygame.K_UP:
        #        player.velocity[1] = -5
        #    if event.key == pygame.K_DOWN:
        #        player.velocity[1] = +5

    for i in rect_list :
        pygame.draw.polygon(screen, i.color, (i.now_position).tolist())
    for i in pysics.globalObjectList :
        i.Update()
    #pygame.draw.circle(screen, circle1.color, [circle1.x_pos, circle1.y_pos], circle1.radius, 1)
    pygame.display.update()