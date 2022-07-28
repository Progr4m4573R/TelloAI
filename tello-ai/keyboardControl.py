from curses import keyname
from djitellopy import Tello
import pygame


tello = Tello()

def init():
    pygame.init()
    win = pygame.display.set_mode((400,400))

def getKey(keyName):

    ans = False

    for event in pygame.event.get():
        pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame,'K_{}'.format(keyName))
    if keyInput[myKey]:
        ans = True

    pygame.display.update()

    return ans 

def action(state):

    if state ==True:
        lr,fb,ud,yv = 0,0,0,0
        speed = 50


        #move left right
        if getKey("d"): lr = speed
        elif getKey("a"): lr = -speed
        #up down
        if getKey("UP"): ud = speed
        elif getKey("DOWN"): ud = -speed
        # move forward back
        if getKey("w"): fb = speed
        elif getKey("s"): fb = -speed
        #rotate clockwise and anticlockwise
        if getKey("RIGHT"): yv = speed
        elif getKey("LEFT"): yv = -speed

        if getKey("e"): tello.takeoff()
        if getKey("q"): tello.land()

        return [lr,fb,ud,yv]
    else: pass


def main():
    
    if getKey("LEFT"): print("Left key pressed")
    if getKey("RIGHT"): print("Right key pressed")
    if getKey("UP"): print("UP key pressed")
    if getKey("DOWN"): print("Down key pressed")

    if getKey("w"): print("w key pressed")
    if getKey("a"): print("a key pressed")
    if getKey("s"): print("s key pressed")
    if getKey("d"): print("d key pressed")

if __name__ == '__main__':
    init()
    while True:
        main()