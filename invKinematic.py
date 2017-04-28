#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import pygame
import sys
from pygame.locals import *


WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

EPS = 1e-7

pygame.init()
WIDTH = 700
HEIGHT = 700
screen = pygame.display.set_mode( (WIDTH, HEIGHT) )
pygame.display.set_caption("inv Kinematic")
Sprite=pygame.sprite.Sprite

def IdenticalMatrix(n):
    return np.matrix([[float(j==i) for j in range(n)] for i in range(n)])

def addMatrix(a):
    dim = len(a)
    raw = [[0] * (dim+1) for i in range(dim + 1)]
    for i in range(dim+1):
        raw[i][i] = 1
    for i in range(dim):
        raw[i][-1] = a[i]

    return np.matrix(raw)

def rotateMatrix(theta):
    return np.matrix([[np.cos(theta), np.sin(theta), 0],
                      [-np.sin(theta), np.cos(theta), 0],
                      [0, 0, 1]])

def goback(l, r, n):
    n %= (r - l) * 2
    if n > r - l:
        n = (r - l) * 2 - n
    return n
    
    

class swit(Sprite):
    def __init__ (self,x,y):
        Sprite.__init__(self)
        self.color=(0,0,0)
        self.image=pygame.Surface((Square_size-2,Square_size-2))
        self.image.fill(self.color)
        self.rect=pygame.Rect(x*Square_size+1,y*Square_size+1,(x+1)*Square_size-2,(y+1)*Square_size-2)
        self.mode=0

    def update(self):
	if(self.mode==0):self.color=(100,0,0)
	if(self.mode==1):self.color=(255,0,0)
	if(self.mode>=2):self.color=(self.color[0],120,120)
        self.image.fill(self.color)
    def draw(self,screen):
        screen.blit(self.image,self.rect)

        
class RotateArm():
    def __init__(self, length, color = RED):
        self.width = 10
        self.length = length
        self.basepos = (0, 0)
        self.edgepos = (0, self.length)
        self.mat = rotateMatrix(0).dot(addMatrix((0, self.length)))
        self.color = color
        
    def setPos(self, mat):
        base = np.matrix([[0],
                         [0],
                         [1]])
        edge = np.matrix([[0],
                          [0],
                          [1]])
        
        base = mat.dot(base)
        self.basepos = (base[0,0], base[1,0])
        edge = mat.dot(self.mat).dot(edge)
        self.edgepos = (edge[0,0], edge[1,0])
        
    def setParm(self, theta):
        self.mat = rotateMatrix(theta).dot(addMatrix((0, self.length)))
        
        
    def getMatrix(self):
        return self.mat
    
    def draw(self, screen):
        pygame.draw.line(screen, self.color, self.basepos, self.edgepos, 10)


        
        
class SlideArm():
    def __init__(self, length, theta = np.pi / 2,color = GREEN):
        self.width = 10
        self.length = length
        self.l = length
        self.basepos = (0, 0)
        self.edgepos = (0, self.length)
        self.mat = rotateMatrix(theta).dot(addMatrix((0, self.l)))
        self.theta = theta
        self.color = color
        
    def setPos(self, mat):
        base = np.matrix([[0],
                         [0],
                         [1]])
        edge = np.matrix([[0],
                          [0],
                          [1]])
        
        base = mat.dot(rotateMatrix(self.theta)).dot(addMatrix((0, self.l - self.length))).dot(base)
        self.basepos = (base[0,0], base[1,0])
        edge = mat.dot(self.mat).dot(edge)
        self.edgepos = (edge[0,0], edge[1,0])
        
    def setParm(self, p):
        p = goback(0, 1, p)
        self.l = self.length * p
        self.mat = rotateMatrix(self.theta) * addMatrix((0, self.l))
        
        
    def getMatrix(self):
        return self.mat
    
    def draw(self, screen):
        pygame.draw.line(screen, self.color, self.basepos, self.edgepos, 10)


class expandArm():
    def __init__(self, length, color = BLUE):
        self.width = 10
        self.length = length
        self.l = length
        self.basepos = (0, 0)
        self.edgepos = (0, self.length)
        self.mat = rotateMatrix(np.pi / 2).dot(addMatrix((0, self.l)))
        self.color = color
        
    def setPos(self, mat):
        base = np.matrix([[0],
                         [0],
                         [1]])
        edge = np.matrix([[0],
                          [0],
                          [1]])
        
        base = mat.dot(base)
        self.basepos = (base[0,0], base[1,0])
        edge = mat.dot(self.mat).dot(edge)
        self.edgepos = (edge[0,0], edge[1,0])
        
    def setParm(self, p):
        p = goback(0.1, 1, p)
        self.l = self.length*p
        self.mat = addMatrix((0, self.l))
        
        
    def getMatrix(self):
        return self.mat
    
    def draw(self, screen):
        pygame.draw.line(screen, self.color, self.basepos, self.edgepos, 10)

class Linkage():
    def __init__(self, pos, parts):
        self.pos = pos
        self.parts = parts
        self.partscnt = len(parts)
        self.parms = np.matrix([[0.5] for i in range(self.partscnt)])
        
    def getLastpos(self, parms):
        if parms.shape != (self.partscnt, 1):
            print "number of parametors dosen't match!!"
            return

        pos = self.forward(parms).dot( np.matrix([[0],
                                                  [0],
                                                  [1]]))
        
        return np.matrix([[pos[0,0]],[pos[1,0]]])
                           
    def forward(self, parms):
        if parms.shape != (self.partscnt, 1):
            print "number of parametors dosen't match!!"
            return

        self.parms = parms
        mat = addMatrix(self.pos)
        for i in range(len(parms)):
            self.parts[i].setParm(parms[i,0])
            self.parts[i].setPos(mat)
            mat = mat.dot(self.parts[i].getMatrix())
           
        return mat

    
    def jacobi(self, parms, dim=2):
        n = parms.shape[0]
        ret = np.matrix([[0] * n for i in range(dim)])
        zeros = np.matrix([[0] for i in range(n)])

        
        for i in range(n):
            delta = np.matrix([[EPS * (p == i)] for p in range(n)])
            tmp1 = (self.getLastpos(parms + delta) - self.getLastpos(parms))/ EPS
            tmp2 = (self.getLastpos(parms) - self.getLastpos(parms - delta))/ EPS
            for j in range(dim):
                if abs(tmp1[j,0]) > abs(tmp2[j,0]):
                    ret[j,i] = tmp1[j,0]
                else:
                    ret[j,i] = tmp2[j,0]

        return ret

    
    def inv(self, pos):
        parm = self.parms
        try:
            cnt = 0
            for i in range(100):
                nowpos = self.getLastpos(parm)
                print max(abs(pos-nowpos))
                if max(abs(pos-nowpos)) < EPS:
                    break

                j = self.jacobi(parm)
                d = np.linalg.pinv(j)*(pos-nowpos)
                parm += d

            else:
                self.parms = np.matrix([[0.5] for i in range(self.partscnt)])
        except:
            parm = np.matrix([[0.5] for i in range(self.partscnt)])

        return parm
        
        
        
            
    def draw(self, screen):
        screen.fill(WHITE)
        mat = IdenticalMatrix(3)
        for part in self.parts:
            part.draw(screen)

    

        
def main():
    sysfont = pygame.font.SysFont(None, 80)
    clock=pygame.time.Clock()

    parts = [RotateArm(180), RotateArm(90), SlideArm(50, np.pi/-2), RotateArm(180)]
    
    linkage = Linkage((WIDTH/2, HEIGHT/2), parts)
    
    while True:
        clock.tick(60)
        screen.fill((0,0,255))
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit()
        x, y = pygame.mouse.get_pos()
        parms = linkage.inv(np.matrix([[x],[y]]))
        linkage.forward(np.matrix(parms))
        linkage.draw(screen)
        pygame.display.flip()


if __name__ == "__main__":
    main()
