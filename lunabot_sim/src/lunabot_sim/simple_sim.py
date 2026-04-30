import sys

import pygame, time
import numpy as np

pygame.init()
size = width, height = 1500, 900
font = pygame.font.Font("freesansbold.ttf", 32)

# Colors
black = (0, 0, 0)
white = (255, 255, 255)
bg = (207, 185, 145)

class Object(pygame.Surface):
  def __init__(self, size=(100, 100), pose=(500, 500)):
      self.item = 1
      pygame.Surface.__init__(self, size)
      self.fill(white)
      self.rect = self.get_rect()
      self.rect.x = pose[0]
      self.rect.y = pose[1]

  def collide(self, event):
      return self.rect.collidepoint(event.pos)
    
class Robot(Object):
  def __init__(self, pose, size):
    super().__init__(size=size, pose=pose)
    self.direction = 0.0 #angle in degrees
    self.img = pygame.image.load("robotIconOff.svg")

  
  def move(self, v, vtheta):
    # move in a global frame
    self.rect.x += v * np.cos(np.deg2rad(self.direction))
    self.rect.y += v * np.sin(np.deg2rad(self.direction))

    self.direction += vtheta
    print(self.direction)

    self.checkCollisions(v*.1)  # check if object is colliding with anything

  def initDraw(self):
        if self.intakeOn:
            self.img = self.imgIntakeOn
        else:
            self.img = self.imgIntakeOff
        self.imgRect = self.img.get_rect()
        pygame.transform.scale(self.img, (self.rect.width, self.rect.height))
      
  def draw(self, surf: pygame.Surface):
    # draw the images, rotated in the direction of the object
    img = pygame.transform.rotate(self.img, self.direction)
    rect = img.get_rect()
    rect.center = self.rect.center
    surf.blit(img, rect)
    
  def checkCollisions(self, v):
    # Checks if this object is colliding with any other objects and moves if so.
    for i in simItems:
      if i == self:
          continue
      # if this object's rectangle is colliding with another
      if self.rect.colliderect(i.rect):
        self.move(-1*v, 0)  # move self backwards
        print("Attempted to move into an obstacle")
        # if self.rect.colliderect(i.rect):
        #   self.checkCollisions(v)


def generateSim(classes: list):
  simItems = []
  filename = "simSetup.txt"
  try:
    with open(filename, "r") as file:
      for line in file.readlines():
        if len(line) == 0:
          continue
        lis = line.split(",")
        if len(lis) < 3:
          print(f"Error with importing {filename}, ensure that all lines have 3 items seperated by commas")
          exit()
        pose = (int(lis[1]), int(lis[2]))
        size = (int(lis[3]), int(lis[4]))
        obj = None
        for i in classes:
          if lis[0] == i.__name__:
            obj = i(pose, size)
            simItems.append(obj)
            print(obj)
            break
        if not obj:
          print(f"Error with importing {filename}, ensure that all lines have a correct Name.")
          exit()
      file.close()
    return simItems
  except:
      print(f"simSetup.txt is missing")
      exit()

iteration = 1
start = time.time_ns()
classes = [Robot]  # sim classes
simItems = generateSim(classes)
keys = []
# Start the window
screen = pygame.display.set_mode(size, pygame.RESIZABLE, pygame.SRCALPHA)
while True:
  # LOGIC
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      pygame.quit()
      sys.exit()
    if event.type == pygame.KEYDOWN:
      print(event.key)
      if not event.key in keys:
        keys.append(event.key)
    if event.type == pygame.KEYUP:
      keys.remove(event.key)
  
  
  for key in keys:
    if key == 119:
      simItems[0].move(1,0)
    if key == 115:
      simItems[0].move(-1,0)
    if key == 97:
      simItems[0].move(0,0.1)
    if key == 100:
      simItems[0].move(0,-0.1)
        
      # simItems[0].move()
  
  # DRAW
  screen.fill(bg)
  for i in simItems:
    i.draw(screen)
  
  # ----------------
  # Keep display code above
  # show new content
  pygame.display.flip()
  # incremement iterators
  iteration += 1
  if iteration % 1000 == 0:  # keep track of loop timing
      end = time.time_ns()
      print(
          "1000 iterations took: "
          + str(round((end - start) / 1_000_000 / 1_000, 3))
          + " ms each"
      )
      start = time.time_ns()