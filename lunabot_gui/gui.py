# @author Ankur Raghavan

import sys, pygame, time, math

pygame.init()
size = width, height = 1500, 900
font = pygame.font.Font("freesansbold.ttf", 32)

# ====================================
#          Generic Classes
# ====================================

class Object(pygame.Surface):  
  # Generic Object, It has a size, position, and a collide function
  def __init__(self, size=(100, 100), pose=(500, 500)):
    self.item = 1
    pygame.Surface.__init__(self, size)
    self.fill(bgPrimary)
    self.rect = self.get_rect()
    self.rect.x = pose[0]
    self.rect.y = pose[1]

  def collide(self, event):
    return self.rect.collidepoint(event.pos)
      
class TwoLineText(Object):
  # an visual object with two lines of text
  def __init__(self, size, pose):
    super().__init__(size, pose)
    self.initDraw()

  def initDraw(self):
    # should be overriden by the visual object
    self.fill(bgPrimary)
    self.textL1 = font.render("", True, textPrimary)
    self.textL2 = font.render("", True, textPrimary)
    self.textRectL1 = self.textL1.get_rect()
    self.textRectL2 = self.textL2.get_rect()

  def draw(self, surf):
    # create the background
    surf.blit(self, self.rect)
    # center the text on the object
    self.textRectL1.center = (
        (self.rect.width // 2) + self.rect.x,
        (self.rect.height // 2) + self.rect.y - 20,
    )
    self.textRectL2.center = (
        (self.rect.width // 2) + self.rect.x,
        (self.rect.height // 2) + self.rect.y + 20,
    )
    # add the text on top
    surf.blit(self.textL1, self.textRectL1)
    surf.blit(self.textL2, self.textRectL2)


class OneLineText(Object):
  # a visual object with one line of text
  def __init__(self, size, pose):
    super().__init__(size, pose)
    self.initDraw()

  def initDraw(self):
    # should be overwritten by the visual object
    self.fill(bgPrimary)
    self.textL1 = font.render("", True, textPrimary)
    self.textRectL1 = self.textL1.get_rect()

  def draw(self, surf):
    # create the background
    surf.blit(self, self.rect)
    # center the text
    self.textRectL1.center = (
        (self.rect.width // 2) + self.rect.x,
        (self.rect.height // 2) + self.rect.y,
    )
    # add the text on top
    surf.blit(self.textL1, self.textRectL1)

class Clickable(Object):
  # Generic Clickable Icon
  def __init__(self, size, pose):
    super().__init__(size, pose)

  def onClick():
    # Called when it is clicked (Implemented in child classes)
    pass

# ====================================
#        Launch Classes
# ====================================

class LaunchButton(TwoLineText, Clickable):
  def __init__(self, launchFile, text):
    self.text = text
    self.launchFile = launchFile
    super().__init__(buttonSize, (500,500))
  
  def initDraw(self):
      self.fill(bgPrimary)
      self.textL1 = font.render("Launch", True, textPrimary)
      self.textL2 = font.render(self.text, True, textPrimary)
      self.textRectL1 = self.textL1.get_rect()
      self.textRectL2 = self.textL2.get_rect()
      
  def onClick(self):
    #TODO: Implement starting lauch file
    print("Would Launch: " + self.launchFile)
    pass
  
class StatusButton(OneLineText):
  def __init__(self, text):
    self.text = text
    self.condition = True
    super().__init__(buttonSize, (100,500))
  
  def initDraw(self):
    if self.condition:
      self.fill((50,200,50))
    else:
      self.fill((200, 50, 50))
    self.textL1 = font.render(self.text, True, textPrimary)
    self.textRectL1 = self.textL1.get_rect()

  def checkCondition(self):
    #implemented in child classes
    self.condition = True
  
# ====================================
#          MISC Objects
# ====================================

class Warning(Object):
  def __init__(self, content):
    if len(warnings) == 0:
      warning_width = 500
      pose = ((width-warning_width)/2, 100)
    else:
      pose = ((width-warning_width)/2, warnings[-1].rect.bottom + 20)
    self.initDraw(content)
    super().__init__((warning_width, len(self.textRects) * 30 + 20), pose)

  def initDraw(self, content):
    lis = content.split(" ")
    lines = [[]]
    currLine = 0
    currCount = 0
    for word in lis:
      currCount += len(word) + 1
      if currCount > 25:
        lines.append([])
        currCount = 0
        currLine += 1
      lines[currLine].append(word)
    self.textLs = []
    self.textRects = []
    for line in lines:
      line = " ".join(line)
      self.textLs.append(font.render(line, True, textPrimary))
      self.textRects.append(self.textLs[-1].get_rect())

  def draw(self, surf):
    self.fill((200, 45, 50))
    self.set_alpha(200)
    # print(self.rect.height)
    surf.blit(self, self.rect)
    for i in range(len(self.textRects)):
      offset = i * 30
      self.textRects[i].center = (self.rect.centerx, self.rect.top + offset + 25)
      surf.blit(self.textLs[i], self.textRects[i])
      
# ====================================
#          Draw Varaibles
# ====================================

bg = (16, 20, 110)
bgPrimary = (178, 129, 10)
textPrimary = (255,255,255)
buttonSize = (180,100)

# ====================================
#          Runtime Variables
# ====================================

timeSinceLastClick = 0
iteration = 0
start = time.time_ns()

# Objects
buttons = [LaunchButton("test", "Test")]
statuses = [StatusButton("Status")]
warnings = []
warnings.append(Warning("This is a warning!")) 

# ====================================
#                Main Loop
# ====================================

# Start the window
screen = pygame.display.set_mode(size, pygame.RESIZABLE, pygame.SRCALPHA)
while True:
  # ====================================
  #                Logic
  # ====================================
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      #TODO: Close processes
      pygame.QUIT
      sys.exit()
    if (
            event.type == pygame.MOUSEBUTTONDOWN
            and event.button == 1
            and timeSinceLastClick > 20
        ):
      # Mouse (left) Click
      timeSinceLastClick = 0
      for i in buttons:
        if i.collide(event):
          i.onClick()
      for i in warnings:  # if you click a warning, get rid of it
        if i.collide(event):
          warnings.remove(i)
          del i
  
  
  # ====================================
  #                Draw
  # ====================================
  
  screen.fill(bg)
  
  for i in buttons:
    i.draw(screen)
  for i in statuses:
    i.draw(screen)
  # draw warnings
  for i in warnings:
    i.draw(screen)
  
  
  # ====================================
  #         Loop Running Stuff
  # ====================================
  pygame.display.flip()
  
  # increment counters
  timeSinceLastClick += 1
  iteration += 1
  
  #track loop timing
  n=1000
  if iteration % n == 0:
        end = time.time_ns()
        print(
            str(n) + " iterations took: "
            + str(round((end - start) / 1_000_000 / n, 3))
            + " ms each"
        )
        start = time.time_ns() 
    