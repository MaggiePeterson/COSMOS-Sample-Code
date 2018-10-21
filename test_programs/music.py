import pygame
pygame.mixer.init()
pygame.mixer.music.load("BST.wav")
pygame.mixer.music.play()
pygame.mixer.music.set_volume(1)
##while pygame.mixer.music.get_busy() == True:
##    continue