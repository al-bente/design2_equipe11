#!/usr/bin/env python3
import pygame
import sys
import serial

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("My Pygame Window")
 

connection = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600,
    timeout=1
)

clock = pygame.time.Clock()
FPS = 60

# Main game loop
running = True
while running:
    clock.tick(FPS)
    
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    

    # Draw everything
    screen.fill((255, 255, 255))  # White background
    
    # Update display
    pygame.display.flip()

pygame.quit()
sys.exit()







