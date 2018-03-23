'''Camera Attack'''

import numpy as np
import matplotlib.pyplot as plt

from . import sensor
from warp_functions import *

from skimage import data
from skimage.transform import swirl
from skimage import transform, data, io, filters
import Image
import time

def noise_attack(image):
	noise = np.random.randint(0,130,image.shape,dtype=np.dtype("uint8"))
	#print image.shape
	#print type(image)
	return image + noise

def block_attack(image):
	image[115:355,325:450,:] = 0
	return image

def flip_attack(image):
	return np.fliplr(image)

def warp_attack(image):
	image = transform.warp(image, fisheye, mode='wrap')
	image = image*255
	image = image.astype("uint8")
	return image

def colour_attack(image):
	tmp_array = np.zeros(image.shape, dtype="uint8")
	tmp_array[:,:,3] = image[:,:,3]
	image = tmp_array
	return image

def laser_attack(image):
	foreground = Image.open("/home/carla/Downloads/laser_transparent.png")
	foreground = np.array(foreground)
	foreground[:,:,0] = 0
	foreground[:,:,2] = 0
	foreground = Image.fromarray(foreground)
	
	basewidth = 800
	wpercent = (basewidth/float(foreground.size[0]))
	hsize = int((float(foreground.size[1])*float(wpercent)))
	foreground = foreground.resize((basewidth,hsize), Image.ANTIALIAS)
	foreground = foreground.crop((0,200,800,800))

	background = Image.fromarray(image)
	background = background.convert('RGBA')
	print background.size
	combined = Image.new('RGBA',background.size)
	combined = Image.alpha_composite(combined,background)
	combined = Image.alpha_composite(combined,foreground)
	combined = np.array(combined)	

	return combined

def sticker_attack(image):
	foreground = Image.open("/home/carla/Downloads/cutout.png")
	foreground = np.array(foreground)
	foreground = Image.fromarray(foreground)

	background = Image.fromarray(image)
	background = background.convert('RGBA')
	combined = Image.new('RGBA',background.size)
	combined = Image.alpha_composite(combined,background)
	combined = Image.alpha_composite(combined,foreground)
	combined = np.array(combined)	

	return combined	

def blur_attack(image):
	image = filters.gaussian(image, sigma =10, multichannel=True)
	image = image*255
	image = image.astype("uint8")
	return image


def perform_attack(image):
	perform_noise_attack = False 
	perform_block_attack = False
	perform_flip_attack = False
	perform_warp_attack = True
	perform_colour_attack = False
	perform_laser_attack = False 
	perform_sticker_attack = False
	perform_blur_attack = False

	width = image.width
	height = image.height

	image = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
	image = np.reshape(image, (height, width, 4))
	image.setflags(write=1)

	'''if perform_noise_attack:
		image = noise_attack(image)

	if perform_block_attack: 
		image = block_attack(image)

	if perform_flip_attack:
		image = flip_attack(image)'''
	
	if perform_warp_attack:
		image = warp_attack(image)

	if perform_colour_attack:
		image = colour_attack(image)

	if perform_laser_attack:
		image = laser_attack(image)
	
	if perform_sticker_attack:
		image = sticker_attack(image)

	if perform_blur_attack:
		image = blur_attack(image)

	#Serialize the data
	image = np.reshape(image, (height*width*4))
	image = image.tostring()
	return image
