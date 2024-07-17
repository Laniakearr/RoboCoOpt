from PIL import Image
import glob,os

def img_to_gif(pattern, output, size, duration):
    # create an empty list called images
    images = []

    # get all the images in the 'images for gif' folder
    frmId=0
    while True: # loop through all png files in the folder
        if not os.path.exists(pattern%frmId):
            break
        im = Image.open(pattern%frmId) # open the image
        im_small = im.resize(size, resample=0) # resize them to make them a bit smaller
        images.append(im_small) # add the image to the list
        frmId+=1

    # calculate the frame number of the last frame (ie the number of images)
    last_frame = (len(images))

    # create 10 extra copies of the last frame (to make the gif spend longer on the most recent data)
    for x in range(0, 9):
        im = images[last_frame-1]
        images.append(im)

    # save as a gif
    images[0].save(output, save_all=True, append_images=images[1:], optimize=False, duration=duration, loop=0)