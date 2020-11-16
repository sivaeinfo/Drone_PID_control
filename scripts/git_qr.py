#!/usr/bin/env python
from sys import argv
import pyzbar as zbar

import cv2

class DetectQRCode(object):

    #@classmethod
    def detect_qr(self, image):
        # create a reader
        scanner = zbar.ImageScanner()

        # configure the reader
        scanner.parse_config('enable')

        # obtain image data
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY,dstCn=0)
        pil = Image.fromarray(gray)
        width, height = pil.size
        raw = pil.tostring()


        # wrap image data
        image = zbar.Image(width, height, 'Y800', raw)

        # scan the image for barcodes
        scanner.scan(image)

        # extract results
        for symbol in image:
            # do something useful with results
            if symbol.data == "None":
                return "Drone bevindt zich buiten het raster"
            else:
                return symbol.data