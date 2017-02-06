#!/usr/bin/env python

from __future__ import print_function

"""
Simple tool for calibrating the HSV filter values for finding a target.

Take an image with the PI camera. Wait for the operator to indentify
targets in the image. Suggest a set of HsV filters to identify the
targets.

Inspired by the blog entry here:

http://www.pyimagesearch.com/2015/03/09/capturing-mouse-click-events-with-python-and-opencv/
"""

import argparse
import cv2
import io
import picamera
import numpy

def click_and_crop():
    print("click_and_crop")

class DreadbotCalibrate(object):

    def __init__(self):
        self.camera = picamera.PiCamera()
        self.camera.resolution = (400,300)
        self.camera.brightness = 30
        self.refPt = []
        self.cropping = False

    def getImage(self):
        camera = self.camera
        # Create the in-memory stream
        stream = io.BytesIO()
        camera.capture(stream, format='jpeg')
        data = numpy.fromstring( stream.getvalue(), dtype=numpy.uint8)
        image = cv2.imdecode(data, 1)
        return image

    def reject_outliers(self, data, m=2.0):
        return data[numpy.abs(data - numpy.mean(data)) < m * numpy.std(data)]
    
    def _reject_outliers(self, data, m = 2.):
        d = numpy.abs( data - numpy.median(data))
        mdev = numpy.median(d)
        s = d/mdev if mdev else 0.
        return data[s<m]

    def summarize(self, name, data ):
        print( "%s" % sorted(data) )
        print( "%s: count=%s, min=%s, max=%s, ave=%s, median=%s" %
               (name,
                data.size,
                numpy.min(data), numpy.max(data), numpy.mean(data), numpy.median(data) )
        )

    def analyze(self, roi):
        _roi = roi.copy()
        _roi = cv2.cvtColor( roi, cv2.COLOR_BGR2HSV )

        hsv_h = []
        hsv_s = []
        hsv_v = []

        height, width, depth = _roi.shape
        print( "height = %s, width = %s, depth = %s" % ( height, width, depth ) )
        for hidx in xrange(0, height):
            for widx in xrange(0, width):
                pix = _roi[hidx,widx]
                hsv_h.append( pix[0] )
                hsv_s.append( pix[1] )
                hsv_v.append( pix[2] )
                
                #for didx in xrange(0, depth):
                #    pix = _roi[hidx,widx,didx]
                #    print( pix )

        self.summarize( "hue", numpy.array(hsv_h) )
        self.summarize( "sat", numpy.array(hsv_s) )
        self.summarize( "val", numpy.array(hsv_v) )
                
        self.summarize( "hue",
                        self.reject_outliers(numpy.array(hsv_h)) )
        self.summarize( "sat",
                        self.reject_outliers(numpy.array(hsv_s)) )
        self.summarize( "val",
                        self.reject_outliers(numpy.array(hsv_v)) )
        
        


    def calibrate(self):
        self.refPt = []        
        self.img = self.getImage()
        clone= self.img.copy()


        cv2.namedWindow("Calibrate")
        cv2.setMouseCallback("Calibrate",self.click_and_crop)
        
        
        while True:
            cv2.imshow( "Calibrate", self.img )
            key = cv2.waitKey( 100 ) & 0xFF

            if key == ord("r"):
                self.img = clone.copy()

            if key == ord("c"):
                break

        if len(self.refPt) > 1:
            refPt = self.refPt
            roi = clone[ refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]

            bigroi = cv2.resize( roi, (0,0), fx=4.0, fy=4.0 )
            cv2.imshow("ROI", bigroi)
            cv2.waitKey(0)

            self.analyze( roi )
                    

    def click_and_crop(self, event, x,y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPt = [(x,y)]
            self.cropping = True

        elif event == cv2.EVENT_LBUTTONUP:
            self.refPt.append((x,y))
            cropping = False

            refPt = self.refPt
            cv2.rectangle( self.img, refPt[0], refPt[1], (0,255,0), 2)
            cv2.imshow( "Calibrate", self.img )
            print( "%s" % refPt )            
            cv2.waitKey( 0 )

    
        
                    
            
def main():
    dbc = DreadbotCalibrate()
    dbc.calibrate()

if "__main__" == __name__:
    main()


        
