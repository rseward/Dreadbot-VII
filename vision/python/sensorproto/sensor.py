#!/usr/bin/env python

from __future__ import print_function

import argparse

"""
Dreadbot sensor processor loop. 

Runs continuously.
Monitors PI camera.
Detects gear target blobs.
Communicates back to the roborio via NetworkTables.
"""

from picamera.array import PiRGBArray
import picamera
import io

import cv2
import numpy
import math
import time
import datetime
from enum import Enum
from networktables import NetworkTables

class Timer(object):
    def __init__(self, name):
        self.name = name
        self.t1 = time.time()
        pass

    def start(self):
        self.t1 = time.time()

    def stop(self):
        self.t2 = time.time()

    def show(self):
        print( "%s: %4.4f" % ( self.name, (self.t2-self.t1) ) )
#|        


class DreadbotSensor(object):

    def __init__(self, sensor, pipeline, debug=False):
        self.pipeline = pipeline
        self.camera = picamera.PiCamera()
        self.camera.resolution = (400, 300)
        self.camera.brightness = 30
        self.sd = NetworkTables.getTable('SmartDashboard')
        self.lastimage = None
        self.blobs = []
        self.debug = debug
        self.rawCapture = None

        # Info in the protocol to send back
        self.name = sensor
        self.suggest_no = 0
        self.active = False
        self.lastsave = None

        """ target_zone: A filter band of where we expect the blobs to 
              appear in the image expressed in a percentage.
              0.25 indicates we expect the target blobs to appear in the
              quarter above and below the center of the image. """
        self.target_zone = (0.20, 1.0) 

    def getImage(self):
        camera = self.camera
        # Create the in-memory stream
        #stream = io.BytesIO()
        #camera.capture(stream, format='bgr')
        #data = numpy.fromstring( stream.getvalue(), dtype=numpy.uint8)
        #image = cv2.imdecode(data, 1)

        #if not(self.rawCapture):
        self.rawCapture = PiRGBArray(camera)
        # use_video_port=True, from picamera docs. The image captured this way
        #  is of lower quality but faster. Seems to be roughly 5 times faster.
        camera.capture(self.rawCapture, format="bgr", use_video_port=True)
        image = self.rawCapture.array
        return image
#|
    def cp(self):
        res = self.camera.resolution
        cx=int(res[0]/2)
        cy=int(res[1]/2)

        return (cx,cy)

    def filter(self, blobs):
        """Use the target_zone to filter blobs that are likely not of interest."""

        res=self.camera.resolution
        center=self.cp()
        top_y=self.target_zone[0]*res[1]
        bot_y=self.target_zone[1]*res[1]

        fblobs = []
        for b in blobs:
            if b.size>5.0:            
                if b.pt[1] >= top_y and b.pt[1] <= bot_y:
                    fblobs.append( b )

        self.blobs = fblobs

        return fblobs

    def analyze(self, blobs):
        """Analyze the blobs and summarize the information to send to the roboRIO"""
        res = self.camera.resolution
        resizefactor=1.0
        cx=int(res[0]/2)
        cy=int(res[1]/2)

        red = (0, 0, 255)
        bcount = 0
        print( "blobs=%s" % blobs )
        self.blobs = self.filter( blobs )
        now = datetime.datetime.now()
        if self.debug:
            cv2.imshow( "Analyze", self.lastimage )
            cv2.waitKey(100)        

        print( "fblobs=%s" % self.blobs )            
        for b in self.blobs:
            print( "   blob=pt=%s, size=%s " % ( b.pt, b.size) )
            #bx=int(cx - int(b.pt[0] * resizefactor))
            #by=int(cy - int(b.pt[1] * resizefactor))
            bx=int(b.pt[0])
            by=int(b.pt[1])                
            print( " - (x=%s , y=%s )" % (bx,by) ) 
            cv2.circle( self.lastimage, (bx,by), int(b.size), red )
            cv2.putText(self.lastimage, "#{}".format(bcount), (bx - 10, by - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, red, 1)
                
            bcount+=1

        cv2.putText( self.lastimage, "%s" % now,  (20, res[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, red, 1 )

        imgcenter = (cx, cy)
        cv2.line( self.lastimage, (cx-5,cy),(cx+5, cy), red )
        cv2.line( self.lastimage, (cx,cy+5),(cx, cy-5), red )

        top_y=int(self.target_zone[0]*res[1])
        bot_y=int(self.target_zone[1]*res[1])

        cv2.line( self.lastimage, (0,top_y),(res[0],top_y), red )
        cv2.line( self.lastimage, (0,bot_y),(res[0],bot_y), red )
        
        self.active = bcount>0

        if self.active and self.debug:
            cv2.imshow( "Analyze", self.lastimage )
            cv2.waitKey(100)
            self.suggest_no += 1

        now = datetime.datetime.now()
        if self.active and \
           ( not(self.lastsave) or (now - self.lastsave).seconds> 5.0 ) :
            self.lastsave = now
            f = "images/%s.jpg" % self.suggest_no
            cv2.imwrite( f, self.lastimage )
            print( "Wrote %s" % f )
            

    def updateRobot(self):
        """
        gearsensor.blob_1.active
        gearsensor.blob_1.suggest_no
        gearsensor.blob_1.cx
        gearsensor.blob_1.cy
        gearsensor.blob_1.radius
        """


 

        # Sort the blobs by size and return the two largest blobs
        blobs = sorted( self.blobs, key=lambda b: b.size, reverse=True)
        
        self.sd.putNumber( "%s.active" % self.name, len(self.blobs) )

        def putNumber( name, val ):
            print( "%s=%s" % (name, val) )
            self.sd.putNumber( name, val )

        # Write each blob to the network table
        bidx=1
        for b in blobs:
            
            putNumber( "%s.blob_%s.cx" % (self.name, bidx), b.pt[0] )
            putNumber( "%s.blob_%s.cy" % (self.name, bidx), b.pt[1] )
            putNumber( "%s.blob_%s.radius" % (self.name, bidx), b.size )           
            bidx += 1
            if bidx>2:
                break
            
        """
        self.sd.putNumber( "%s.x_adj" % self.name, self.x_adj )
        self.sd.putNumber( "%s.y_adj" % self.name, self.y_adj )
        self.sd.putNumber( "%s.z_adj" % self.name, self.z_adj )                
        """
        
        # Update the suggest_no last to guard against partial writes or network failures
        self.sd.putNumber( "%s.suggest_no" % self.name, self.suggest_no )
        self.suggest_no += 1
        

    def watch(self):
        """Watch the camera detecting Pipeline blobs as they appear"""

        itimer = Timer("getImage")
        gtimer = Timer("GRIP Pipeline")
        atimer = Timer("Analyze Blobs")
        utimer = Timer("Update Robo RIO")

        timers = [ itimer, gtimer, atimer, utimer ]
        
        while True:
            tw1=time.time()
            itimer.start()
            self.lastimage = self.getImage( )
            itimer.stop()            
            gtimer.start()
            self.pipeline.process( self.lastimage )
            blobs = self.pipeline.find_blobs_output            
            gtimer.stop()
            #blobs = self.pipeline.filter_contours_output
            atimer.start()
            self.analyze( blobs )
            atimer.stop()
            if True: # not(self.debug): Turns out we want to see pictures and update the robot
                utimer.start()
                self.updateRobot()
                utimer.stop()

            print( "bgr" )
            for t in timers:
                t.show()
                    


    

from geargrip import GripPipeline

def main():
    ap = argparse.ArgumentParser( description="Dreadbot Sensor Prototype" )
    ap.add_argument( "--debug","-d", action="store_true", default=False )

    args = ap.parse_args()
    
    NetworkTables.initialize(server='roborio-3656-frc.local')
    sensor = DreadbotSensor( "gearsensor", GripPipeline(), debug=args.debug)

    sensor.watch()

if "__main__" == __name__:
    main()
        
