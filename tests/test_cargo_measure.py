"""Image evidence, plane geometry, and physically visible box boundaries."""
import copy
import math
import unittest
import cv2
import numpy as np
from cargo_measure import (Camera, front_rectangle_hint, grabcut_mask,
                           visible_bottom_edge, measure)


def scene(width=.185,height=.32,offset=0.,angle=0.):
    cfg={'camera':dict(width=320,height=240,fx=300.,fy=300.,cx=160.,cy=120.,
                      dist=[0.]*5,x=.1,y=0.,z=.235,yaw=0.,pitch=0.),
         'lidar':dict(x=.014,z=.19,half_fov_deg=35,max_range=1.2,scans=10)}
    cam=Camera(cfg);n=np.array([-math.cos(angle),-math.sin(angle)])
    tangent=np.array([-math.sin(angle),math.cos(angle)])
    centre=np.array([1.,offset]);d=float(n@centre)
    left,right=centre+tangent*width/2,centre-tangent*width/2
    corners=np.array([cam.project(np.r_[xy,z]) for xy,z in
                      ((left,height),(right,height),(right,0.),(left,0.))])
    img=np.full((240,320,3),210,np.uint8)
    img[199:]=(160,165,170)
    cv2.fillConvexPoly(img,np.round(corners).astype(np.int32),(65,105,145))
    amin=-math.pi;inc=2*math.pi/640
    ranges=[]
    for a in amin+np.arange(640)*inc:
        direction=np.array([math.cos(a),math.sin(a)])
        den=float(n@direction)
        r=(d-n[0]*.014)/den if abs(den)>1e-8 else -1.
        hit=np.array([.014,0.])+r*direction
        ranges.append(float(r) if r>0 and abs((hit-centre)@tangent)<=width/2 else float('inf'))
    scans=[(amin,inc,ranges)]*10
    return cfg,img,scans,corners


class ImageEvidenceTests(unittest.TestCase):
    def test_rectangle_hint_ignores_background_windows_and_matches_ground(self):
        img=np.full((240,320,3),200,np.uint8)
        cv2.rectangle(img,(140,10),(205,85),(30,40,60),3)  # wrong height/ground
        cv2.rectangle(img,(140,98),(205,212),(65,100,145),-1)
        hint=front_rectangle_hint(img,142,203,180,214)
        self.assertIsNotNone(hint)
        self.assertLess(abs(hint[1]-98),4)
        mask=grabcut_mask(img,142,203,180,rectangle=hint)
        self.assertFalse(mask[0].any())
        self.assertTrue(mask[150,170])

    def test_mask_is_repeatable_even_if_other_opencv_work_changes_the_rng(self):
        img=np.full((240,320,3),200,np.uint8)
        cv2.rectangle(img,(140,98),(205,212),(65,100,145),-1)
        hint=front_rectangle_hint(img,142,203,180,214)
        cv2.setRNGSeed(1);first=grabcut_mask(img,142,203,180,rectangle=hint)
        cv2.setRNGSeed(9981);second=grabcut_mask(img,142,203,180,rectangle=hint)
        np.testing.assert_array_equal(first,second)

    def test_clipped_rectangle_is_not_a_full_visible_box_hint(self):
        img=np.full((240,320,3),200,np.uint8)
        cv2.rectangle(img,(140,0),(205,212),(65,100,145),-1)
        self.assertIsNone(front_rectangle_hint(img,142,203,180,214))

    def test_bottom_requires_a_strong_consistent_image_edge(self):
        gray=np.full((240,320),65,np.uint8);gray[211:]=120
        self.assertAlmostEqual(visible_bottom_edge(gray,140,205,211.5,215.),210.5,delta=1.)
        flat=np.full((240,320),65,np.uint8)
        self.assertIsNone(visible_bottom_edge(flat,140,205,211.5,215.))
        self.assertIsNone(visible_bottom_edge(gray,140,205,239.,239.))

    def test_complete_box_uses_visible_height_and_compensates_face_angle(self):
        for angle in (0.,math.radians(5.),math.radians(-10.)):
            with self.subTest(angle=angle):
                cfg,img,scans,_=scene(angle=angle)
                result,_=measure(img,scans,cfg)
                self.assertLess(abs(result['width']-.185),.012)
                self.assertLess(abs(result['height']-.32),.012)
                self.assertEqual(result['height_source'],'visible_edges')

    def test_common_camera_height_offset_cancels_when_both_edges_are_visible(self):
        cfg,img,scans,_=scene()
        cfg['camera']['z']+=.012
        result,_=measure(img,scans,cfg)
        self.assertEqual(result['height_source'],'visible_edges')
        self.assertLess(abs(result['height']-.32),.012)
        self.assertGreater(result['bottom_z'],.004)

    def test_truncated_box_still_fails_instead_of_locking_wrong_height(self):
        cfg,img,scans,_=scene(height=.75)
        with self.assertRaises(ValueError):measure(img,scans,cfg)


if __name__=='__main__':unittest.main()
