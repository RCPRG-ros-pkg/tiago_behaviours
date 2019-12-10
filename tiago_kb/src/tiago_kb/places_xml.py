#!/usr/bin/env python

import os
import math
import PyKDL
import numpy as np
import xml.dom.minidom as minidom
import cv2
import yaml

def rad2deg(angle_rad):
    return 180.0 * angle_rad / math.pi

def parsePointStr(point_str):
    fields = point_str.strip().split()
    assert len(fields) == 2
    return ( float(fields[0]), float(fields[1]) )

class VolumetricPlace:
    def __init__(self, pl_id, name, img):
        self.__pl_id__  = pl_id
        self.__name__  = name
        self.__img__  = img

    def getType(self):
        return 'volumetric'

    def getId(self):
        return self.__pl_id__

    def getName(self):
        return self.__name__

    def getImage(self):
        return self.__img__

class PointPlace:
    def __init__(self, pl_id, name, pt, n):
        self.__pl_id__  = pl_id
        self.__name__  = name
        self.__pt__ = pt
        self.__n__ = n

    def getType(self):
        return 'point'

    def getId(self):
        return self.__pl_id__

    def getName(self):
        return self.__name__

    def getPt(self):
        return self.__pt__

    def getN(self):
        return self.__n__

class MapContext:
    def __init__(self, name, resolution, origin, map_img):
        self.__name__ = name
        self.__map_resolution__ = resolution
        self.__map_origin__ = origin
        self.__map_img__ = map_img
        self.__volumetric_mask__ = None
        self.__volumetric_places__ = []
        self.__point_places__ = []

    def getName(self):
        return self.__name__

    def getMapOrigin(self):
        return self.__map_origin__

    def getMapResolution(self):
        return self.__map_resolution__

    def setVolumetricMask(self, img):
        if self.__volumetric_mask__ is None:
            self.__volumetric_mask__ = img
        else:
            raise Exception('Method setVolumetricMask executed twice for the same MapContext object.')

    def getVolumetricMask(self):
        return self.__volumetric_mask__

    def getPlaceById(self, pl_id):
        for pl in self.__volumetric_places__:
            if pl.getId() == pl_id:
                return pl
        for pl in self.__point_places__:
            if pl.getId() == pl_id:
                return pl
        return None

    def getPlaceByName(self, name):
        for pl in self.__volumetric_places__:
            if pl.getName() == name:
                return pl
        for pl in self.__point_places__:
            if pl.getName() == name:
                return pl
        return None

    def addVolumetricPlace(self, pl_id, name, img):
        if not self.getPlaceById( pl_id ) is None:
            raise Exception('Two places with the same id: "' + pl_id + '"')
        if not self.getPlaceByName( name ) is None:
            raise Exception('Two places with the same name: "' + name + '"')
        self.__volumetric_places__.append( VolumetricPlace(pl_id, name, img) )

    def addPointPlace(self, pl_id, name, position, front_vec):
        if not self.getPlaceById( pl_id ) is None:
            raise Exception('Two places with the same id: "' + pl_id + '"')
        if not self.getPlaceByName( name ) is None:
            raise Exception('Two places with the same name: "' + name + '"')
        self.__point_places__.append( PointPlace(pl_id, name, position, front_vec) )

class PlacesXml:
    def __init__(self, filename):
        xml_str = None
        with open(filename, "r") as f:
            xml_str = f.read()
        dom = minidom.parseString(xml_str)

        '''
        print "COMMENT_NODE", dom.COMMENT_NODE
        print "ELEMENT_NODE", dom.ELEMENT_NODE
        print "ATTRIBUTE_NODE", dom.ATTRIBUTE_NODE
        print "TEXT_NODE", dom.TEXT_NODE
        print "CDATA_SECTION_NODE", dom.CDATA_SECTION_NODE
        print "ENTITY_NODE", dom.ENTITY_NODE
        print "PROCESSING_INSTRUCTION_NODE", dom.PROCESSING_INSTRUCTION_NODE
        print "COMMENT_NODE", dom.COMMENT_NODE
        print "DOCUMENT_NODE", dom.DOCUMENT_NODE
        print "DOCUMENT_TYPE_NODE", dom.DOCUMENT_TYPE_NODE
        print "NOTATION_NODE", dom.NOTATION_NODE
        '''

        head_tail = os.path.split(filename)
        self.__cwd__ = head_tail[0]
        assert self.__cwd__

        self.__map_contexts__ = []
        self.__map_transforms__ = []
        for n in dom.childNodes:
            if n.nodeType == n.ELEMENT_NODE:
                if n.tagName == "places":
                    self.parsePlaces( n )

    def parsePlaces(self, xml):
        assert xml.tagName == "places"

        for n in xml.childNodes:
            if n.nodeType == n.ELEMENT_NODE:
                if n.tagName == "map_context":
                    self.parseMapContext( n )
                elif n.tagName == "map_transform":
                    self.parseMapTransform( n )
                else:
                    raise Exception('Unknown xml tag "' + n.tagName + '" within "places" node.')

    def parseMapContext(self, xml):
        assert xml.tagName == "map_context"

        name_str = xml.getAttribute("name")
        map_param_filename_str = xml.getAttribute("map_param_filename")
        map_filename_str = xml.getAttribute("map_filename")

        map_img = cv2.imread(self.__cwd__ + '/' + map_filename_str, cv2.IMREAD_GRAYSCALE)
        with open(self.__cwd__ + '/' + map_param_filename_str, 'r') as yaml_file:
            map_file_yaml = yaml.safe_load(yaml_file)
            resolution = map_file_yaml['resolution']
            origin = ( map_file_yaml['origin'][0], map_file_yaml['origin'][1] )

        mc = MapContext(name_str, resolution, origin, map_img)

        for n in xml.childNodes:
            if n.nodeType == n.ELEMENT_NODE:
                if n.tagName == "volumetric_mask":
                    self.parseVolumetricMask( n, mc )
                elif n.tagName == "volumetric_place":
                    self.parseVolumetricPlace( n, mc )
                elif n.tagName == "point_place":
                    self.parsePointPlace( n, mc )
                else:
                    raise Exception('Unknown xml tag "' + n.tagName + '" within "places" node.')
        self.__map_contexts__.append(mc)

    def parseVolumetricMask( self, xml, mc ):
        # <volumetric_mask filename="img/places_mask.png" />
        assert xml.tagName == "volumetric_mask"
        filename_str = xml.getAttribute("filename")
        img = cv2.imread(self.__cwd__ + '/' + filename_str, cv2.IMREAD_GRAYSCALE)
        mc.setVolumetricMask(img)

    def parseVolumetricPlace( self, xml, mc ):
        # <volumetric_place id="warsztat" name="warsztat" filename="img/warsztat.png" />
        assert xml.tagName == "volumetric_place"
        id_str = xml.getAttribute("id")
        name_str = xml.getAttribute("name")
        filename_str = xml.getAttribute("filename")

        img = cv2.imread(self.__cwd__ + '/' + filename_str, cv2.IMREAD_GRAYSCALE)
        mc.addVolumetricPlace(id_str, name_str, img)

    def parsePointPlace( self, xml, mc ):
        # <point_place id="fotel" name="fotel" position="-0.64 0.34" front_vec="0 -1" />
        assert xml.tagName == "point_place"
        id_str = xml.getAttribute("id")
        name_str = xml.getAttribute("name")
        position_str = xml.getAttribute("position")
        front_vec_str = xml.getAttribute("front_vec")

        pos = parsePointStr( position_str )
        vec_n = parsePointStr( front_vec_str )
        mc.addPointPlace(id_str, name_str, pos, vec_n)

    def parseMapTransform(self, xml):
        # <map_transform map1="sim" map2="real" translation="-1.87296056747 -0.53914129734" rotation="-0.0821140241595" />
        assert xml.tagName == "map_transform"

        map1_str = xml.getAttribute("map1")
        map2_str = xml.getAttribute("map2")
        translation_str = xml.getAttribute("translation")
        rotation_str = xml.getAttribute("rotation")

        transl = parsePointStr( translation_str )
        rot = float(rotation_str)

        self.__map_transforms__.append( (map1_str, map2_str, transl, rot) )

    def getMapsTransform(self, map1, map2):
        for mt in self.__map_transforms__:
            transl = mt[2]
            rot = mt[3]
            if mt[0] == map1 and mt[1] == map2:
                return transl, rot
            elif mt[0] == map2 and mt[1] == map1:
                fr = PyKDL.Frame( PyKDL.Rotation.RotZ(mt[3]), PyKDL.Vector(transl[0], transl[1]) )
                fr_inv = fr.Inverse()
                return (fr_inv.p.x(), fr_inv.p.y()), -rot
        raise Exception('No such transform is available: "' + map1 + '", "' + map2 + '"')

    def transformImage(self, img, tf):
        M_rot = cv2.getRotationMatrix2D((img.shape[1]/2,img.shape[0]/2), rad2deg(tf[1]),1)
        M_trans = np.zeros( (2,3) )
        M_trans[0,0] = 1
        M_trans[1,1] = 1
        M_trans[0,2] = tf[0][0]
        M_trans[1,2] = tf[0][1]
        tf_img = cv2.warpAffine(img.astype(float),M_rot,(img.shape[1],img.shape[0]))
        tf_img = cv2.warpAffine(tf_img,M_trans,(img.shape[1],img.shape[0]))
        tf_img[tf_img>0.5] = 255
        tf_img[tf_img<1] = 0
        return tf_img

    def transformPlace(self, pl, tf):
        if pl.getType() == 'volumetric':
            img = pl.getImage()
            tf_img = self.transformImage(img, tf)
            tf_pl = VolumetricPlace( pl.getId(), pl.getName(), tf_img.astype(int))
        elif pl.getType() == 'point':
            fr = PyKDL.Frame( PyKDL.Rotation.RotZ(tf[1]), PyKDL.Vector(tf[0][0], tf[0][1], 0) )
            tf_pt = fr * PyKDL.Vector(pl.getPt()[0], pl.getPt()[1], 0)
            tf_pt = (tf_pt.x(), tf_pt.y())
            tf_n = fr.M * PyKDL.Vector(pl.getN()[0], pl.getN()[1], 0)
            tf_n = (tf_n.x(), tf_n.y())
            tf_pl = PointPlace( pl.getId(), pl.getName(), tf_pt, tf_n)
        else:
            raise Exception('Unknown place type: "' + pl.getType())
        return tf_pl

    def getPlaceById(self, pl_id, dest_map_context_name):
        for mc in self.__map_contexts__:
            pl = mc.getPlaceById( pl_id )
            if pl is None:
                continue
            if mc.getName() == dest_map_context_name:
                return pl
            else:
                tf = self.getMapsTransform(mc.getName(), dest_map_context_name)
                return self.transformPlace(pl, tf)
        raise Exception( 'Place id not found: "' + pl_id + '"')

    def getPlaceByName(self, name, dest_map_context_name):
        for mc in self.__map_contexts__:
            pl = mc.getPlaceByName( name )
            if pl is None:
                continue
            if mc.getName() == dest_map_context_name:
                return pl
            else:
                tf = self.getMapsTransform(mc.getName(), dest_map_context_name)
                return self.transformPlace(pl, tf)
        raise Exception( 'Place name not found: "' + name + '"')

    def getMask(self, dest_map_context_name):
        for mc in self.__map_contexts__:
            img = mc.getVolumetricMask()
            if img is None:
                continue
            if mc.getName() == dest_map_context_name:
                return img
            else:
                tf = self.getMapsTransform(mc.getName(), dest_map_context_name)
                return self.transformImage(img, tf)
        raise Exception( 'Volumetric mask not found')

    def getMapContext(self, mc_name):
        for mc in self.__map_contexts__:
            if mc.getName() == mc_name:
                return mc
        raise Exception('Map context not found: "' + mc_name + '"')
