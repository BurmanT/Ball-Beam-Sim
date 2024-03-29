#! /usr/bin/env python

import math

class InertialCalculator(object):

    def __init__(self):
          print("URDF Inertial Calculator Initialised...")
  
    def start_ask_loop(self):
  
        selection = "START"

        while selection.upper() != "Q":     # accept 'q' as well
            print("#############################")
            print("Select Geometry to Calculate:")
            print ("[1]Box width(w)*depth(d)*height(h)")
            print ("[2]Sphere radius(r)")
            print ("[3]Cylinder radius(r)*height(h)")
            print ("[Q]END program")
            selection = raw_input(">>")
            self.select_action(selection)

        print("URDF Inertial Calculator Quit...Thank you")

    def select_action(self, selection):
        if selection == "1":
            mass = float(raw_input("mass>>"))
            width = float(raw_input("width>>"))
            depth = float(raw_input("depth>>"))
            height = float(raw_input("height>>"))
            self.calculate_box_inertia(m=mass, w=width, d=depth, h=height)
        elif selection == "2":
            mass = float(raw_input("mass>>"))
            radius = float(raw_input("radius>>"))
            self.calculate_sphere_inertia(m=mass, r=radius)
        elif selection == "3":
            mass = float(raw_input("mass>>"))
            radius = float(raw_input("radius>>"))
            height = float(raw_input("height>>"))
            self.calculate_cylinder_inertia(m=mass, r=radius, h=height)
        elif selection.upper() == "Q": # accept 'q' as well
            print("Selected Quit")
        else:
            print("Usage: Select one of the give options")


    def calculate_box_inertia(self, m, w, d, h):
        Iw = (m/12.0)*(pow(d,2)+pow(h,2))
        Id = (m / 12.0) * (pow(w, 2) + pow(h, 2))
        Ih = (m / 12.0) * (pow(w, 2) + pow(d, 2))
        print('BOX w*d*h, need to define which axes are which for ixx, iyy, and izz')
        print ('Iw="' + str(Iw) + '" Id="'+ str(Id) + '" Ih="' +
            str(Ih) + '" ixy="0.0" ixz="0.0" iyz="0.0"')

    def calculate_sphere_inertia(self, m, r):
        I = (2*m*pow(r,2))/5.0
        print ('SPHERE ixx="' + str(I) + '" ixy="0.0" ixz="0.0" iyy="' 
                + str(I) + '" iyz="0.0" izz="' + str(I) + '"')

    def calculate_cylinder_inertia(self, m, r, h):
        Ix = (m/12.0)*(3*pow(r,2)+pow(h,2))
        Iy = Ix
        Iz = (m*pow(r,2))/2.0
        print ('Cylinder ixx="' + str(Ix) + '" ixy="0.0" ixz="0.0" iyy="' 
                + str(Iy) + '" iyz="0.0" izz="' + str(Iz) + '"')

if __name__ == "__main__":
    inertial_object = InertialCalculator()
    m = 0.125
    w = 0.01
    d = 0.1
    h = 0.06
    print(inertial_object.calculate_box_inertia(m,w,d,h))

    #inertial_object.start_ask_loop()