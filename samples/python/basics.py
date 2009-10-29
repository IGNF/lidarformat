#!/usr/bin/python
# -*- coding: utf8 -*-

import sys
import os
sys.path.append(os.getcwd() + '/../../')
import libLidarFormat as lf



f = lf.LidarFile("../../data/testAscii.xml")
print f.get_meta_data()

container = lf.LidarDataContainer()
f.load_data(container)

#def allways_true(echo):
#    return True
#
#lf.print_if_echo(container, allways_true)

print container.header()
for i in container:
    print i

container.del_attribute("x")

print container.header()
for i in container:
    print i


