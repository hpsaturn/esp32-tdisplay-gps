# build.py
# pre-build script, setting up build environment and fetch hal file for user's board

import sys
import os
import os.path
import requests
import json
from os.path import basename
from platformio import util
from SCons.Script import DefaultEnvironment

try:
    import configparser
except ImportError:
    import ConfigParser as configparser

# get platformio environment variables
env = DefaultEnvironment()
config = configparser.ConfigParser()
config.read("platformio.ini")

# get platformio source path
srcdir = env.get("PROJECTSRC_DIR")
flavor = env.get("PIOENV")
revision = config.get("common","revision")
version = config.get("common", "version")
target = config.get("common", "target")

# print ("environment:")
# print (env.Dump())

# get runtime credentials and put them to compiler directive
env.Append(BUILD_FLAGS=[
    u'-DREVISION=' + revision + '',
    u'-DVERSION=\\"' + version + '\\"',
    u'-DFLAVOR=\\"' + flavor + '\\"',
    u'-DTARGET=\\"' + target + '\\"',
    u'-D'+ flavor + '=1',
    u'-I \"' + srcdir + '\"'
    ])


