#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
#----------------------------------------------------------------------
#--- Python Make file for application test
#--- automatically generated by goil on Www Mmm dd hh:mm:ss yyyy
#--- from root OIL file icm.oil
#---
#--- Compiling a Trampoline application is done in 2 stages
#--- 1 - From the OIL file, a set of files is generated as long as
#---     build options. Among these files are the build.py and make.py 
#---     files. make.py contains compilation rules when the OIL file
#---     and the included OIL files are modified. make.py is only
#---     written once when the goil command is typed.
#--- 2 - From all the C/assembly files (OS, Application, librairies,
#---     configuration, ...), the objects and the executable are
#---     produced. This is done by the build.py file.
#---     build.py is generated each time goil is called by make.py
#----------------------------------------------------------------------
from __future__ import print_function
import sys, os, subprocess, atexit, string

#--- Add the location of makefile.py to the import path
sys.path.append("../../../..//make")

#--- Import the makefile system in python
try:
	import makefile, projfile
except ImportError:
	#it defines inline the red color, as other are defined in the makefile.py
	print("\033[91mThe OS->BUILD->TRAMPOLINE_BASE_PATH key is not set correctly in the oil file icm.oil.")
	print("Set the correct Trampoline installation path in the oil file and run goil again:\033[0m")
	print("goil -t=avr/arduino/nano --templates=../../../../goil/templates/ icm.oil")
	sys.exit(1)

#--- To work with relative paths
scriptDir = os.path.dirname (os.path.abspath (sys.argv[0]))
os.chdir (scriptDir)

#--- Get goal as first argument
askedGoal = "all"
if len (sys.argv) > 1 :
  askedGoal = sys.argv [1]

if askedGoal == "all" or askedGoal == "clean" :
  goal = askedGoal
else :
  goal = "all"
  
#--- Get max parallel jobs as second argument
maxParallelJobs = 0 # 0 means use host processor count
if len (sys.argv) > 2 :
  maxParallelJobs = int (sys.argv [2])

#--- Instanciate a new makefile object
make = makefile.Make(goal)

#----------------------------------------------------------------------
#--- Various variables used after
#----------------------------------------------------------------------
oilFile = "icm.oil"
oilFileDep = "build/" + oilFile + ".dep"
oilCompiler = "goil"
oilFlags = ["-t=avr/arduino/nano", "--templates=../../../../goil/templates/"]
trampoline_base_path = "../../../..//"

#----------------------------------------------------------------------
#--- Build the source files list
#----------------------------------------------------------------------
oilSourceList = []
sourceList = []

#--- Add generated files
sourceList.append("icm/tpl_app_config.c")

sourceList.append("icm/tpl_dispatch_table.c")
sourceList.append("icm/tpl_invoque.S")
sourceList.append("icm/tpl_interrupts.c")

rule = makefile.Rule (sourceList, "Compiling OIL file " + oilFile)
rule.deleteTargetFileOnClean()
rule.mDeleteTargetOnError = True
rule.mDependences.append(oilFile)
rule.enterSecondaryDependanceFile(oilFileDep, make)
rule.mCommand.append("goil")
rule.mCommand += oilFlags
rule.mCommand.append(oilFile)

make.addRule (rule)
make.addGoal("all", sourceList, "Building all")

if goal == "all" or goal == "clean" :
  make.runGoal(maxParallelJobs, maxParallelJobs == 1)

#----------------------------------------------------------------------
#--- Call the seconde stage of make
#----------------------------------------------------------------------

def cleanup():
  if childProcess.poll () == None :
    childProcess.kill ()

#--- Register a function for killing subprocess
atexit.register (cleanup)

#--- Get script absolute path
scriptDir = os.path.dirname (os.path.abspath (sys.argv [0]))
os.chdir (scriptDir)

#--- Launch build.py
# is there the 'python' executable?
import shutil
cmd = shutil.which('python')
if not cmd:
  cmd = shutil.which('python3')
childProcess = subprocess.Popen ([cmd, "build.py", askedGoal, str(maxParallelJobs)])

#--- Wait for subprocess termination
if childProcess.poll () == None :
  childProcess.wait ()
if childProcess.returncode != 0 :
  sys.exit (childProcess.returncode)

#----------------------------------------------------------------------
