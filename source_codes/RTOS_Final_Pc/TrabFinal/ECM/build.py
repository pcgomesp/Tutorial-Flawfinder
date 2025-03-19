#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
#----------------------------------------------------------------------
#--- Python Build file for application test
#--- automatically generated by goil on Www Mmm dd hh:mm:ss yyyy
#--- from root OIL file ecm.oil
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

import sys, os, subprocess, string
from string import Template

#--- Add some function for Python3 support

if sys.version_info[0] >=3 :
    def bytesdecode(obj) :
        return obj.decode(sys.stdout.encoding)
else :
    def bytesdecode(obj) :
        return obj

#--- Add the location of makefile.py to the import path
sys.path.append("../../../..//make")

#--- Import the makefile system in python
import makefile, projfile

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
compiler = r"avr-gcc"
cppCompiler = r"avr-g++"
linker = r"avr-gcc"
assembler = r"avr-gcc"
autosar = False
autosar_sc = 0
autosar_osapplications = False
with_ioc = False
with_com = False
scheduler = "osek"
trampoline_base_path = "../../../..//"
cflags = []
cppflags = []
ldflags = []
asflags = []
# flags that should appear at the beginning of the command.
precflags = []
precppflags = []
preldflags = []
preasflags = []
includeDirs = []

cppflags += "-std=c++11".split()
cflags += "-DARDUINO=10809 -Wno-unused-but-set-variable".split()
cppflags += "-DARDUINO=10809 -Wno-unused-but-set-variable".split()
cflags += "-Os -Wall -DF_CPU=16000000 -mmcu=atmega328p".split()
cppflags += "-Os -Wall -DF_CPU=16000000 -mmcu=atmega328p".split()
cflags += "-ffunction-sections".split()
cppflags += "-ffunction-sections".split()
cflags += "-fdata-sections".split()
cppflags += "-fdata-sections".split()
includeDirs += ["-I","../../../..//machines/avr/arduino/cores/arduino"]
includeDirs += ["-I","../../../..//machines/avr/arduino/variants/eightanaloginputs"]
includeDirs += ["-I","../../../..//machines/avr/arduino/"]
includeDirs += ["-I","../../../..//machines/avr"]
includeDirs += ["-I","../../../..//machines/avr/arduino/libraries/mcp_can/src"]
includeDirs += ["-I","../../../..//machines/avr/arduino/cores/arduino"]
includeDirs += ["-I","../../../..//machines/avr/arduino/cores/arduino"]
includeDirs += ["-I","../../../..//machines/avr/arduino/libraries/SPI/src"]
if with_ioc:
  includeDirs += ["-I", "../../../..//ioc"]
includeDirs += ["-I", "../../../..//com"]
includeDirs += ["-I", "../../../..//os"]
includeDirs += ["-I", "../../../..//debug"]
includeDirs += ["-I", "ecm"]
cflags   += includeDirs
cppflags += includeDirs
asflags  += includeDirs

ldflags += "-DF_CPU=16000000 -mmcu=atmega328p".split()
ldflags += "-Wl,--gc-sections".split()
preldflags += "-lm".split()
asflags += "-DF_CPU=16000000 -mmcu=atmega328p".split()
preasflags += "-x assembler-with-cpp".split()

#----------------------------------------------------------------------
#--- Try to detect automatically the location of libc and libgcc
#--- This supposes the compiler has a (something)-gcc name
#--- for non posix target
#----------------------------------------------------------------------
if "gcc" in compiler:
  compilerFullPath = makefile.find_executable(compiler)
  if compilerFullPath != None:
    #--- Extract the (something) from the compiler executable name
    compilerParts = compiler.split('-')
    compilerParts.pop()
    compilerPrefix = "-".join(compilerParts)
    #--- Get the full path of the compiler
    compilerToolChainPath = os.path.dirname(os.path.dirname(compilerFullPath))
    #--- Get the version of the compiler, [:-1] removes the carriage return
    compilerVersion = subprocess.check_output([compiler, '-dumpversion'])[:-1]
    #--- Build the libc and libgcc paths
    procLibc  =subprocess.Popen([compilerFullPath,"-print-file-name=libc.a"]+cflags,stdout=subprocess.PIPE)
    procLibgcc=subprocess.Popen([compilerFullPath,"-print-libgcc-file-name"]+cflags,stdout=subprocess.PIPE)
    procLibc.wait()
    procLibgcc.wait()
    libcPath = os.path.dirname(procLibc.stdout.readline().strip())
    libgccPath = os.path.dirname(procLibgcc.stdout.readline().strip())
    #--- Add both to linker flags
    ldflags += ['-L' + bytesdecode(libcPath), '-lc']
    ldflags += ['-L' + bytesdecode(libgccPath), '-lgcc']

#----------------------------------------------------------------------
#--- Build the source files list
#----------------------------------------------------------------------
cSourceList = []
cppSourceList = []
sSourceList = []
oilSourceList = []

#--- OIL file
oilSourceList.append("ecm.oil")

#--- Kernel files 
cSourceList.append(projfile.ProjectFile("os/tpl_os_kernel.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("os/tpl_os_timeobj_kernel.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("os/tpl_os_action.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("os/tpl_os_error.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("os/tpl_os_os_kernel.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("os/tpl_os_os.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("os/tpl_os_interrupt_kernel.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("os/tpl_os_task_kernel.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("os/tpl_os_resource_kernel.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("os/tpl_os_alarm_kernel.c", trampoline_base_path))

#--- Add C files of the application
cSourceList.append(projfile.ProjectFile("ecm.cpp"))

#--- Add generated files
cSourceList.append(projfile.ProjectFile("ecm/tpl_app_config.c"))

cSourceList.append(projfile.ProjectFile("ecm/tpl_dispatch_table.c"))
cSourceList.append(projfile.ProjectFile("ecm/tpl_invoque.S"))
cSourceList.append(projfile.ProjectFile("ecm/tpl_interrupts.c"))
cSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/wiring_digital.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/wiring.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/hooks.c", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/wiring_analog.c", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/main.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/WMath.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/WString.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/WInterrupts.c", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/abi.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/new.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino//tpl_trace.cpp", trampoline_base_path))
cSourceList.append(projfile.ProjectFile("machines/avr/tpl_machine.c", trampoline_base_path))
sSourceList.append(projfile.ProjectFile("machines/avr/avr_switch_context.s", trampoline_base_path))
sSourceList.append(projfile.ProjectFile("machines/avr/tpl_dispatch.S", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/libraries/mcp_can/src/mcp_can.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/Print.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/Stream.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/HardwareSerial0.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/HardwareSerial1.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/HardwareSerial2.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/HardwareSerial3.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/cores/arduino/HardwareSerial.cpp", trampoline_base_path))
cppSourceList.append(projfile.ProjectFile("machines/avr/arduino/libraries/SPI/src/SPI.cpp", trampoline_base_path))
#----------------------------------------------------------------------
#--- Build the object list and the compiler dependancies
#----------------------------------------------------------------------
objectList = []

for sourceFile in cSourceList:
  source = sourceFile.src()
  object = sourceFile.obj("build")
  depObject = sourceFile.dep("build")
  objectList.append(object)
  rule = makefile.Rule([object], "Compiling " + source)
  rule.deleteTargetDirectoryOnClean()
  rule.mDependences.append(source)
  rule.mCommand.append(compiler)
  rule.mCommand += precflags
  rule.mCommand += ["-c", source]
  rule.mCommand += ["-o", object]
  rule.mCommand += ["-MD", "-MP", "-MF", depObject]
  rule.mCommand += cflags
  rule.enterSecondaryDependanceFile (depObject, make)
  make.addRule(rule)

for sourceFile in cppSourceList:
  source = sourceFile.src()
  object = sourceFile.obj("build")
  depObject = sourceFile.dep("build")
  objectList.append(object)
  rule = makefile.Rule([object], "Compiling " + source)
  rule.deleteTargetDirectoryOnClean()
  rule.mDependences.append(source)
  rule.mCommand.append(cppCompiler)
  rule.mCommand += precppflags
  rule.mCommand += ["-c", source]
  rule.mCommand += ["-o", object]
  rule.mCommand += ["-MD", "-MP", "-MF", depObject]
  rule.mCommand += cppflags
  rule.enterSecondaryDependanceFile (depObject, make)
  make.addRule(rule)

for sourceFile in sSourceList:
  source = sourceFile.src()
  object = sourceFile.obj("build")
  objectList.append(object)
  rule = makefile.Rule([object], "Assembling " + source)
  rule.mDependences.append(source)
  rule.mCommand.append(assembler)
  rule.mCommand += preasflags
  rule.mCommand += ["-c", source]
  rule.mCommand += ["-o", object]
  rule.mCommand += asflags
  make.addRule(rule)

product = "ecm_image"
rule = makefile.Rule ([product], "Linking " + product)
rule.deleteTargetFileOnClean()
rule.mDeleteTargetOnError = True
rule.mDependences += objectList
rule.mCommand += [linker]
rule.mCommand += ["-o", product]
rule.mCommand += preldflags
rule.mCommand += objectList
rule.mCommand += ldflags
if True:
  postCommand = makefile.PostCommand("Generating binary ecm_image.hex from ecm_image")
  postCommand.mCommand.append(r"avr-objcopy")
  postCommand.mCommand += "-O ihex".split()
  postCommand.mCommand.append("ecm_image")
  postCommand.mCommand.append("ecm_image.hex")
  rule.mPostCommands.append(postCommand)

make.addRule (rule)

make.addGoal("all", [product], "Building all")
make.addGoal("compile", objectList, "Compile source files")


make.runGoal(maxParallelJobs, maxParallelJobs == 1)

postVariableMapping = dict(
  MACHINE_PATH='../../../..//machines',
  ARCH_PATH='../../../..//machines/avr',
  BOARD_PATH='../../../..//machines/avr/arduino/nano',
  TARGET='avr/arduino/nano'
)


#----------------------------------------------------------------------
#--- post commands
#----------------------------------------------------------------------
if make.errorCount() == 0:
  if askedGoal == "flash" :
    commandLine = "avrdude -c arduino -p m328p "
    if os.environ.get('AVRDUDE_PORT') != None:
      commandLine += "-P" + os.environ['AVRDUDE_PORT'] + " "
    else:
      print("Cannot flash, environment variable AVRDUDE_PORT is not defined")
      exit(1)
    commandLine += "-Uflash:w:"
    commandLine += "ecm_image.hex "
    commandLineTemplate = Template(commandLine)
    commandLine = commandLineTemplate.safe_substitute(postVariableMapping)
    if makefile.find_executable("avrdude") != None:
      print("\033[1m\033[95mFlashing ecm_image on avr/arduino/nano\033[0m")
      subprocess.call(commandLine, shell=True)
    else:
      print("Command 'avrdude' not in PATH")


make.printErrorCountAndExitOnError()

#----------------------------------------------------------------------
#-- End of build.py
#----------------------------------------------------------------------
