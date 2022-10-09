###########################################################################
## Makefile generated for MATLAB file/project 'getRegressionMatrix'. 
## 
## Makefile     : getRegressionMatrix_rtw.mk
## Generated on : Thu Sep 01 00:44:03 2022
## MATLAB Coder version: 4.1 (R2018b)
## 
## Build Info:
## 
## Final product: .getRegressionMatrix.a
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPUTER                Computer type. See the MATLAB "computer" command.

PRODUCT_NAME              = getRegressionMatrix
MAKEFILE                  = getRegressionMatrix_rtw.mk
COMPUTER                  = PCWIN64
MATLAB_ROOT               = C:\PROGRA~1\MATLAB\R2018b
MATLAB_BIN                = C:\PROGRA~1\MATLAB\R2018b\bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)glnxa64
MASTER_ANCHOR_DIR         = 
START_DIR                 = C:\software\YZY\px-identification-matlab\LegIdentification\Identification\codegen\lib\getRegressionMatrix
ARCH                      = win64
RELATIVE_PATH_TO_ANCHOR   = .
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          QuestaSim\Cadence linux v1.0 | gmake (64-bit Linux)
# Supported Version(s):    1.0
# ToolchainInfo Version:   R2018b
# Specification Revision:  1.0
# 
TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: C Compiler
CC = gcc

# Linker: Linker
LD = gcc

# C++ Compiler: GNU C++ Compiler
CPP = g++

# C++ Linker: GNU C++ Linker
CPP_LD = g++

# Archiver: GNU Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE = echo "### Successfully generated all binary outputs."


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              =
C_OUTPUT_FLAG       =
LDDEBUG             =
OUTPUT_FLAG         =
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  =
ECHO                =
MV                  =
RUN                 =

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------

ARFLAGS              =
CFLAGS               =
CPPFLAGS             =
CPP_LDFLAGS          =
CPP_SHAREDLIB_LDFLAGS  =
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              =
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           =  $(MAKEFILE)
SHAREDLIB_LDFLAGS    =

#--------------------
# File extensions
#--------------------

H_EXT               = .h
OBJ_EXT             = .o
C_EXT               = .c
EXE_EXT             =
SHAREDLIB_EXT       = .so
HPP_EXT             = .hpp
OBJ_EXT             = .o
CPP_EXT             = .cpp
EXE_EXT             =
SHAREDLIB_EXT       = .so
STATICLIB_EXT       = .a
MEX_EXT             = .mexa64
MAKE_EXT            = .do


###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = .getRegressionMatrix.a
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -ccflags$(START_DIR) -ccflagsC:\software\YZY\PX-IDE~1\LEGIDE~1\IDENTI~1 -ccflags$(MATLAB_ROOT)\extern\include -ccflags$(MATLAB_ROOT)\simulink\include -ccflags$(MATLAB_ROOT)\rtw\c\src -ccflags$(MATLAB_ROOT)\rtw\c\src\ext_mode\common -ccflags$(MATLAB_ROOT)\rtw\c\ert

INCLUDES = 

###########################################################################
## DEFINES
###########################################################################

DEFINES_STANDARD = MODEL=getRegressionMatrix HAVESTDIO USE_RTMODEL

DEFINES = 

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)getRegressionMatrix_initialize.cpp $(START_DIR)getRegressionMatrix_terminate.cpp $(START_DIR)getRegressionMatrix.cpp $(START_DIR)getYmatrix_leg1.cpp $(START_DIR)getYmatrix_leg2.cpp $(START_DIR)getYmatrix_leg3.cpp $(START_DIR)getYmatrix_leg4.cpp

ALL_SRCS = 

###########################################################################
## OBJECTS
###########################################################################

OBJS = getRegressionMatrix_initialize.o getRegressionMatrix_terminate.o getRegressionMatrix.o getYmatrix_leg1.o getYmatrix_leg2.o getYmatrix_leg3.o getYmatrix_leg4.o

ALL_OBJS = 

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += 

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += 

###########################################################################
## INLINED COMMANDS
###########################################################################

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	 "### Successfully generated all binary outputs."


build : prebuild 


prebuild : 


download : build


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

 :  
	 "### Creating static library "" ..."
	    $(subst ,/,$(subst ,/,))
	 "### Created: "


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	   "$@" $(subst ,/,"$<")


%.o : %.cpp
	  -o "$@" $(subst ,/,"$<")


%.o : C:softwareYZYpx-identification-matlabLegIdentificationIdentification%.c
	   "$@" $(subst ,/,"$<")


%.o : C:softwareYZYpx-identification-matlabLegIdentificationIdentification%.cpp
	  -o "$@" $(subst ,/,"$<")


%.o : $(START_DIR)%.c
	   "$@" $(subst ,/,"$<")


%.o : $(START_DIR)%.cpp
	  -o "$@" $(subst ,/,"$<")


%.o : $(MATLAB_ROOT)rtwcsrc%.c
	   "$@" $(subst ,/,"$<")


%.o : $(MATLAB_ROOT)rtwcsrc%.cpp
	  -o "$@" $(subst ,/,"$<")


###########################################################################
## DEPENDENCIES
###########################################################################

 : rtw_proj.tmw 


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	 "### PRODUCT = "
	 "### PRODUCT_TYPE = "
	 "### BUILD_TYPE = "
	 "### INCLUDES = "
	 "### DEFINES = "
	 "### ALL_SRCS = "
	 "### ALL_OBJS = "
	 "### LIBS = "
	 "### MODELREF_LIBS = "
	 "### SYSTEM_LIBS = "
	 "### TOOLCHAIN_LIBS = "
	 "### CFLAGS = "
	 "### LDFLAGS = "
	 "### SHAREDLIB_LDFLAGS = "
	 "### CPPFLAGS = "
	 "### CPP_LDFLAGS = "
	 "### CPP_SHAREDLIB_LDFLAGS = "
	 "### ARFLAGS = "
	 "### MEX_CFLAGS = "
	 "### MEX_CPPFLAGS = "
	 "### MEX_LDFLAGS = "
	 "### MEX_CPPLDFLAGS = "
	 "### DOWNLOAD_FLAGS = "
	 "### EXECUTE_FLAGS = "
	 "### MAKE_FLAGS = "


clean : 
	 "### Deleting all derived files..."
	 $(subst /,\,)
	 $(subst /,\,)
	 "### Deleted all derived files."


