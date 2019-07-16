##########################
# common definitions. For non-UNC sites, uncomment one of the lines
# that defines hw_os for the machine you are on in the section just
# below. Then, the code should compile in your environment.
#
HW_OS := pc_linux
##########################

##########################
# Mac OS X-specific options. If HW_OS is powerpc_macosx or universal_macosx,
# uncomment one line below to choose the minimum targeted OS X version and
# corresponding SDK. If none of the lines below is commented out, 10.5 will
# be the minimum version.
##########################
#MAC_OS_MIN_VERSION := 10.4
#MAC_OS_MIN_VERSION := 10.5
#MAC_OS_MIN_VERSION := 10.6


INSTALL_DIR := /usr/local
BIN_DIR := $(INSTALL_DIR)/bin
INCLUDE_DIR := $(INSTALL_DIR)/include
LIB_DIR := $(INSTALL_DIR)/lib

MV = /bin/mv
MVF = $(MV) -f

RM = /bin/rm
RMF = $(RM) -f


# Which C++ compiler to use.  Default is g++, but some don't use this.
#
# IF YOU CHANGE THESE, document either here or in the header comment
# why.  Multiple contradictory changes have been made recently.


CC := g++
AR := ar ruv
# need default 'ranlib' to be touch for platforms that don't use it,
# otherwise make fails.
RANLIB := touch

        CC := gcc
        RANLIB := ranlib

##########################
# directories
#

# subdirectory for make
ifeq ($(FORCE_GPP),1)
OBJECT_DIR	 := $(HW_OS)$(OBJECT_DIR_SUFFIX)/g++
SOBJECT_DIR      := $(HW_OS)$(OBJECT_DIR_SUFFIX)/g++/server
AOBJECT_DIR      := $(HW_OS)$(OBJECT_DIR_SUFFIX)/atmellib
else
UNQUAL_OBJECT_DIR := $(HW_OS)$(OBJECT_DIR_SUFFIX)
UNQUAL_SOBJECT_DIR := $(HW_OS)$(OBJECT_DIR_SUFFIX)/server
UNQUAL_AOBJECT_DIR := $(HW_OS)$(OBJECT_DIR_SUFFIX)/atmellib
OBJECT_DIR	 := $(HW_OS)$(OBJECT_DIR_SUFFIX)
SOBJECT_DIR      := $(HW_OS)$(OBJECT_DIR_SUFFIX)/server
AOBJECT_DIR      := $(HW_OS)$(OBJECT_DIR_SUFFIX)/atmellib
endif

# directories that we can do an rm -f on because they only contain
# object files and executables
SAFE_KNOWN_ARCHITECTURES :=	\
	hp700_hpux/* \
	hp700_hpux10/* \
	mips_ultrix/* \
	pc_linux/* \
	sgi_irix.32/* \
	sgi_irix.n32/* \
	sparc_solaris/* \
	sparc_solaris_64/* \
	sparc_sunos/* \
	pc_cygwin/* \
	powerpc_aix/* \
	pc_linux_arm/* \
	powerpc_macosx/* \
	universal_macosx/* \
	pc_linux64/* \
	pc_linux_ia64/*

CLIENT_SKA = $(patsubst %,client_src/%,$(SAFE_KNOWN_ARCHITECTURES))
SERVER_SKA = $(patsubst %,server_src/%,$(SAFE_KNOWN_ARCHITECTURES))

##########################
# Include flags
#

#SYS_INCLUDE := -I/usr/local/contrib/include -I/usr/local/contrib/mod/include
SYS_INCLUDE :=

	# The following is for the InterSense and Freespace libraries.
	SYS_INCLUDE := -DUNIX -DLINUX -I../libfreespace/include -I./submodules/hidapi/hidapi -I/usr/include/libusb-1.0

# On the PC, place quatlib in the directory ./quat.  No actual system
# includes should be needed.

INCLUDE_FLAGS := -I. $(SYS_INCLUDE) -I./quat -I../quat -I./atmellib



##########################
# Load flags
#

#LOAD_FLAGS := -L./$(HW_OS)$(OBJECT_DIR_SUFFIX) -L/usr/local/lib \
#		-L/usr/local/contrib/unmod/lib -L/usr/local/contrib/mod/lib -g
LOAD_FLAGS := -L./$(HW_OS)$(OBJECT_DIR_SUFFIX) -L/usr/local/lib \
		-L/usr/local/contrib/unmod/lib -L/usr/local/contrib/mod/lib $(DEBUG_FLAGS) $(LDFLAGS)

	LOAD_FLAGS := $(LOAD_FLAGS) -L/usr/X11R6/lib


##########################
# Libraries
#

          ARCH_LIBS := -lbsd -ldl

LIBS := -lquat -lsdi $(TCL_LIBS) -lXext -lX11 $(ARCH_LIBS) -lm

#
# Defines for the compilation, CFLAGS
#

#CFLAGS		 := $(INCLUDE_FLAGS) -g
override CFLAGS		 := $(INCLUDE_FLAGS) $(DEBUG_FLAGS) $(CFLAGS)
override CXXFLAGS     := $(INCLUDE_FLAGS) $(DEBUG_FLAGS) $(CXXFLAGS)

# If we're building for sgi_irix, we need both g++ and non-g++ versions,
# unless we're building for one of the weird ABIs, which are only supported
# by the native compiler.


all:	client 
#server

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

.PHONY:	client_g++
client_g++:
	$(MAKE) FORCE_GPP=1 $(UNQUAL_OBJECT_DIR)/g++/libvrpn.a
	$(MV) $(UNQUAL_OBJECT_DIR)/g++/libvrpn.a $(UNQUAL_OBJECT_DIR)/libvrpn_g++.a

.PHONY:	server_g++
server_g++:
	$(MAKE) FORCE_GPP=1 $(UNQUAL_OBJECT_DIR)/g++/libvrpnserver.a
	$(MV) $(UNQUAL_OBJECT_DIR)/g++/libvrpnserver.a $(UNQUAL_OBJECT_DIR)/libvrpnserver_g++.a

.PHONY:	client
client: $(OBJECT_DIR) $(OBJECT_DIR)/libvrpn.a

.PHONY:	server
server: $(SOBJECT_DIR)
	$(MAKE) $(OBJECT_DIR)/libvrpnserver.a

.PHONY: atmellib
atmellib: $(AOBJECT_DIR)
	$(MAKE) $(OBJECT_DIR)/libvrpnatmel.a

$(OBJECT_DIR):
	-mkdir -p $(OBJECT_DIR)

$(SOBJECT_DIR):
	-mkdir -p $(SOBJECT_DIR)

$(AOBJECT_DIR):
	-mkdir -p $(AOBJECT_DIR)

#############################################################################
#
# implicit rule for all .c files
#
.SUFFIXES:	.c .C .o .a

.c.o:
	$(CC) -c $(CFLAGS) $<
.C.o:
	$(CC) -c $(CXXFLAGS) $<

# Build objects from .c files
$(OBJECT_DIR)/%.o: %.c $(LIB_INCLUDES) $(MAKEFILE)
	@[ -d $(OBJECT_DIR) ] || mkdir -p $(OBJECT_DIR)
	$(CC) $(CFLAGS) -DVRPN_CLIENT_ONLY -o $@ -c $<

# Build objects from .C files
$(OBJECT_DIR)/%.o: %.C $(LIB_INCLUDES) $(MAKEFILE)
	@[ -d $(OBJECT_DIR) ] || mkdir -p $(OBJECT_DIR)
	$(CC) $(CFLAGS) -DVRPN_CLIENT_ONLY -o $@ -c $<

# Build objects from .C files
$(SOBJECT_DIR)/%.o: %.C $(SLIB_INCLUDES) $(MAKEFILE)
	@[ -d $(SOBJECT_DIR) ] || mkdir -p $(SOBJECT_DIR)
	$(CC) $(CFLAGS) -o $@ -c $<

# Build objects from .C files
$(AOBJECT_DIR)/%.o: %.C $(ALIB_INCLUDES) $(MAKEFILE)
	@[ -d $(AOBJECT_DIR) ] || mkdir -p $(AOBJECT_DIR)
	$(CC) $(CFLAGS) -o $@ -c $<

# Special rule for vrpn_Local_HIDAPI.C, which must be build with
# the C compiler.
$(SOBJECT_DIR)/vrpn_Local_HIDAPI.o : vrpn_Local_HIDAPI.C $(SLIB_INCLUDES) $(MAKEFILE)
	@[ -d $(SOBJECT_DIR) ] || mkdir -p $(SOBJECT_DIR)
	$(CC) $(CFLAGS) -x c -o $@ -c $<

#
#
#############################################################################

#############################################################################
#
# library code
#
#############################################################################

# files to be compiled into the client library

LIB_FILES =  \
	vrpn_BaseClass.C \
	vrpn_Connection.C \
	vrpn_Shared.C \
	vrpn_SharedObject.C \
	vrpn_Text.C 
	#vrpn_LamportClock.C \

LIB_OBJECTS = $(patsubst %,$(OBJECT_DIR)/%,$(LIB_FILES:.C=.o))

LIB_INCLUDES = \
	vrpn_Connection.h \
	vrpn_Shared.h \
	vrpn_Text.h \
	vrpn_SharedObject.h \
	vrpn_LamportClock.h \
	vrpn_BaseClass.h 


$(LIB_OBJECTS):
$(OBJECT_DIR)/libvrpn.a: $(MAKEFILE) $(LIB_OBJECTS)
	$(AR) $(OBJECT_DIR)/libvrpn.a $(LIB_OBJECTS)
	-$(RANLIB) $(OBJECT_DIR)/libvrpn.a


ALIB_FILES = \
	atmellib/vrpn_atmellib_helper.C \
	atmellib/vrpn_atmellib_iobasic.C \
	atmellib/vrpn_atmellib_openclose.C \
	atmellib/vrpn_atmellib_register.C \
	atmellib/vrpn_atmellib_tester.C

ALIB_OBJECTS = $(patsubst %,$(AOBJECT_DIR)/../%,$(ALIB_FILES:.C=.o))

ALIB_INCLUDES =  \
	vrpn_atmellib.h \
	vrpn_atmellib_helper.h \
	vrpn_atmellib_errno.h

$(ALIB_OBJECTS):
$(OBJECT_DIR)/libvrpnatmel.a: $(MAKEFILE) $(ALIB_OBJECTS)
	$(AR) $(OBJECT_DIR)/libvrpnatmel.a $(ALIB_OBJECTS)
	-$(RANLIB) $(OBJECT_DIR)/libvrpnatmel.a

#############################################################################
#
# other stuff
#
#############################################################################

.PHONY:	clean
clean:
ifeq ($(HW_OS),)
		echo "Must specify HW_OS !"
else
	$(RMF) $(LIB_OBJECTS) $(OBJECT_DIR)/libvrpn.a \
               $(OBJECT_DIR)/libvrpn_g++.a \
               $(SLIB_OBJECTS) \
               $(ALIB_OBJECTS) \
               $(OBJECT_DIR)/libvrpnserver.a \
               $(OBJECT_DIR)/libvrpnatmel.a \
               $(OBJECT_DIR)/libvrpnserver_g++.a \
               $(OBJECT_DIR)/.depend \
               $(OBJECT_DIR)/.depend-old
ifneq (xxx$(FORCE_GPP),xxx1)
	@echo -----------------------------------------------------------------
	@echo -- Wart: type \"$(MAKE) clean_g++\" to clean up after g++
	@echo -- I don\'t do it automatically in case you don\'t have g++
	@echo -----------------------------------------------------------------
endif
#ifneq ($(CC), g++)
#	$(MAKE) FORCE_GPP=1 clean
#endif
endif

.PHONY:	clean
clean_g++:
	$(MAKE) FORCE_GPP=1 clean


# clobberall removes the object directory for EVERY architecture.
# One problem - the object directory for pc_win32 also contains files
# that must be saved.
# clobberall also axes left-over CVS cache files.

.PHONY:	clobberall
clobberall:	clobberwin32
	$(RMF) -r $(SAFE_KNOWN_ARCHITECTURES)
	$(RMF) -r $(CLIENT_SKA)
	$(RMF) -r $(SERVER_SKA)
	$(RMF) .#* server_src/.#* client_src/.#*

.PHONY:	clobberwin32
clobberwin32:
	$(RMF) -r pc_win32/DEBUG/*
	$(RMF) -r pc_win32/vrpn/Debug/*
	$(RMF) -r client_src/pc_win32/printvals/Debug/*
	$(RMF) -r server_src/pc_win32/vrpn_server/Debug/*

install: all
	-mkdir -p $(LIB_DIR)
	( cd $(LIB_DIR) ; rm -f libvrpn*.a )
	( cd $(OBJECT_DIR) ; cp *.a $(LIB_DIR) )
	-mkdir -p $(INCLUDE_DIR)
	cp vrpn*.h $(INCLUDE_DIR)

uninstall:
	( cd $(LIB_DIR) ; rm -f libvrpn*.a )
	( cd $(INCLUDE_DIR) ; rm -f vrpn*.h )

#############################################################################
#
# Dependencies
#
#   If it doesn't already exist, this makefile automatically creates
#   a dependency file called .depend.  Then it includes it so that
#   the build will know the dependency information.
#
#   to recreate a dependencies file, type  "make depend"
#   do this any time you add a file to the project,
#   or add/remove #include lines from a source file
#
#   if you are on an SGI and want g++ to make the dependency file,
#   then type:    gmake CC=g++ depend
#
#   if you don't want a dependency file, then remove .depend if it exists,
#   and type "touch .depend".  if it exists (and is empty), make will not
#   automatically create it or automatically update it (unless you type
#   make depend)
#

###############
### this way works better
###    you type "make depend" anytime you add a file or
###    add/remove #includes from a file
########

include $(OBJECT_DIR)/.depend

.PHONY: depend
depend:
	-$(MVF) $(OBJECT_DIR)/.depend $(OBJECT_DIR)/.depend-old
	$(MAKE) $(OBJECT_DIR)/.depend

$(OBJECT_DIR)/.depend:
	@echo ----------------------------------------------------------------
	@echo -- Making dependency file.  If you add files to the makefile,
	@echo -- or add/remove includes from a .h or .C file, then you should
	@echo -- remake the dependency file by typing \"$(MAKE) depend\"
	@echo ----------------------------------------------------------------
	-mkdir -p $(OBJECT_DIR)
ifeq ($(HW_OS),hp700_hpux10)
	@echo -- $(HW_OS): Using g++ since HP CC does not understand -M
	@echo -- if this causes an error, then delete .depend and type
	@echo -- \"touch .depend\" to create an empty file
	@echo ----------------------------------------------------------------
	$(SHELL) -ec 'g++ -MM $(CXXFLAGS) $(LIB_FILES) \
	    | sed '\''s/\(.*\.o[ ]*:[ ]*\)/$(OBJECT_DIR)\/\1/g'\'' > $(OBJECT_DIR)/.depend'
else
  ifeq ($(HW_OS),hp_flow_aCC)
	@echo -- $(HW_OS): Using g++ since HP aCC does not understand -M
	@echo -- if this causes an error, then delete .depend and type
	@echo -- \"touch .depend\" to create an empty file
	@echo ----------------------------------------------------------------
	$(SHELL) -ec 'g++ -MM $(CXXFLAGS) $(LIB_FILES) \
	    | sed '\''s/\(.*\.o[ ]*:[ ]*\)/$(OBJECT_DIR)\/\1/g'\'' > $(OBJECT_DIR)/.depend'
  else
    ifeq ($(HW_OS),powerpc_aix)
	@$(RMF) *.u
	$(SHELL) -ec '$(CC) -E -M $(CFLAGS) $(LIB_FILES) > /dev/null 2>&1'
	cat *.u > .depend
	@$(RMF) *.u
    else
	$(SHELL) -ec '$(CC) -M $(CFLAGS) $(LIB_FILES) \
	    | sed '\''s/\(.*\.o[ ]*:[ ]*\)/$(OBJECT_DIR)\/\1/g'\'' > $(OBJECT_DIR)/.depend'
    endif
  endif
endif
	@echo ----------------------------------------------------------------
