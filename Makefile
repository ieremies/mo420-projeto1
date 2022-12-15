
#=============================================================================
# Probably, you only need to change the attribution of the following variables:
# LEMONDIR
# GUROBI_DIR
# find the corresponding lines below. Note that for the GUROBI_DIR, there are
# two parts, depending on the operational system (Linux or OSX, see below).
# Change the paths accordingly.
#=============================================================================

#================= LEMON =====================================================
# if you have the lemon package installed in a specific folder, change the
# following line (most probably, if you installed the lemon library using
# the script Install-Lemon-in-subfolder.bash, you will not need to change anything
# (perhaps the lemon version):
LEMONDIR = ./lemon/lemon-1.3.1
#LEMONDIR = /usr/local

LEMONINC  = -I$(LEMONDIR)/include
LEMONLIB  = -L$(LEMONDIR)/lib   -lemon 

#================= GUROBI =====================================================
# If Gurobi is installed, there is a program called gurobi_cl that can be used to
# detect the version
GUROBI_DIR = /opt/gurobi1000/linux64
FLAGVERSION := 100

HOMEDIR = .

# if your operational system is OSX (e.g., for mac computers)
ifeq ($(shell uname), Darwin)
        $(info )
        $(info *)
        $(info *        Makefile for MAC OS environment)
        $(info *)
	PLATFORM = mac64
	CPPSTDLIB = -stdlib=libc++
	CC      = g++
	CC_ARGS    = -m64 -O2 -ferror-limit=2 -Wall -std=c++11 -D_GLIBCXX_USE_CXX11_ABI=0
	CC_LIB   = -lm -lpthread $(CPPSTDLIB)

	# if your operational system is Linux
else
        $(info )
        $(info *)
        $(info *        Makefile for LINUX environment)
        $(info *)
	PLATFORM = linux64
	CC      = g++
	CC_ARGS    = -g -m64 -O2 -Wall -std=c++11 -D_GLIBCXX_USE_CXX11_ABI=0
	CC_LIB   = -lm -lpthread
endif
GUROBI_INC = -I$(GUROBI_DIR)/include
GUROBI_LIB = -L$(GUROBI_DIR)/lib  -lgurobi_c++ -lgurobi$(FLAGVERSION)  $(CPPSTDLIB)
#===============================================================================


HOMEDIR_INC = $(HOMEDIR)/include
HOMEDIR_LIB = $(HOMEDIR)/lib
HOMEDIR_SRC = $(HOMEDIR)/src
HOMEDIR_OBJ = $(HOMEDIR)/obj
HOMEDIR_BIN = $(HOMEDIR)/bin

#---------------------------------------------
# define includes and libraries
INC = $(GUROBI_INC)  $(LEMONINC) -I$(HOMEDIR_INC) 
LIB = $(CC_LIB) $(GUROBI_LIB) $(LEMONLIB) -L$(HOMEDIR_LIB) 

_MLS = mygraphlib.cpp myutils.cpp mycolor.cpp geompack.cpp
_MLO = $(_MLS:.cpp=.o)
MYLIB_SRC = $(patsubst %,$(HOMEDIR_SRC)/%,$(_MLS))
MYLIB_OBJ = $(patsubst %,$(HOMEDIR_OBJ)/%,$(_MLO))

_EX_SEM_GUROBI = pli-projeto.cpp

_EX_COM_GUROBI =



_EX = $(_EX_SEM_GUROBI) $(_EX_COM_GUROBI)
_OB = $(_EX:.cpp=.o)
_BN = $(_EX:.cpp=.e)

# complete the EX's names with the path
EXAMPLES_SRC = $(patsubst %,$(HOMEDIR_SRC)/%,$(_EX))
EXAMPLES_OBJ = $(patsubst %,$(HOMEDIR_OBJ)/%,$(_OB))
EXAMPLES_BIN = $(patsubst %,$(HOMEDIR_BIN)/%,$(_BN))

all: $(EXAMPLES_OBJ) $(EXAMPLES_BIN) $(EXAMPLES_SRC) $(MYLIB_SRC) $(MYLIB_OBJ) $(HOMEDIR_LIB)/mylib.a


$(HOMEDIR_LIB)/mylib.a: $(MYLIB_SRC) $(MYLIB_OBJ)
	#libtool -o $@ $(MYLIB_OBJ)
	ar cru $@ $(MYLIB_OBJ)

$(HOMEDIR_BIN)/%.e: $(HOMEDIR_OBJ)/%.o $(HOMEDIR_LIB)/mylib.a
	$(CC) $(CC_ARGS) $^ -o $@ $(LIB) $(INC) 

$(HOMEDIR_OBJ)/%.o: $(HOMEDIR_SRC)/%.cpp
	$(CC) $(CC_ARGS) -c $^ -o $@ $(INC) 



clean:
	rm -f $(HOMEDIR)/*~ $(HOMEDIR_BIN)/*.e $(HOMEDIR_OBJ)/*.o $(HOMEDIR_LIB)/*.a $(HOMEDIR_SRC)/*~ $(HOMEDIR_INC)/*~ 
