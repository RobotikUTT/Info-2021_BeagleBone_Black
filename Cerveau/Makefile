#####################################
# compilation du projet 
# Attention à bien utiliser un compilateur arm-linux (linaro)
#####################################



######################################
# Flags
######################################

CFLAGS= -Wall  # options de compilation pour les sources C



######################################
# Répertoires
######################################

# Répertoire des Sources
vpath %.c $(CURDIR)/Sources

# Répertoire des Includes
INCDIR = $(CURDIR)/Includes

# Répertoire des fichiers Objets
OBJDIR = $(CURDIR)/object



######################################
# sources et headers
######################################

C_SOURCES= main.c

CPP_SOURCES= main.cpp

INCLUDES= main.h



######################################
# Nommage des fichiers
######################################

# nommage automatique des fichiers objets d'apres les noms des sources C
C_OBJECTS= $(C_SOURCES:.c=.o)

# nommage automatique des fichiers objets d'apres les noms des sources C
CPP_OBJECTS= $(CPP_SOURCES:.cpp=.o)



######################################
# Build
######################################

# numerotation automatique des versions
include build

# numerotation des compilations completes
build:
    echo BUILD= 1 > build
	
clean:
	$(REMOVE) $(OBJDIR)/*.o