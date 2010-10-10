# MIT/X Consortium License
#
# © 2008 - 2009 Christoph Schied
# © 2009 - 2010 Nicolai Waniek
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the 
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# -----------------------------------------------------------------------------

TARGETNAME = kalman
CC         = ccache g++
VERSION    = `date '+%Y%m%d'`
INCS       = -I include \
			 -I /usr/include/boost
LIBS       = -L/usr/lib -Wl,-rpath,/usr/lib \
			 -lpthread
WARNINGS   = -Wall -Woverloaded-virtual -Wextra -Wpointer-arith -Wcast-qual   \
			 -Wswitch-default -Wcast-align -Wundef -Wno-empty-body
CPPFLAGS   = -DVERSION=$(VERSION) -DDEBUG
CFLAGS     = -O3 -fomit-frame-pointer -funroll-loops -ffast-math -msse2       \
			 $(INCS) $(CPPFLAGS) $(WARNINGS)
LDFLAGS    = $(LIBS)
ROOTDIR    = $(PWD)
SRCDIR     = $(ROOTDIR)/src
OBJDIR     = $(ROOTDIR)/build

# -----------------------------------------------------------------------------

DIRS       = 
SRC        = main.cpp 

# -----------------------------------------------------------------------------

OBJ        = $(SRC:%.cpp=$(OBJDIR)/%.o)
DIRTREE    = $(OBJDIR) \
			 $(DIRS:%=$(OBJDIR)/%)
DEPENDS    = $(SRC:%.cpp=$(OBJDIR)/%.d)

# -----------------------------------------------------------------------------

define link
	@echo -e '\033[1;33m'[LD] $1 '\033[1;m'
	@cd $(OBJDIR); $(CC) -o $(ROOTDIR)/$1 $^ $2
endef

# in the compile section, possibliy add the following line to output what
# you're compiling at the moment. but: a clean build should not output
# anything
#
#    #echo [CC] $<
#
define compile
	@$(CC) -o $@ -c $1 $<
endef

define make-dep
	@$(CC) -M -MG -MP -MT "$@" -MF $(subst .o,.d,$@) $1 $<
endef

# -----------------------------------------------------------------------------

.PHONY: all bin builddir dist clean

all: builddir bin

bin: $(OBJ)
	$(call link,$(TARGETNAME),$(LDFLAGS))

builddir:
	@mkdir -p $(DIRTREE)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(call make-dep, $(INCS))
	$(call compile,$(CFLAGS))

clean:
	@echo "cleaning"
	@rm -rf $(TARGETNAME) $(OBJDIR)

dist:
	@echo "creating dist tarball $(TARGETNAME)-$(VERSION).tar.gz"
	@git archive HEAD | gzip > $(TARGETNAME)-$(VERSION).tar.gz

-include $(DEPENDS)
