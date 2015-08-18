# Makefile for bundler

CC=gcc
OPTFLAGS=-O3 -Wall

OS=$(shell uname -o)

ifeq ($(OS), Cygwin)
BUNDLER=bundler.exe
KEYMATCHFULL=KeyMatchFull.exe
KEYMATCH=KeyMatch.exe
BUNDLE2PMVS=Bundle2PMVS.exe
BUNDLE2VIS=Bundle2Vis.exe
RADIALUNDISTORT=RadialUndistort.exe
else
BUNDLER=bundler
KEYMATCHFULL=KeyMatchFull
KEYMATCH=KeyMatch
BUNDLE2PMVS=Bundle2PMVS
BUNDLE2VIS=Bundle2Vis
RADIALUNDISTORT=RadialUndistort
endif

INCLUDE_PATH=-I../lib/imagelib -I../lib/sfm-driver -I../lib/matrix	\
	-I../lib/5point -I../lib/sba-1.5 -I../lib/ann_1.1_char/include

LIB_PATH=-L../lib -L../lib/ann_1.1_char/lib

CPPFLAGS=$(OPTFLAGS) $(OTHERFLAGS) $(INCLUDE_PATH) $(DEFINES)

BUNDLER_DEFINES=-D__NO_UI__ -D__BUNDLER__ -D__BUNDLER_DISTR__

BUNDLER_OBJS=BaseApp.o BundlerApp.o keys.o Register.o Epipolar.o	\
	Bundle.o BundleFast.o MatchTracks.o Camera.o Geometry.o		\
	ImageData.o SifterUtil.o BaseGeometry.o BundlerGeometry.o	\
	BoundingBox.o BundleAdd.o ComputeTracks.o BruteForceSearch.o	\
	BundleIO.o ProcessBundle.o BundleTwo.o Decompose.o		\
	RelativePose.o Distortion.o TwoFrameModel.o LoadJPEG.o

BUNDLER_LIBS=-limage -lsfmdrv -lsba.v1.5 -lmatrix -lz -llapack -lblas \
	-lcblas -lminpack -lm -l5point -ljpeg -lANN_char -lgfortran


all: $(BUNDLER) $(KEYMATCHFULL) $(KEYMATCH) $(BUNDLE2PMVS) $(BUNDLE2VIS) $(RADIALUNDISTORT)

%.o : %.cpp
	$(CXX) -c -o $@ $(CPPFLAGS) $(WXFLAGS) $(BUNDLER_DEFINES) $<

$(BUNDLER): $(BUNDLER_OBJS)
	$(CXX) -o $@ $(CPPFLAGS) $(LIB_PATH) \
		$(BUNDLER_DEFINES) $(BUNDLER_OBJS) $(BUNDLER_LIBS)
	cp $@ ../bin

$(KEYMATCHFULL): KeyMatchFull.o keys2a.o
	$(CXX) -o $@ $(CPPFLAGS) $(LIB_PATH) KeyMatchFull.o keys2a.o \
		-lANN_char -lz
	cp $@ ../bin

$(KEYMATCH): KeyMatch.o keys2a.o
	$(CXX) -o $@ $(CPPFLAGS) $(LIB_PATH) KeyMatch.o keys2a.o \
		-lANN_char -lz
	cp $@ ../bin

$(BUNDLE2PMVS): Bundle2PMVS.o LoadJPEG.o
	$(CXX) -o $@ $(CPPFLAGS) $(LIB_PATH) Bundle2PMVS.o LoadJPEG.o \
		-limage -lmatrix -llapack -lblas -lcblas -lgfortran \
		-lminpack -ljpeg
	cp $@ ../bin

$(BUNDLE2VIS): Bundle2Vis.o
	$(CXX) -o $@ $(CPPFLAGS) $(LIB_PATH) Bundle2Vis.o
	cp $@ ../bin

$(RADIALUNDISTORT): RadialUndistort.o LoadJPEG.o
	$(CXX) -o $@ $(CPPFLAGS) $(LIB_PATH) $^ \
		-limage -lmatrix -llapack -lblas -lcblas -lgfortran \
		-lminpack -ljpeg
	cp $@ ../bin

clean:
	rm -f *.o *~ $(BUNDLER) $(KEYMATCHFULL) $(KEYMATCH) $(BUNDLE2PMVS) \
		$(BUNDLE2VIS) $(RADIALUNDISTORT)
