######################################################################
#
# The following 2 commands to compile pmvs2.
# > make depend
# > make
#
######################################################################
CXX = g++

Your INCLUDE path (e.g., -I/usr/include)
YOUR_INCLUDE_PATH =

Your metis directory (contains header files under graclus1.2/metisLib/)
YOUR_INCLUDE_METIS_PATH =

Your LDLIBRARY path (e.g., -L/usr/lib)
YOUR_LDLIB_PATH =

######################################################################
CXXFLAGS_PMVS = -O2 -Wall -Wno-deprecated ${YOUR_INCLUDE_PATH}

CXXFLAGS_CMVS = -O2 -Wall -Wno-deprecated -DNUMBITS=64 \
		${YOUR_INCLUDE_PATH} ${YOUR_INCLUDE_METIS_PATH} \
		-fopenmp -DNUMBITS=64 ${OPENMP_FLAG}


LDFLAGS_PMVS = ${YOUR_LDLIB_PATH} -lXext -lX11 -ljpeg -lm -lpthread \
		-llapack -lgsl -lgslcblas

LDFLAGS_CMVS = ${YOUR_LDLIB_PATH} -lXext -lX11 -ljpeg -lm -lpthread \
		-llapack -fopenmp -lmultilevel -lmetis -lm

######################################################################
BASE_IMAGE = camera.o image.o photo.o photoSetS.o
BASE_PMVS = detectFeatures.o detector.o dog.o expand.o filter.o \
	findMatch.o harris.o optim.o option.o patch.o patchOrganizerS.o \
	point.o seed.o
BASE_NUMERIC = mylapack.o
BASE_CMVS = bundle.o graclus.o

######################################################################
all: pmvs2 cmvs genOption


pmvs2: pmvs2.o ${BASE_IMAGE} ${BASE_PMVS} ${BASE_NUMERIC}
	${CXX} ${LDFLAGS_PMVS} -o $@ $^ ${LDFLAGS_PMVS}

cmvs: cmvs.o patch.o ${BASE_IMAGE} ${BASE_CMVS}
	${CXX} ${LDFLAGS_CMVS} -o $@ $^ ${LDFLAGS_CMVS}

genOption: genOption.cc
	${CXX} -o $@ $^


pmvs2.o : pmvs2.cc
	$(CXX) -c $(CXXFLAGS_PMVS) $<

cmvs.o: cmvs.cc
	$(CXX) -c $(CXXFLAGS_CMVS) $<

%.o : ../base/pmvs/%.cc
	$(CXX) -c $(CXXFLAGS_PMVS) $<

%.o : ../base/image/%.cc
	$(CXX) -c $(CXXFLAGS_PMVS) $<

%.o : ../base/numeric/%.cc
	$(CXX) -c $(CXXFLAGS_PMVS) $<

%.o : ../base/cmvs/%.cc
	$(CXX) -c $(CXXFLAGS_CMVS) $<

######################################################################
# general commands
clean:
	/bin/rm -f core core.* *.o ${TARGET}

depend:
	rm -f Dependencies
	for SOURCEFILE in `ls *.cc ../base/*/*.cc ../base/*/*.c ../base/*/*.C ../base/*/*.f`; do \
	  echo $$SOURCEFILE; \
	  $(CXX) -MM $(CXXFLAGS) $$SOURCEFILE >> Dependencies; \
	done

-include Dependencies
