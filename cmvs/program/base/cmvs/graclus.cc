#include <iostream>
#include "graclus.h"

using namespace CMVS;
using namespace std;

// Required by library
int boundary_points;
int spectral_initialization;
int cutType;
int memory_saving;

Cgraclus::Cgraclus(void) {
}

Cgraclus::~Cgraclus() {
}

/*************************************************************************
* This function reads the spd matrix
**************************************************************************/
void Cgraclus::initGraph(GraphType& graph) {
  graph.gdata = graph.rdata = NULL;

  graph.nvtxs = graph.nedges = -1;
  graph.mincut = graph.minvol = -1;

  graph.xadj = graph.vwgt = graph.adjncy = graph.adjwgt = NULL;
  graph.adjwgtsum = NULL;
  graph.label = NULL;
  graph.cmap = NULL;

  graph.where = graph.pwgts = NULL;
  graph.id = graph.ed = NULL;
  graph.bndptr = graph.bndind = NULL;
  graph.rinfo = NULL;
  graph.vrinfo = NULL;
  graph.nrinfo = NULL;

  graph.ncon = -1;
  graph.nvwgt = NULL;
  graph.npwgts = NULL;

  graph.vsize = NULL;

  graph.coarser = graph.finer = NULL;
}

//----------------------------------------------------------------------
// cutType
// 0: NCUT, 1: RASSO
void Cgraclus::run(std::vector<idxtype>& xadj,
                   std::vector<idxtype>& adjncy,
                   const int nparts, const int cutType,
                   std::vector<idxtype>& part) {
  GraphType graph;
  initGraph(graph);

  graph.ncon = 1;

  graph.xadj = &xadj[0];
  graph.adjncy = &adjncy[0];
  graph.vwgt = NULL;
  graph.adjwgt = NULL;

  graph.nvtxs = (int)xadj.size() - 1;
  graph.nedges = (int)adjncy.size();

  const int wgtflag = 0;
  runSub(graph, nparts, cutType, wgtflag, part);
}

void Cgraclus::runV(std::vector<idxtype>& xadj,
                    std::vector<idxtype>& adjncy,
                    std::vector<idxtype>& vwgt,
                    const int nparts, const int cutType,
                    std::vector<idxtype>& part) {
  GraphType graph;
  initGraph(graph);
  
  graph.ncon = 1;

  graph.xadj = &xadj[0];
  graph.adjncy = &adjncy[0];
  graph.vwgt = &vwgt[0];
  graph.adjwgt = NULL;

  graph.nvtxs = (int)xadj.size() - 1;
  graph.nedges = (int)adjncy.size();

  const int wgtflag = 2;
  runSub(graph, nparts, cutType, wgtflag, part);
}

//----------------------------------------------------------------------
void Cgraclus::runE(std::vector<idxtype>& xadj,
                    std::vector<idxtype>& adjncy,
                    std::vector<idxtype>& adjwgt,
                    const int nparts, const int cutType,
                    std::vector<idxtype>& part) {
  GraphType graph;
  initGraph(graph);

  graph.ncon = 1;

  graph.xadj = &xadj[0];
  graph.adjncy = &adjncy[0];
  graph.vwgt = NULL;
  graph.adjwgt = &adjwgt[0];

  graph.nvtxs = (int)xadj.size() - 1;
  graph.nedges = (int)adjncy.size();
  
  const int wgtflag = 1;
  runSub(graph, nparts, cutType, wgtflag, part);
}

void Cgraclus::runVE(std::vector<idxtype>& xadj,
                     std::vector<idxtype>& adjncy,
                     std::vector<idxtype>& vwgt,
                     std::vector<idxtype>& adjwgt,
                     const int nparts, const int cutType,
                     std::vector<idxtype>& part) {
  GraphType graph;
  initGraph(graph);
  
  graph.ncon = 1;

  graph.xadj = &xadj[0];
  graph.adjncy = &adjncy[0];
  graph.vwgt = &vwgt[0];
  graph.adjwgt = &adjwgt[0];

  graph.nvtxs = (int)xadj.size() - 1;
  graph.nedges = (int)adjncy.size();
  
  const int wgtflag = 3;
  runSub(graph, nparts, cutType, wgtflag, part);
}

int Cgraclus::mylog2(int a) {
  int i;
  for (i = 1 ; a > 1; i++, a = a>>1);
  return i-1;
}

void Cgraclus::runSub(GraphType& graph, int nparts, int cutType,
                      int wgtflag, std::vector<idxtype>& part) {
  const int levels =
    amax((graph.nvtxs)/(40*mylog2(nparts)), 20*(nparts));
  part.resize(graph.nvtxs);

  int options[11];       options[0] = 0;
  int numflag = 0;
  int chain_length = 0;  int edgecut;

  MLKKM_PartGraphKway(&graph.nvtxs, graph.xadj, graph.adjncy,
                      graph.vwgt, graph.adjwgt, 
                      &wgtflag, &numflag, &nparts,
                      &chain_length, options, &edgecut, &part[0], levels);

  float lbvec[MAXNCON];
  ComputePartitionBalance(&graph, nparts, &part[0], lbvec);

  float result;
  if (cutType == 0){
    result = ComputeNCut(&graph, &part[0], nparts);
    printf("\nNormalized-Cut... \n   Cut value: %7f, Balance: ", result);
  }
  else{
    result = ComputeRAsso(&graph, &part[0], nparts);
    printf("\nRatio Association...  \n  Association value: %7f, Balance: ", result);
  }
}
