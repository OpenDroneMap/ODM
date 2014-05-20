#ifndef CMVS_GRACLUS_H
#define CMVS_GRACLUS_H

#include <vector>
#include <metis.h>

/*
  0 -- 1 -- 2 -- 3 -- 4
  |    |    |    |    |
  5 -- 6 -- 7 -- 8 -- 9
  |    |    |    |    |
 10 --11 --12 --13 --14

 CSR format
 xadj: 0 2 5 8 11 13 16 20 24 28 31 33 36 39 42 44
 Size of xadj is the number of vertices plus 1.
 
 adjncy: 1 5 0 2 6 1 3 7 2 4 8 3 9 0 6 10 1 5 7 11 2 6 8 12 3 7 9 13 4 8 14 5 11 6 10 12 7 11 13 8 12 14 9 13

 Size of adjncy is twice the number of edges.
*/

namespace CMVS {
class Cgraclus {
 public:
  Cgraclus(void);
  virtual ~Cgraclus();

  // Threads free. no weights
  static void run(std::vector<idxtype>& xadj,
                  std::vector<idxtype>& adjncy,
                  const int nparts, const int cutType,
                  std::vector<idxtype>& part);
  
  // Threads free. vertex weights
  static void runV(std::vector<idxtype>& xadj,
                   std::vector<idxtype>& adjncy,
                   std::vector<idxtype>& vwgt,
                   const int nparts, const int cutType,
                   std::vector<idxtype>& part);
  
  // Threads free. edge weights
  static void runE(std::vector<idxtype>& xadj,
                   std::vector<idxtype>& adjncy,
                   std::vector<idxtype>& adjwgt,
                   const int nparts, const int cutType,
                   std::vector<idxtype>& part);
  
  // Threads free. vertex and edge weights
  static void runVE(std::vector<idxtype>& xadj,
                    std::vector<idxtype>& adjncy,
                    std::vector<idxtype>& vwgt,
                    std::vector<idxtype>& adjwgt,
                    const int nparts, const int cutType,
                    std::vector<idxtype>& part);
  
 protected:
  static int mylog2(int a);

  static void runSub(GraphType& graph, int nparts, int cutType,
                     int wgtflag, std::vector<idxtype>& part);
  
  static void initGraph(GraphType& graph);
};
};

#endif // CMVS_GRACLUS_H
