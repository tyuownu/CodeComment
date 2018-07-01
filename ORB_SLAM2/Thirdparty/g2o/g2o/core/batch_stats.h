// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_BATCH_STATS_H_
#define G2O_BATCH_STATS_H_
#include <iostream> #include <vector>


namespace g2o {

  /**
   * \brief statistics about the optimization
   */
  struct  G2OBatchStatistics {
    G2OBatchStatistics();
    ///< which iteration
    int iteration;
    ///< how many vertices are involved
    int numVertices;
    ///< how many edges
    int numEdges;
    ///< total chi2
    double chi2;

    /** timings **/
    // nonlinear part
    ///< residuals
    double timeResiduals;
    ///< jacobians
    double timeLinearize;
    ///< construct the quadratic form in the graph
    double timeQuadraticForm;
    ///< number of iterations performed by LM
    int levenbergIterations;
    // block_solver (constructs Ax=b, plus maybe schur)
    ///< compute schur complement (0 if not done)
    double timeSchurComplement;

    // linear solver (computes Ax=b);
    ///< symbolic decomposition (0 if not done)
    double timeSymbolicDecomposition;
    ///< numeric decomposition  (0 if not done)
    double timeNumericDecomposition;
    ///< total time for solving Ax=b (including detup for schur)
    double timeLinearSolution;
    ///< time for solving, excluding Schur setup
    double timeLinearSolver;
    ///< iterations of PCG, (0 if not used, i.e., Cholesky)
    int    iterationsLinearSolver;
    ///< time to apply the update
    double timeUpdate;
    ///< total time;
    double timeIteration;

    ///< computing the inverse elements (solve blocks) and thus the marginal covariances
    double timeMarginals;

    // information about the Hessian matrix
    ///< rows / cols of the Hessian
    size_t hessianDimension;
    ///< dimension of the pose matrix in Schur
    size_t hessianPoseDimension;
    ///< dimension of the landmark matrix in Schur
    size_t hessianLandmarkDimension;
    ///< number of non-zeros in the cholesky factor
    size_t choleskyNNZ;

    static G2OBatchStatistics* globalStats() {return _globalStats;}
    static void setGlobalStats(G2OBatchStatistics* b);
    protected:
    static G2OBatchStatistics* _globalStats;
  };

  std::ostream& operator<<(std::ostream&, const G2OBatchStatistics&);

  typedef std::vector<G2OBatchStatistics> BatchStatisticsContainer;
}  // namespace g2o

#endif
