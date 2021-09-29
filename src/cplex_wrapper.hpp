// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// run via:
// export LD_LIBRARY_PATH=/opt/ibm/ILOG/CPLEX_Studio1210/opl/bin/x86-64_linux
// test : env | grep LD_
// any code that uses the wrapper has to set the following copts: "-DIL_STD",
// "-fPIC", "-DILOUSEMT", "-D_REENTRANT" bazel test //test:cplex_wrapper_test

#ifndef CPLEX_WRAPPER_HEADER
#define CPLEX_WRAPPER_HEADER

#include "miqp_planner_data.hpp"
#include "miqp_planner_settings.h"
#include "model_input_data_source.hpp"

#include <sstream>

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include "bark/commons/util/util.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

#include "ilopl/iloopl.h"

namespace miqp {
namespace planner {
namespace cplex {

class NotImplementedException : public std::logic_error {
 public:
  NotImplementedException()
      : std::logic_error{"Function not yet implemented."} {}
};

struct SolutionProperties {
  int status;
  double gap;
  double objective;
  double time;
  int NrConstraints;
  int NrBinaryVariables;
  int NrFloatVariables;
  int NonZeroCoefficients;
  int NrIterations;
  int NrSolutionPool;
};

enum OptimizationStatus {
  SUCCESS = 0,
  FAILED_NO_SOLUT = 1,
  FAILED_SEG_FAULT = 2,
  FAILED_TIMEOUT = 3
};

class CplexWrapper {
 public:
  enum ParameterSource { DATFILE = 0, CPPINPUTS = 1, MIXED = 2 };
  typedef MiqpPlannerWarmstartType WarmstartType;
  typedef MiqpPlannerParallelMode ParallelMode;

  /**
   * @brief CplexWrapper - a class interfacing our MIQP cplex model with the car
   *agent.
   *
   * @note the @param percision: The floating point precision that is a) used to
   *fix all parameters to in ModelInputDataSource and b) as an output accurancy
   *when serializing to the problem to a DAT file. A too low precision procudes
   *wrong results. A too high precision causes numerical problems and
   *differences in the dat file and cpp struct solutions of the same problem. A
   *precision of 12 is the sweet spot in the py_debugging_toolchain_test unit
   *test.
   * @note the precision in the ModelInputDataSource ds_ shall be -2 the
   *precision that the results match! -2 is the sweet spot in the unit test.
   **/

  CplexWrapper(const char* modpath, const char* modfile,
               ParameterSource parameterSource, const int precision)
      : modfile_(std::string(modpath) + std::string(modfile)),
        datfile_(""),
        env_(IloEnv()),
        precision_(precision),
        ds_(env_, precision - 2),  // -2
        parameterSource_(parameterSource),
        rawResults_(std::make_shared<RawResults>()),
        print_debug_outputs_(false),
        doWarmstart_(NO_WARMSTART),
        warmstartValues_(std::make_shared<RawResults>()),
        cplexHandlerOut_(),
        handler_(env_, cplexHandlerOut_),
        modelSource_(env_, modfile_.c_str()),
        settings_(env_, handler_),
        def_(modelSource_, settings_),
        solutionProperties_(),
        debugOutputFilePath_(""),
        debugOutputFilePrefix_(""),
        debugOutputParameterFilePath_(""),
        useSpecialOrderedSets_(false),
        tmpWarmstartFile_("/tmp/warmstart_debug_res.mst"),
        useBranchingPriorities_(false),
        increasedBranchingPriorityValue_(1),
        branchingPriorityExtent_(1),
        bufferCplexOutputsToStream_(false) {
    settings_.setWithNames(IloTrue);
    settings_.setDisplayPrecision(precision);
  }

  CplexWrapper(const char* modfile, ParameterSource parameterSource,
               const int precision)
      : CplexWrapper("../cplex_models/", modfile, parameterSource, precision) {
  }  // due to the bazel structure this is
     // the correct relative location....

  CplexWrapper(const char* modfile, const int precision)
      : CplexWrapper(modfile, ParameterSource::DATFILE, precision) {}

  CplexWrapper() : CplexWrapper("", ParameterSource::CPPINPUTS, 12) {}

  CplexWrapper(const CplexWrapper& cp2)
      : modfile_(cp2.modfile_),
        datfile_(cp2.datfile_),
        env_(IloEnv()),  // HACK
        precision_(cp2.precision_),
        ds_(env_, cp2.precision_),  // HACK
        parameterSource_(cp2.parameterSource_),
        rawResults_(std::make_shared<RawResults>()),
        print_debug_outputs_(cp2.print_debug_outputs_),
        doWarmstart_(cp2.doWarmstart_),
        warmstartValues_(std::make_shared<RawResults>()),
        cplexHandlerOut_(cp2.cplexHandlerOut_.str().c_str()),
        handler_(env_, cplexHandlerOut_),
        modelSource_(env_, modfile_.c_str()),
        settings_(env_, handler_),
        def_(modelSource_, settings_),
        solutionProperties_(cp2.solutionProperties_),
        debugOutputFilePath_(cp2.debugOutputFilePath_),
        debugOutputFilePrefix_(cp2.debugOutputFilePrefix_),
        debugOutputParameterFilePath_(cp2.debugOutputParameterFilePath_),
        tmpWarmstartFile_(cp2.tmpWarmstartFile_),
        useBranchingPriorities_(cp2.useBranchingPriorities_),
        increasedBranchingPriorityValue_(cp2.increasedBranchingPriorityValue_),
        branchingPriorityExtent_(cp2.branchingPriorityExtent_),
        bufferCplexOutputsToStream_(cp2.bufferCplexOutputsToStream_) {
    settings_.setWithNames(IloTrue);
    settings_.setDisplayPrecision(precision_);
  }

  CplexWrapper& operator=(const CplexWrapper& rhs) {
    this->debugOutputParameterFilePath_ = rhs.debugOutputParameterFilePath_;
    return *this;
  }

  ~CplexWrapper() { env_.end(); }

 private:
  std::string modfile_;
  std::string datfile_;
  IloEnv env_;
  int precision_;
  ModelInputDataSource ds_;
  ParameterSource parameterSource_;
  std::shared_ptr<RawResults> rawResults_;
  bool print_debug_outputs_;
  WarmstartType doWarmstart_;
  std::shared_ptr<RawResults> warmstartValues_;
  std::stringstream cplexHandlerOut_;
  IloOplErrorHandler handler_;
  IloOplModelSource modelSource_;
  IloOplSettings settings_;
  IloOplModelDefinition def_;
  SolutionProperties solutionProperties_;
  std::string debugOutputFilePath_;
  std::string debugOutputFilePrefix_;
  std::string debugOutputParameterFilePath_;
  bool useSpecialOrderedSets_;
  std::string tmpWarmstartFile_;
  bool useBranchingPriorities_;
  int increasedBranchingPriorityValue_;
  int branchingPriorityExtent_;  // from 1 to N-1
  bool bufferCplexOutputsToStream_;

 public:
  void addRecedingHorizonWarmstart(
      std::shared_ptr<RawResults> warmstart,
      WarmstartType wt = RECEDING_HORIZON_WARMSTART);

  void setLastSolutionWarmstart(WarmstartType wt = LAST_SOLUTION_WARMSTART);

  void deleteLastSolutionWarmstartFile();

  void setParameterDatFileRelative(const char* datfile);

  void setParameterDatFileAbsolute(const char* datfile);

  void resetParameters(std::shared_ptr<ModelParameters> parameters);

  OptimizationStatus callCplex(const double timestemp = 0.0);

  std::shared_ptr<RawResults> getRawResults() const { return rawResults_; }

  SolutionProperties getSolutionProperties() const {
    return solutionProperties_;
  }

  void setDebugOutputPrint(bool print_debug_outputs) {
    print_debug_outputs_ = print_debug_outputs;
  }

  void setDebugOutputFilePath(std::string in);

  void setDebugOutputFilePrefix(std::string in) { debugOutputFilePrefix_ = in; }

  std::string getDebugOutputParameterFilePath() const {
    return debugOutputParameterFilePath_;
  }

  void setSpecialOrderedSets(bool in) { useSpecialOrderedSets_ = in; }

  void setUseBranchingPriorities(bool in) { useBranchingPriorities_ = in; }

  std::string getTmpWarmstartFile() { return tmpWarmstartFile_; }

  void setBranchingPriorityValueExtent(int value, int extent);

  void setBufferCplexOutputsToStream(bool in) {
    bufferCplexOutputsToStream_ = in;
  }

  void overrideSolverSettingsDataSource(
      std::shared_ptr<ModelParameters> parameters);

 private:
  void initializeWarmstart(IloOplModel& opl, IloCplex& cplex);

  template <class TensorType, class IloVarMapType>
  void initializeWarmstartElement(IloVarMapType& varMap, TensorType& vals,
                                  IloNumVarArray& startVar,
                                  IloNumArray& startVal, size_t dimDepth,
                                  std::vector<size_t> dimIdx);

  template <class TensorType>
  void getDecisionVariable(const IloOplElement& element, TensorType& outdata);

  // getNdimDecisionVariable: Put content from a IloMap into an EigenTensor
  // NOTE: no size checking! sizes have to be a priori correct!
  template <class TensorType, class IloMapType>
  void getNdimDecisionVariable(IloMapType m, TensorType& outdata,
                               std::vector<int> dimIdx);

  void setParametersFromDatfile(IloOplModel& opl);

  void setParametersFromModelInputDataSource(IloOplModel& opl);

  void setParallelMode(IloCplex& cplex);

  void collectRawResults(IloOplModel& opl);

  void collectSolutionStatus(const IloCplex& cplex, const double& solutiontime);

  void collectCplexStatistics(const IloCplex& cplex);

  void addSOSConstraints(IloOplModel& opl);

  template <class TensorType>
  void setBranchingPriorityFromTensor(IloCplex& cplex, IloIntVarMap& varMap,
                                      TensorType& vals, size_t dimDepth,
                                      std::vector<size_t> dimIdx);

  void setBranchingPriorities(IloOplModel& opl, IloCplex& cplex);
};

}  // namespace cplex
}  // namespace planner
}  // namespace miqp

#endif  // CPLEX_WRAPPER_HEADER