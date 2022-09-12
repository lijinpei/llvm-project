#ifndef LLVM_CODEGEN_INTERFERENCE_GRAPH_H
#define LLVM_CODEGEN_INTERFERENCE_GRAPH_H

#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/DenseMapInfo.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/GraphTraits.h"
#include "llvm/ADT/IndexedMap.h"
#include "llvm/ADT/Twine.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/CodeGen/SlotIndexes.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/raw_ostream.h"

#include <utility>

namespace llvm {

class MCRegisterClass;
class MCRegisterInfo;
class MachineRegisterInfo;
class LiveIntervals;
class SlotIndexes;

// TODO(lijinpei): handle reserved registers
/// RegisterClassInterferenceInfo - Two register classes interference iff they
/// have aliasing register. We won't bother to create a edge for two register if
/// their register classes don't interference.
class RegisterClassInterferenceInfo {
  const MCRegisterInfo &MCRI;
  BitVector IntfVec;
  static inline std::size_t getIndexInVec(unsigned ID1, unsigned ID2) {
    if (ID1 > ID2) {
      std::swap(ID1, ID2);
    }
    return ID2 * (ID2 - 1) / 2 + ID1;
  }
  void compute();

public:
  RegisterClassInterferenceInfo(const MCRegisterInfo &MCRI) : MCRI(MCRI) {
    compute();
  }
  unsigned getNumRegClasses() const { return MCRI.getNumRegClasses(); }
  bool needCheckInterference(const MCRegisterClass &RC1,
                             const MCRegisterClass &RC2) const;
};

struct IntfGraphNode {
  DenseSet<Register> Neighboors;
  bool addNeighboor(Register Reg1) { return Neighboors.insert(Reg1).second; }
};

class InterferenceGraph {
  const MachineRegisterInfo &MRI;
  const LiveIntervals &LIS;
  const SlotIndexes &SI;
  const RegisterClassInterferenceInfo RCIntf;
  IndexedMap<IntfGraphNode, VirtReg2IndexFunctor> VirtRegNodes;

public:
  InterferenceGraph(const MachineRegisterInfo &MRI, const LiveIntervals &LI);
  void buildRange(SlotIndex Begin, SlotIndex End);
  void addInterference(Register Reg1, Register Reg2, SlotIndex Pos);
  const RegisterClassInterferenceInfo &getRCIntfInfo() const { return RCIntf; }
  void dumpDotGraph(raw_ostream &OS, const Twine &Name) const;
};

class InterferenceGraphConstruction : public MachineFunctionPass {
  std::unique_ptr<InterferenceGraph> IntfGraph;

public:
  static char ID;
  InterferenceGraphConstruction();
  ~InterferenceGraphConstruction() override;
  void getAnalysisUsage(AnalysisUsage &au) const override;
  void releaseMemory() override;
  bool runOnMachineFunction(MachineFunction &MF) override;
  InterferenceGraph &getIntfGraph() { return *IntfGraph; }
  const InterferenceGraph &getIntfGraph() const { return *IntfGraph; }
};
} // end namespace llvm

#endif // LLVM_CODEGEN_INTERFERENCE_GRAPH_H
