#include "llvm/CodeGen/InterferenceGraph.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/InitializePasses.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Pass.h"

#include <queue>
#include <utility>

using namespace llvm;
#define DEBUG_TYPE "intfgraph"

static cl::opt<bool> DumpIntfGraph("dump-intf-graph", cl::Hidden,
                                   cl::desc("Dump interference graph"));

void RegisterClassInterferenceInfo::compute() {
  unsigned NumRegClasses = getNumRegClasses();
  IntfVec.clear();
  IntfVec.resize(NumRegClasses * (NumRegClasses - 1) / 2);
  BitVector AliasingRC(NumRegClasses);
  for (unsigned RU = 0, RUE = MCRI.getNumRegUnits(); RU < RUE; ++RU) {
    AliasingRC.reset();
    for (MCRegUnitRootIterator RURI(RU, &MCRI); RURI.isValid(); ++RURI) {
      for (MCSuperRegIterator SRI(*RURI, &MCRI, true); SRI.isValid(); ++SRI) {
        for (unsigned RCI = 0; RCI < NumRegClasses; ++RCI) {
          if (MCRI.getRegClass(RCI).contains(*SRI)) {
            AliasingRC.set(RCI);
          }
        }
      }
    }
    for (unsigned ID1 : AliasingRC.set_bits()) {
      unsigned ID2 = ID1;
      do {
        int NextID = AliasingRC.find_next(ID2);
        if (NextID < 0) {
          break;
        }
        ID2 = (unsigned)NextID;
        std::size_t Index = getIndexInVec(ID1, ID2);
        IntfVec.set(Index);
      } while (true);
    }
  }
}

bool RegisterClassInterferenceInfo::needCheckInterference(
    const MCRegisterClass &RC1, const MCRegisterClass &RC2) const {
  if (&RC1 == &RC2) {
    return true;
  }
  if (!RC1.isAllocatable() || !RC2.isAllocatable()) {
    return false;
  }
  std::size_t Idx = getIndexInVec(RC1.getID(), RC2.getID());
  return IntfVec.test(Idx);
}

InterferenceGraph::InterferenceGraph(const MachineRegisterInfo &MRI,
                                     const LiveIntervals &LIS)
    : MRI(MRI), LIS(LIS), SI(*LIS.getSlotIndexes()),
      RCIntf(*MRI.getTargetRegisterInfo()) {
  VirtRegNodes.resize(MRI.getNumVirtRegs());
}

void InterferenceGraph::addInterference(Register Reg1, Register Reg2,
                                        SlotIndex Pos) {
  const TargetRegisterClass *RC1 = MRI.getRegClass(Reg1);
  const TargetRegisterClass *RC2 = MRI.getRegClass(Reg2);
  if (!RCIntf.needCheckInterference(*RC1->MC, *RC2->MC)) {
    return;
  }
  VirtRegNodes[Reg1].addNeighboor(Reg2);
  VirtRegNodes[Reg2].addNeighboor(Reg1);
}

namespace {
struct VRegIntfInfo {
  const LiveInterval *LI = nullptr;
  LiveInterval::const_iterator Iter;
  VRegIntfInfo(const LiveInterval &LI, SlotIndex Start)
      : LI(&LI), Iter(LI.find(Start)) {}
  bool isValid() const { return Iter != LI->end(); }
  Register reg() const { return LI->reg(); }
  SlotIndex start() const {
    assert(isValid());
    return Iter->start;
  }
  SlotIndex end() const {
    assert(isValid());
    return Iter->end;
  }
  bool bumpToNext() {
    assert(isValid());
    Iter = std::next(Iter);
    return isValid();
  }
};
struct VRegInfoStartComp {
  bool operator()(const VRegIntfInfo &VR1, const VRegIntfInfo &VR2) {
    return VR1.start() < VR2.start();
  }
};
struct VRegInfoEndComp {
  bool operator()(const VRegIntfInfo &VR1, const VRegIntfInfo &VR2) {
    return VR1.end() < VR2.end();
  }
};
using ActiveVRegPQ =
    std::priority_queue<VRegIntfInfo, std::vector<VRegIntfInfo>,
                        VRegInfoEndComp>;
using PendingVRegPQ =
    std::priority_queue<VRegIntfInfo, std::vector<VRegIntfInfo>,
                        VRegInfoStartComp>;
} // namespace

void InterferenceGraph::buildRange(SlotIndex Begin, SlotIndex End) {
  ActiveVRegPQ Actives;
  PendingVRegPQ Pendings;
  SmallDenseSet<Register, 32> CurrentRegs;
  for (unsigned VRegIdx = 0, NumVRegs = MRI.getNumVirtRegs();
       VRegIdx < NumVRegs; ++VRegIdx) {
    Register VReg = Register::index2VirtReg(VRegIdx);
    if (!LIS.hasInterval(VReg)) {
      continue;
    }
    VRegIntfInfo IntfInfo(LIS.getInterval(VReg), Begin);
    if (!IntfInfo.isValid()) {
      continue;
    }
    Pendings.push(IntfInfo);
  }
  auto ActiveFirstStart = [&]() {
    VRegIntfInfo Info = Pendings.top();
    Pendings.pop();
    SlotIndex Pos = std::max(Begin, Info.start());
    for (auto Reg : CurrentRegs) {
      addInterference(Info.reg(), Reg, Pos);
    }
    CurrentRegs.insert(Info.reg());
    Actives.push(Info);
  };
  auto DeactiveFirstEnd = [&]() {
    VRegIntfInfo Info = Actives.top();
    if (!Info.bumpToNext()) {
      return;
    }
    Pendings.push(Info);
  };
  while (!Pendings.empty()) {
    SlotIndex FirstPending = Pendings.top().start();
    if (FirstPending >= End) {
      break;
    }
    while (!Actives.empty() && Actives.top().end() <= FirstPending) {
      DeactiveFirstEnd();
    }
    ActiveFirstStart();
  }
}

void InterferenceGraph::dumpDotGraph(raw_ostream &OS, const Twine &Name) const {
  OS << "graph " << Name << " {\n";
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();
  for (unsigned I = 0, E = MRI.getNumVirtRegs(); I < E; ++I) {
    Register Reg = Register::index2VirtReg(I);
    OS << "\t" << I << "[label="
       << "\"" << printReg(Reg, TRI, 0, &MRI) << ":"
       << TRI->getRegClassName(MRI.getRegClass(Reg)) << "\"];";
  }
  OS << "\n";
  for (unsigned I = 0, E = MRI.getNumVirtRegs(); I < E; ++I) {
    Register RegI = Register::index2VirtReg(I);
    for (auto RegJ : VirtRegNodes[RegI].Neighboors) {
      if (RegJ < RegI) {
        continue;
      }
      unsigned J = Register::virtReg2Index(RegJ);
      OS << "\t" << I << " -- " << J << ";\n";
    }
  }
  OS << "}\n";
}

char InterferenceGraphConstruction::ID = 0;
char &llvm::InterferenceGraphConstructionID = LiveIntervals::ID;
INITIALIZE_PASS_BEGIN(InterferenceGraphConstruction, DEBUG_TYPE,
                      "Interference Graph Construction", false, false)
INITIALIZE_PASS_DEPENDENCY(LiveIntervals)
INITIALIZE_PASS_END(InterferenceGraphConstruction, DEBUG_TYPE,
                    "Interference Graph Construction", false, false)

void InterferenceGraphConstruction::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesCFG();
  AU.addRequiredTransitive<LiveIntervals>();
  AU.addPreserved<LiveIntervals>();
  MachineFunctionPass::getAnalysisUsage(AU);
}

InterferenceGraphConstruction::InterferenceGraphConstruction()
    : MachineFunctionPass(ID) {
  initializeInterferenceGraphConstructionPass(*PassRegistry::getPassRegistry());
}
InterferenceGraphConstruction::~InterferenceGraphConstruction() {}

void InterferenceGraphConstruction::releaseMemory() { IntfGraph.reset(); }

bool InterferenceGraphConstruction::runOnMachineFunction(MachineFunction &MF) {
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  const LiveIntervals &LI = getAnalysis<LiveIntervals>();
  SlotIndexes &SI = *LI.getSlotIndexes();
  IntfGraph.reset(new InterferenceGraph(MRI, LI));
  IntfGraph->buildRange(SI.getZeroIndex(), SI.getLastIndex());
  if (DumpIntfGraph) {
    IntfGraph->dumpDotGraph(dbgs(), MF.getName());
  }
  return false;
}
