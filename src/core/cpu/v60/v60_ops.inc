// Converted by tools/v60conv.py from MAME 0.289 src/devices/cpu/v60/*.hxx
// (BSD-3-Clause, copyright-holders: Farfetch'd, R. Belmont). Do not edit; edit the converter.
// This file is part of ReBOX. GPL-2.0-or-later.

static uint32_t am1Register(void);
static uint32_t am1RegisterIndirect(void);
static uint32_t bam1RegisterIndirect(void);
static uint32_t am1RegisterIndirectIndexed(void);
static uint32_t bam1RegisterIndirectIndexed(void);
static uint32_t am1Autoincrement(void);
static uint32_t bam1Autoincrement(void);
static uint32_t am1Autodecrement(void);
static uint32_t bam1Autodecrement(void);
static uint32_t am1Displacement8(void);
static uint32_t bam1Displacement8(void);
static uint32_t am1Displacement16(void);
static uint32_t bam1Displacement16(void);
static uint32_t am1Displacement32(void);
static uint32_t bam1Displacement32(void);
static uint32_t am1DisplacementIndexed8(void);
static uint32_t bam1DisplacementIndexed8(void);
static uint32_t am1DisplacementIndexed16(void);
static uint32_t bam1DisplacementIndexed16(void);
static uint32_t am1DisplacementIndexed32(void);
static uint32_t bam1DisplacementIndexed32(void);
static uint32_t am1PCDisplacement8(void);
static uint32_t bam1PCDisplacement8(void);
static uint32_t am1PCDisplacement16(void);
static uint32_t bam1PCDisplacement16(void);
static uint32_t am1PCDisplacement32(void);
static uint32_t bam1PCDisplacement32(void);
static uint32_t am1PCDisplacementIndexed8(void);
static uint32_t bam1PCDisplacementIndexed8(void);
static uint32_t am1PCDisplacementIndexed16(void);
static uint32_t bam1PCDisplacementIndexed16(void);
static uint32_t am1PCDisplacementIndexed32(void);
static uint32_t bam1PCDisplacementIndexed32(void);
static uint32_t am1DisplacementIndirect8(void);
static uint32_t bam1DisplacementIndirect8(void);
static uint32_t am1DisplacementIndirect16(void);
static uint32_t bam1DisplacementIndirect16(void);
static uint32_t am1DisplacementIndirect32(void);
static uint32_t bam1DisplacementIndirect32(void);
static uint32_t am1DisplacementIndirectIndexed8(void);
static uint32_t bam1DisplacementIndirectIndexed8(void);
static uint32_t am1DisplacementIndirectIndexed16(void);
static uint32_t bam1DisplacementIndirectIndexed16(void);
static uint32_t am1DisplacementIndirectIndexed32(void);
static uint32_t bam1DisplacementIndirectIndexed32(void);
static uint32_t am1PCDisplacementIndirect8(void);
static uint32_t bam1PCDisplacementIndirect8(void);
static uint32_t am1PCDisplacementIndirect16(void);
static uint32_t bam1PCDisplacementIndirect16(void);
static uint32_t am1PCDisplacementIndirect32(void);
static uint32_t bam1PCDisplacementIndirect32(void);
static uint32_t am1PCDisplacementIndirectIndexed8(void);
static uint32_t bam1PCDisplacementIndirectIndexed8(void);
static uint32_t am1PCDisplacementIndirectIndexed16(void);
static uint32_t bam1PCDisplacementIndirectIndexed16(void);
static uint32_t am1PCDisplacementIndirectIndexed32(void);
static uint32_t bam1PCDisplacementIndirectIndexed32(void);
static uint32_t am1DoubleDisplacement8(void);
static uint32_t bam1DoubleDisplacement8(void);
static uint32_t am1DoubleDisplacement16(void);
static uint32_t bam1DoubleDisplacement16(void);
static uint32_t am1DoubleDisplacement32(void);
static uint32_t bam1DoubleDisplacement32(void);
static uint32_t am1PCDoubleDisplacement8(void);
static uint32_t bam1PCDoubleDisplacement8(void);
static uint32_t am1PCDoubleDisplacement16(void);
static uint32_t bam1PCDoubleDisplacement16(void);
static uint32_t am1PCDoubleDisplacement32(void);
static uint32_t bam1PCDoubleDisplacement32(void);
static uint32_t am1DirectAddress(void);
static uint32_t bam1DirectAddress(void);
static uint32_t am1DirectAddressIndexed(void);
static uint32_t bam1DirectAddressIndexed(void);
static uint32_t am1DirectAddressDeferred(void);
static uint32_t bam1DirectAddressDeferred(void);
static uint32_t am1DirectAddressDeferredIndexed(void);
static uint32_t bam1DirectAddressDeferredIndexed(void);
static uint32_t am1Immediate(void);
static uint32_t am1ImmediateQuick(void);
static uint32_t am1Error1(void);
static uint32_t bam1Error1(void);
static uint32_t am1Error2(void);
static uint32_t bam1Error2(void);
static uint32_t am1Error3(void);
static uint32_t bam1Error3(void);
static uint32_t am1Error4(void);
static uint32_t bam1Error4(void);
static uint32_t am1Error5(void);
static uint32_t bam1Error5(void);
static uint32_t bam1Error6(void);
static uint32_t am1Group7a(void);
static uint32_t bam1Group7a(void);
static uint32_t am1Group6(void);
static uint32_t bam1Group6(void);
static uint32_t am1Group7(void);
static uint32_t bam1Group7(void);
static const am_func s_AMTable1_G7a[16];
static const am_func s_BAMTable1_G7a[16];
static const am_func s_AMTable1_G7[32];
static const am_func s_BAMTable1_G7[32];
static const am_func s_AMTable1_G6[8];
static const am_func s_BAMTable1_G6[8];
static const am_func s_AMTable1[2][8];
static const am_func s_BAMTable1[2][8];
static uint32_t am2Register(void);
static uint32_t am2RegisterIndirect(void);
static uint32_t bam2RegisterIndirect(void);
static uint32_t am2RegisterIndirectIndexed(void);
static uint32_t bam2RegisterIndirectIndexed(void);
static uint32_t am2Autoincrement(void);
static uint32_t bam2Autoincrement(void);
static uint32_t am2Autodecrement(void);
static uint32_t bam2Autodecrement(void);
static uint32_t am2Displacement8(void);
static uint32_t bam2Displacement8(void);
static uint32_t am2Displacement16(void);
static uint32_t bam2Displacement16(void);
static uint32_t am2Displacement32(void);
static uint32_t bam2Displacement32(void);
static uint32_t am2DisplacementIndexed8(void);
static uint32_t bam2DisplacementIndexed8(void);
static uint32_t am2DisplacementIndexed16(void);
static uint32_t bam2DisplacementIndexed16(void);
static uint32_t am2DisplacementIndexed32(void);
static uint32_t bam2DisplacementIndexed32(void);
static uint32_t am2PCDisplacement8(void);
static uint32_t bam2PCDisplacement8(void);
static uint32_t am2PCDisplacement16(void);
static uint32_t bam2PCDisplacement16(void);
static uint32_t am2PCDisplacement32(void);
static uint32_t bam2PCDisplacement32(void);
static uint32_t am2PCDisplacementIndexed8(void);
static uint32_t bam2PCDisplacementIndexed8(void);
static uint32_t am2PCDisplacementIndexed16(void);
static uint32_t bam2PCDisplacementIndexed16(void);
static uint32_t am2PCDisplacementIndexed32(void);
static uint32_t bam2PCDisplacementIndexed32(void);
static uint32_t am2DisplacementIndirect8(void);
static uint32_t bam2DisplacementIndirect8(void);
static uint32_t am2DisplacementIndirect16(void);
static uint32_t bam2DisplacementIndirect16(void);
static uint32_t am2DisplacementIndirect32(void);
static uint32_t bam2DisplacementIndirect32(void);
static uint32_t am2DisplacementIndirectIndexed8(void);
static uint32_t bam2DisplacementIndirectIndexed8(void);
static uint32_t am2DisplacementIndirectIndexed16(void);
static uint32_t bam2DisplacementIndirectIndexed16(void);
static uint32_t am2DisplacementIndirectIndexed32(void);
static uint32_t bam2DisplacementIndirectIndexed32(void);
static uint32_t am2PCDisplacementIndirect8(void);
static uint32_t bam2PCDisplacementIndirect8(void);
static uint32_t am2PCDisplacementIndirect16(void);
static uint32_t bam2PCDisplacementIndirect16(void);
static uint32_t am2PCDisplacementIndirect32(void);
static uint32_t bam2PCDisplacementIndirect32(void);
static uint32_t am2PCDisplacementIndirectIndexed8(void);
static uint32_t bam2PCDisplacementIndirectIndexed8(void);
static uint32_t am2PCDisplacementIndirectIndexed16(void);
static uint32_t bam2PCDisplacementIndirectIndexed16(void);
static uint32_t am2PCDisplacementIndirectIndexed32(void);
static uint32_t bam2PCDisplacementIndirectIndexed32(void);
static uint32_t am2DoubleDisplacement8(void);
static uint32_t bam2DoubleDisplacement8(void);
static uint32_t am2DoubleDisplacement16(void);
static uint32_t bam2DoubleDisplacement16(void);
static uint32_t am2DoubleDisplacement32(void);
static uint32_t bam2DoubleDisplacement32(void);
static uint32_t am2PCDoubleDisplacement8(void);
static uint32_t bam2PCDoubleDisplacement8(void);
static uint32_t am2PCDoubleDisplacement16(void);
static uint32_t bam2PCDoubleDisplacement16(void);
static uint32_t am2PCDoubleDisplacement32(void);
static uint32_t bam2PCDoubleDisplacement32(void);
static uint32_t am2DirectAddress(void);
static uint32_t bam2DirectAddress(void);
static uint32_t am2DirectAddressIndexed(void);
static uint32_t bam2DirectAddressIndexed(void);
static uint32_t am2DirectAddressDeferred(void);
static uint32_t bam2DirectAddressDeferred(void);
static uint32_t am2DirectAddressDeferredIndexed(void);
static uint32_t bam2DirectAddressDeferredIndexed(void);
static uint32_t am2Immediate(void);
static uint32_t am2ImmediateQuick(void);
static uint32_t am2Error1(void);
static uint32_t am2Error2(void);
static uint32_t am2Error3(void);
static uint32_t am2Error4(void);
static uint32_t am2Error5(void);
static uint32_t bam2Error1(void);
static uint32_t bam2Error2(void);
static uint32_t bam2Error3(void);
static uint32_t bam2Error4(void);
static uint32_t bam2Error5(void);
static uint32_t bam2Error6(void);
static uint32_t am2Group7a(void);
static uint32_t bam2Group7a(void);
static uint32_t am2Group6(void);
static uint32_t bam2Group6(void);
static uint32_t am2Group7(void);
static uint32_t bam2Group7(void);
static const am_func s_AMTable2_G7a[16];
static const am_func s_BAMTable2_G7a[16];
static const am_func s_AMTable2_G7[32];
static const am_func s_BAMTable2_G7[32];
static const am_func s_AMTable2_G6[8];
static const am_func s_BAMTable2_G6[8];
static const am_func s_AMTable2[2][8];
static const am_func s_BAMTable2[2][8];
static uint32_t am3Register(void);
static uint32_t am3RegisterIndirect(void);
static uint32_t am3RegisterIndirectIndexed(void);
static uint32_t am3Autoincrement(void);
static uint32_t am3Autodecrement(void);
static uint32_t am3Displacement8(void);
static uint32_t am3Displacement16(void);
static uint32_t am3Displacement32(void);
static uint32_t am3DisplacementIndexed8(void);
static uint32_t am3DisplacementIndexed16(void);
static uint32_t am3DisplacementIndexed32(void);
static uint32_t am3PCDisplacement8(void);
static uint32_t am3PCDisplacement16(void);
static uint32_t am3PCDisplacement32(void);
static uint32_t am3PCDisplacementIndexed8(void);
static uint32_t am3PCDisplacementIndexed16(void);
static uint32_t am3PCDisplacementIndexed32(void);
static uint32_t am3DisplacementIndirect8(void);
static uint32_t am3DisplacementIndirect16(void);
static uint32_t am3DisplacementIndirect32(void);
static uint32_t am3DisplacementIndirectIndexed8(void);
static uint32_t am3DisplacementIndirectIndexed16(void);
static uint32_t am3DisplacementIndirectIndexed32(void);
static uint32_t am3PCDisplacementIndirect8(void);
static uint32_t am3PCDisplacementIndirect16(void);
static uint32_t am3PCDisplacementIndirect32(void);
static uint32_t am3PCDisplacementIndirectIndexed8(void);
static uint32_t am3PCDisplacementIndirectIndexed16(void);
static uint32_t am3PCDisplacementIndirectIndexed32(void);
static uint32_t am3DoubleDisplacement8(void);
static uint32_t am3DoubleDisplacement16(void);
static uint32_t am3DoubleDisplacement32(void);
static uint32_t am3PCDoubleDisplacement8(void);
static uint32_t am3PCDoubleDisplacement16(void);
static uint32_t am3PCDoubleDisplacement32(void);
static uint32_t am3DirectAddress(void);
static uint32_t am3DirectAddressIndexed(void);
static uint32_t am3DirectAddressDeferred(void);
static uint32_t am3DirectAddressDeferredIndexed(void);
static uint32_t am3Immediate(void);
static uint32_t am3ImmediateQuick(void);
static uint32_t am3Error1(void);
static uint32_t am3Error2(void);
static uint32_t am3Error3(void);
static uint32_t am3Error4(void);
static uint32_t am3Error5(void);
static uint32_t am3Group7a(void);
static uint32_t am3Group6(void);
static uint32_t am3Group7(void);
static const am_func s_AMTable3_G7a[16];
static const am_func s_AMTable3_G7[32];
static const am_func s_AMTable3_G6[8];
static const am_func s_AMTable3[2][8];
static uint32_t ReadAM(void);
static uint32_t BitReadAM(void);
static uint32_t ReadAMAddress(void);
static uint32_t BitReadAMAddress(void);
static uint32_t WriteAM(void);
static void F12DecodeFirstOperand(am_func DecodeOp1, uint8_t dim1);
static void F12WriteSecondOperand(uint8_t dim2);
static void F12DecodeOperands(am_func DecodeOp1, uint8_t dim1, am_func DecodeOp2, uint8_t dim2);
static uint32_t opADDB(void);
static uint32_t opADDH(void);
static uint32_t opADDW(void);
static uint32_t opADDCB(void);
static uint32_t opADDCH(void);
static uint32_t opADDCW(void);
static uint32_t opANDB(void);
static uint32_t opANDH(void);
static uint32_t opANDW(void);
static uint32_t opCALL(void);
static uint32_t opCHKAR(void);
static uint32_t opCHKAW(void);
static uint32_t opCHKAE(void);
static uint32_t opCHLVL(void);
static uint32_t opCLR1(void);
static uint32_t opCMPB(void);
static uint32_t opCMPH(void);
static uint32_t opCMPW(void);
static uint32_t opDIVB(void);
static uint32_t opDIVH(void);
static uint32_t opDIVW(void);
static uint32_t opDIVX(void);
static uint32_t opDIVUX(void);
static uint32_t opDIVUB(void);
static uint32_t opDIVUH(void);
static uint32_t opDIVUW(void);
static uint32_t opINB(void);
static uint32_t opINH(void);
static uint32_t opINW(void);
static uint32_t opLDPR(void);
static uint32_t opLDTASK(void);
static uint32_t opMOVD(void);
static uint32_t opMOVB(void);
static uint32_t opMOVH(void);
static uint32_t opMOVW(void);
static uint32_t opMOVEAB(void);
static uint32_t opMOVEAH(void);
static uint32_t opMOVEAW(void);
static uint32_t opMOVSBH(void);
static uint32_t opMOVSBW(void);
static uint32_t opMOVSHW(void);
static uint32_t opMOVTHB(void);
static uint32_t opMOVTWB(void);
static uint32_t opMOVTWH(void);
static uint32_t opMOVZBH(void);
static uint32_t opMOVZBW(void);
static uint32_t opMOVZHW(void);
static uint32_t opMULB(void);
static uint32_t opMULH(void);
static uint32_t opMULW(void);
static uint32_t opMULUB(void);
static uint32_t opMULUH(void);
static uint32_t opMULUW(void);
static uint32_t opNEGB(void);
static uint32_t opNEGH(void);
static uint32_t opNEGW(void);
static uint32_t opNOTB(void);
static uint32_t opNOTH(void);
static uint32_t opNOTW(void);
static uint32_t opNOT1(void);
static uint32_t opORB(void);
static uint32_t opORH(void);
static uint32_t opORW(void);
static uint32_t opOUTB(void);
static uint32_t opOUTH(void);
static uint32_t opOUTW(void);
static uint32_t opREMB(void);
static uint32_t opREMH(void);
static uint32_t opREMW(void);
static uint32_t opREMUB(void);
static uint32_t opREMUH(void);
static uint32_t opREMUW(void);
static uint32_t opROTB(void);
static uint32_t opROTH(void);
static uint32_t opROTW(void);
static uint32_t opROTCB(void);
static uint32_t opROTCH(void);
static uint32_t opROTCW(void);
static uint32_t opRVBIT(void);
static uint32_t opRVBYT(void);
static uint32_t opSET1(void);
static uint32_t opSETF(void);
static uint32_t opSHAB(void);
static uint32_t opSHAH(void);
static uint32_t opSHAW(void);
static uint32_t opSHLB(void);
static uint32_t opSHLH(void);
static uint32_t opSHLW(void);
static uint32_t opSTPR(void);
static uint32_t opSUBB(void);
static uint32_t opSUBH(void);
static uint32_t opSUBW(void);
static uint32_t opSUBCB(void);
static uint32_t opSUBCH(void);
static uint32_t opSUBCW(void);
static uint32_t opTEST1(void);
static uint32_t opUPDPSWW(void);
static uint32_t opUPDPSWH(void);
static uint32_t opXCHB(void);
static uint32_t opXCHH(void);
static uint32_t opXCHW(void);
static uint32_t opXORB(void);
static uint32_t opXORH(void);
static uint32_t opXORW(void);
static uint32_t opMULX(void);
static uint32_t opMULUX(void);
static void F2DecodeFirstOperand(am_func DecodeOp1, uint8_t dim1);
static void F2DecodeSecondOperand(am_func DecodeOp2, uint8_t dim2);
static void F2WriteSecondOperand(uint8_t dim2);
static uint32_t opCVTWS(void);
static uint32_t opCVTSW(void);
static uint32_t opMOVFS(void);
static uint32_t opNEGFS(void);
static uint32_t opABSFS(void);
static uint32_t opADDFS(void);
static uint32_t opSUBFS(void);
static uint32_t opMULFS(void);
static uint32_t opDIVFS(void);
static uint32_t opSCLFS(void);
static uint32_t opCMPF(void);
static uint32_t op5FUNHANDLED(void);
static uint32_t op5CUNHANDLED(void);
static uint32_t op5F(void);
static uint32_t op5C(void);
static const am_func s_Op5FTable[32];
static const am_func s_Op5CTable[32];
static uint32_t opINCB(void);
static uint32_t opINCH(void);
static uint32_t opINCW(void);
static uint32_t opDECB(void);
static uint32_t opDECH(void);
static uint32_t opDECW(void);
static uint32_t opJMP(void);
static uint32_t opJSR(void);
static uint32_t opPREPARE(void);
static uint32_t opRET(void);
static uint32_t opTRAP(void);
static uint32_t opRETIU(void);
static uint32_t opRETIS(void);
static uint32_t opSTTASK(void);
static uint32_t opGETPSW(void);
static uint32_t opTASI(void);
static uint32_t opCLRTLB(void);
static uint32_t opPOPM(void);
static uint32_t opPUSHM(void);
static uint32_t opTESTB(void);
static uint32_t opTESTH(void);
static uint32_t opTESTW(void);
static uint32_t opPUSH(void);
static uint32_t opPOP(void);
static uint32_t opINCB_0(void);
static uint32_t opINCB_1(void);
static uint32_t opINCH_0(void);
static uint32_t opINCH_1(void);
static uint32_t opINCW_0(void);
static uint32_t opINCW_1(void);
static uint32_t opDECB_0(void);
static uint32_t opDECB_1(void);
static uint32_t opDECH_0(void);
static uint32_t opDECH_1(void);
static uint32_t opDECW_0(void);
static uint32_t opDECW_1(void);
static uint32_t opJMP_0(void);
static uint32_t opJMP_1(void);
static uint32_t opJSR_0(void);
static uint32_t opJSR_1(void);
static uint32_t opPREPARE_0(void);
static uint32_t opPREPARE_1(void);
static uint32_t opRET_0(void);
static uint32_t opRET_1(void);
static uint32_t opTRAP_0(void);
static uint32_t opTRAP_1(void);
static uint32_t opRETIU_0(void);
static uint32_t opRETIU_1(void);
static uint32_t opRETIS_0(void);
static uint32_t opRETIS_1(void);
static uint32_t opGETPSW_0(void);
static uint32_t opGETPSW_1(void);
static uint32_t opTASI_0(void);
static uint32_t opTASI_1(void);
static uint32_t opCLRTLB_0(void);
static uint32_t opCLRTLB_1(void);
static uint32_t opPOPM_0(void);
static uint32_t opPOPM_1(void);
static uint32_t opPUSHM_0(void);
static uint32_t opPUSHM_1(void);
static uint32_t opTESTB_0(void);
static uint32_t opTESTB_1(void);
static uint32_t opTESTH_0(void);
static uint32_t opTESTH_1(void);
static uint32_t opTESTW_0(void);
static uint32_t opTESTW_1(void);
static uint32_t opPUSH_0(void);
static uint32_t opPUSH_1(void);
static uint32_t opPOP_0(void);
static uint32_t opPOP_1(void);
static uint32_t opSTTASK_0(void);
static uint32_t opSTTASK_1(void);
static uint32_t opBGT8(void);
static uint32_t opBGT16(void);
static uint32_t opBGE8(void);
static uint32_t opBGE16(void);
static uint32_t opBLT8(void);
static uint32_t opBLT16(void);
static uint32_t opBLE8(void);
static uint32_t opBLE16(void);
static uint32_t opBH8(void);
static uint32_t opBH16(void);
static uint32_t opBNH8(void);
static uint32_t opBNH16(void);
static uint32_t opBNL8(void);
static uint32_t opBNL16(void);
static uint32_t opBL8(void);
static uint32_t opBL16(void);
static uint32_t opBNE8(void);
static uint32_t opBNE16(void);
static uint32_t opBE8(void);
static uint32_t opBE16(void);
static uint32_t opBNV8(void);
static uint32_t opBNV16(void);
static uint32_t opBV8(void);
static uint32_t opBV16(void);
static uint32_t opBP8(void);
static uint32_t opBP16(void);
static uint32_t opBN8(void);
static uint32_t opBN16(void);
static uint32_t opBR8(void);
static uint32_t opBR16(void);
static uint32_t opBSR(void);
static uint32_t opBRK(void);
static uint32_t opBRKV(void);
static uint32_t opCLRTLBA(void);
static uint32_t opDISPOSE(void);
static uint32_t opHALT(void);
static uint32_t opNOP(void);
static uint32_t opRSR(void);
static uint32_t opTRAPFL(void);
static uint32_t opTB(int reg);
static uint32_t opDBGT(int reg);
static uint32_t opDBLE(int reg);
static uint32_t opDBGE(int reg);
static uint32_t opDBLT(int reg);
static uint32_t opDBH(int reg);
static uint32_t opDBNH(int reg);
static uint32_t opDBL(int reg);
static uint32_t opDBNL(int reg);
static uint32_t opDBE(int reg);
static uint32_t opDBNE(int reg);
static uint32_t opDBV(int reg);
static uint32_t opDBNV(int reg);
static uint32_t opDBN(int reg);
static uint32_t opDBP(int reg);
static uint32_t opDBR(int reg);
static uint32_t opC6(void);
static uint32_t opC7(void);
static const op6_func s_OpC6Table[8];
static const op6_func s_OpC7Table[8];
static void F7aDecodeOperands(am_func DecodeOp1, uint8_t dim1, am_func DecodeOp2, uint8_t dim2);
static void F7bDecodeFirstOperand(am_func DecodeOp1, uint8_t dim1);
static void F7bWriteSecondOperand(uint8_t dim2);
static void F7bDecodeOperands(am_func DecodeOp1, uint8_t dim1, am_func DecodeOp2, uint8_t dim2);
static void F7cDecodeOperands(am_func DecodeOp1, uint8_t dim1, am_func DecodeOp2, uint8_t dim2);
static uint32_t opCMPSTRB(uint8_t bFill, uint8_t bStop);
static uint32_t opCMPSTRH(uint8_t bFill, uint8_t bStop);
static uint32_t opMOVSTRUB(uint8_t bFill, uint8_t bStop);
static uint32_t opMOVSTRDB(uint8_t bFill, uint8_t bStop);
static uint32_t opMOVSTRUH(uint8_t bFill, uint8_t bStop);
static uint32_t opMOVSTRDH(uint8_t bFill, uint8_t bStop);
static uint32_t opSEARCHUB(uint8_t bSearch);
static uint32_t opSEARCHUH(uint8_t bSearch);
static uint32_t opSEARCHDB(uint8_t bSearch);
static uint32_t opSEARCHDH(uint8_t bSearch);
static uint32_t opSCHCUB(void);
static uint32_t opSCHCUH(void);
static uint32_t opSCHCDB(void);
static uint32_t opSCHCDH(void);
static uint32_t opSKPCUB(void);
static uint32_t opSKPCUH(void);
static uint32_t opSKPCDB(void);
static uint32_t opSKPCDH(void);
static uint32_t opCMPCB(void);
static uint32_t opCMPCH(void);
static uint32_t opCMPCFB(void);
static uint32_t opCMPCFH(void);
static uint32_t opCMPCSB(void);
static uint32_t opCMPCSH(void);
static uint32_t opMOVCUB(void);
static uint32_t opMOVCUH(void);
static uint32_t opMOVCFUB(void);
static uint32_t opMOVCFUH(void);
static uint32_t opMOVCSUB(void);
static uint32_t opMOVCSUH(void);
static uint32_t opMOVCDB(void);
static uint32_t opMOVCDH(void);
static uint32_t opMOVCFDB(void);
static uint32_t opMOVCFDH(void);
static uint32_t opEXTBFZ(void);
static uint32_t opEXTBFS(void);
static uint32_t opEXTBFL(void);
static uint32_t opSCHBS(uint32_t bSearch1);
static uint32_t opSCH0BSU(void);
static uint32_t opSCH1BSU(void);
static uint32_t opINSBFR(void);
static uint32_t opINSBFL(void);
static uint32_t opMOVBSD(void);
static uint32_t opMOVBSU(void);
static uint32_t opADDDC(void);
static uint32_t opSUBDC(void);
static uint32_t opSUBRDC(void);
static uint32_t opCVTDPZ(void);
static uint32_t opCVTDZP(void);
static uint32_t op58UNHANDLED(void);
static uint32_t op5AUNHANDLED(void);
static uint32_t op5BUNHANDLED(void);
static uint32_t op5DUNHANDLED(void);
static uint32_t op59UNHANDLED(void);
static uint32_t op58(void);
static uint32_t op5A(void);
static uint32_t op5B(void);
static uint32_t op5D(void);
static uint32_t op59(void);
static const am_func s_Op59Table[32];
static const am_func s_Op5BTable[32];
static const am_func s_Op5DTable[32];
static const am_func s_Op58Table[32];
static const am_func s_Op5ATable[32];
static uint32_t opUNHANDLED(void);
static const am_func s_OpCodeTable[256];

// ---- am1.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
// AM1 Functions (for ReadAM)
// **************************

static uint32_t am1Register(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = (uint8_t)m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = (uint16_t)m_reg[m_modval & 0x1F];
		break;
	case 2:
		m_amout = m_reg[m_modval & 0x1F];
		break;
	}

	return 1;
}

static uint32_t am1RegisterIndirect(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(m_reg[m_modval & 0x1F]);
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval & 0x1F]);
		break;
	}

	return 1;
}

static uint32_t bam1RegisterIndirect(void)
{
	m_bamoffset = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F]);
	return 1;
}

static uint32_t am1RegisterIndirectIndexed(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(m_reg[m_modval2 & 0x1F] + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(m_reg[m_modval2 & 0x1F] + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 2;
}

static uint32_t bam1RegisterIndirectIndexed(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + m_bamoffset / 8);
	m_bamoffset&=7;
	return 2;
}

static uint32_t am1Autoincrement(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(m_reg[m_modval & 0x1F]);
		m_reg[m_modval & 0x1F]++;
		break;
	case 1:
		m_amout = V60_RW(m_reg[m_modval & 0x1F]);
		m_reg[m_modval & 0x1F] +=2;
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval & 0x1F]);
		m_reg[m_modval & 0x1F] +=4;
		break;
	}

	return 1;
}

static uint32_t bam1Autoincrement(void)
{
	m_bamoffset = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F]);
	switch (m_moddim)
	{
	case 10:
		m_reg[m_modval & 0x1F] +=1;
		break;
	case 11:
		m_reg[m_modval & 0x1F] +=4;
		break;
	default:
		return v60_fatal("CPU - BAM1 - 7\n");
		break;
	}
	return 1;
}

static uint32_t am1Autodecrement(void)
{
	switch (m_moddim)
	{
	case 0:
		m_reg[m_modval & 0x1F]--;
		m_amout = V60_RB(m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_reg[m_modval & 0x1F]-=2;
		m_amout = V60_RW(m_reg[m_modval & 0x1F]);
		break;
	case 2:
		m_reg[m_modval & 0x1F]-=4;
		m_amout = V60_RD(m_reg[m_modval & 0x1F]);
		break;
	}

	return 1;
}

static uint32_t bam1Autodecrement(void)
{
	m_bamoffset = 0;
	switch (m_moddim)
	{
	case 10:
		m_reg[m_modval & 0x1F]-=1;
		break;
	case 11:
		m_reg[m_modval & 0x1F]-=4;
		break;
	default:
		return v60_fatal("CPU - BAM1 - 7\n");
		break;
	}
	m_amout = V60_RD(m_reg[m_modval & 0x1F]);
	return 1;
}

static uint32_t am1Displacement8(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1));
		break;
	case 1:
		m_amout = V60_RW(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1));
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1));
		break;
	}

	return 2;
}

static uint32_t bam1Displacement8(void)
{
	m_bamoffset = V60_RB(m_modadd + 1);
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + m_bamoffset / 8);
	m_bamoffset&=7;
	return 2;
}


static uint32_t am1Displacement16(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1));
		break;
	case 1:
		m_amout = V60_RW(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1));
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1));
		break;
	}

	return 3;
}

static uint32_t bam1Displacement16(void)
{
	m_bamoffset = OpRead16(m_modadd + 1);
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + m_bamoffset / 8);
	m_bamoffset&=7;
	return 3;
}

static uint32_t am1Displacement32(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1));
		break;
	case 1:
		m_amout = V60_RW(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1));
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1));
		break;
	}

	return 5;
}

static uint32_t bam1Displacement32(void)
{
	m_bamoffset = OpRead32(m_modadd + 1);
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + m_bamoffset / 8);
	m_bamoffset&=7;
	return 5;
}

static uint32_t am1DisplacementIndexed8(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 3;
}

static uint32_t bam1DisplacementIndexed8(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 3;
}

static uint32_t am1DisplacementIndexed16(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 4;
}

static uint32_t bam1DisplacementIndexed16(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 4;
}

static uint32_t am1DisplacementIndexed32(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 6;
}

static uint32_t bam1DisplacementIndexed32(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 6;
}


static uint32_t am1PCDisplacement8(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(PC + (int8_t)OpRead8(m_modadd + 1));
		break;
	case 1:
		m_amout = V60_RW(PC + (int8_t)OpRead8(m_modadd + 1));
		break;
	case 2:
		m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 1));
		break;
	}

	return 2;
}

static uint32_t bam1PCDisplacement8(void)
{
	m_bamoffset = OpRead8(m_modadd + 1);
	m_amout = V60_RD(PC + m_bamoffset / 8);
	m_bamoffset&=7;
	return 2;
}

static uint32_t am1PCDisplacement16(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(PC + (int16_t)OpRead16(m_modadd + 1));
		break;
	case 1:
		m_amout = V60_RW(PC + (int16_t)OpRead16(m_modadd + 1));
		break;
	case 2:
		m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 1));
		break;
	}

	return 3;
}

static uint32_t bam1PCDisplacement16(void)
{
	m_bamoffset = OpRead16(m_modadd + 1);
	m_amout = V60_RD(PC + m_bamoffset / 8);
	m_bamoffset&=7;
	return 3;
}

static uint32_t am1PCDisplacement32(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(PC + OpRead32(m_modadd + 1));
		break;
	case 1:
		m_amout = V60_RW(PC + OpRead32(m_modadd + 1));
		break;
	case 2:
		m_amout = V60_RD(PC + OpRead32(m_modadd + 1));
		break;
	}

	return 5;
}

static uint32_t bam1PCDisplacement32(void)
{
	m_bamoffset = OpRead32(m_modadd + 1);
	m_amout = V60_RD(PC + m_bamoffset / 8);
	m_bamoffset&=7;
	return 5;
}

static uint32_t am1PCDisplacementIndexed8(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(PC + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(PC + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 3;
}

static uint32_t bam1PCDisplacementIndexed8(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 2) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 3;
}


static uint32_t am1PCDisplacementIndexed16(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(PC + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(PC + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 4;
}

static uint32_t bam1PCDisplacementIndexed16(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 2) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 4;
}

static uint32_t am1PCDisplacementIndexed32(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(PC + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(PC + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(PC + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 6;
}

static uint32_t bam1PCDisplacementIndexed32(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(PC + OpRead32(m_modadd + 2) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 6;
}

static uint32_t am1DisplacementIndirect8(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)));
		break;
	case 1:
		m_amout = V60_RW(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)));
		break;
	case 2:
		m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)));
		break;
	}

	return 2;
}

static uint32_t bam1DisplacementIndirect8(void)
{
	m_bamoffset = 0;
	m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)));
	return 2;
}

static uint32_t am1DisplacementIndirect16(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)));
		break;
	case 1:
		m_amout = V60_RW(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)));
		break;
	case 2:
		m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)));
		break;
	}

	return 3;
}

static uint32_t bam1DisplacementIndirect16(void)
{
	m_bamoffset = 0;
	m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)));
	return 3;
}

static uint32_t am1DisplacementIndirect32(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)));
		break;
	case 1:
		m_amout = V60_RW(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)));
		break;
	case 2:
		m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)));
		break;
	}

	return 5;
}

static uint32_t bam1DisplacementIndirect32(void)
{
	m_bamoffset = 0;
	m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)));
	return 5;
}

static uint32_t am1DisplacementIndirectIndexed8(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 3;
}

static uint32_t bam1DisplacementIndirectIndexed8(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 3;
}

static uint32_t am1DisplacementIndirectIndexed16(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 4;
}

static uint32_t bam1DisplacementIndirectIndexed16(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 4;
}

static uint32_t am1DisplacementIndirectIndexed32(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 6;
}

static uint32_t bam1DisplacementIndirectIndexed32(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 6;
}

static uint32_t am1PCDisplacementIndirect8(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)));
		break;
	case 1:
		m_amout = V60_RW(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)));
		break;
	case 2:
		m_amout = V60_RD(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)));
		break;
	}

	return 2;
}

static uint32_t bam1PCDisplacementIndirect8(void)
{
	m_bamoffset = 0;
	m_amout = V60_RD(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)));
	return 2;
}

static uint32_t am1PCDisplacementIndirect16(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)));
		break;
	case 1:
		m_amout = V60_RW(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)));
		break;
	case 2:
		m_amout = V60_RD(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)));
		break;
	}

	return 3;
}

static uint32_t bam1PCDisplacementIndirect16(void)
{
	m_bamoffset = 0;
	m_amout = V60_RD(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)));
	return 3;
}

static uint32_t am1PCDisplacementIndirect32(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(PC + OpRead32(m_modadd + 1)));
		break;
	case 1:
		m_amout = V60_RW(V60_RD(PC + OpRead32(m_modadd + 1)));
		break;
	case 2:
		m_amout = V60_RD(V60_RD(PC + OpRead32(m_modadd + 1)));
		break;
	}

	return 5;
}

static uint32_t bam1PCDisplacementIndirect32(void)
{
	m_bamoffset = 0;
	m_amout = V60_RD(V60_RD(PC + OpRead32(m_modadd + 1)));
	return 5;
}

static uint32_t am1PCDisplacementIndirectIndexed8(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 3;
}

static uint32_t bam1PCDisplacementIndirectIndexed8(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 3;
}

static uint32_t am1PCDisplacementIndirectIndexed16(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 4;
}

static uint32_t bam1PCDisplacementIndirectIndexed16(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 4;
}

static uint32_t am1PCDisplacementIndirectIndexed32(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(PC + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F]);
		break;
	case 1:
		m_amout = V60_RW(V60_RD(PC + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2);
		break;
	case 2:
		m_amout = V60_RD(V60_RD(PC + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 6;
}

static uint32_t bam1PCDisplacementIndirectIndexed32(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(V60_RD(PC + OpRead32(m_modadd + 2)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 6;
}

static uint32_t am1DoubleDisplacement8(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2));
		break;

	case 1:
		m_amout = V60_RW(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2));
		break;

	case 2:
		m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2));
		break;
	}

	return 3;
}

static uint32_t bam1DoubleDisplacement8(void)
{
	m_bamoffset = OpRead8(m_modadd + 2);
	m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 3;
}

static uint32_t am1DoubleDisplacement16(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3));
		break;

	case 1:
		m_amout = V60_RW(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3));
		break;

	case 2:
		m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3));
		break;
	}

	return 5;
}

static uint32_t bam1DoubleDisplacement16(void)
{
	m_bamoffset = OpRead16(m_modadd + 3);
	m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 5;
}

static uint32_t am1DoubleDisplacement32(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5));
		break;

	case 1:
		m_amout = V60_RW(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5));
		break;

	case 2:
		m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5));
		break;
	}

	return 9;
}

static uint32_t bam1DoubleDisplacement32(void)
{
	m_bamoffset = OpRead32(m_modadd + 5);
	m_amout = V60_RD(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 9;
}

static uint32_t am1PCDoubleDisplacement8(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2));
		break;

	case 1:
		m_amout = V60_RW(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2));
		break;

	case 2:
		m_amout = V60_RD(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2));
		break;
	}

	return 3;
}

static uint32_t bam1PCDoubleDisplacement8(void)
{
	m_bamoffset = OpRead8(m_modadd + 2);
	m_amout = V60_RD(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 3;
}

static uint32_t am1PCDoubleDisplacement16(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3));
		break;

	case 1:
		m_amout = V60_RW(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3));
		break;

	case 2:
		m_amout = V60_RD(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3));
		break;
	}

	return 5;
}

static uint32_t bam1PCDoubleDisplacement16(void)
{
	m_bamoffset = OpRead16(m_modadd + 3);
	m_amout = V60_RD(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 5;
}


static uint32_t am1PCDoubleDisplacement32(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(PC + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5));
		break;

	case 1:
		m_amout = V60_RW(V60_RD(PC + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5));
		break;

	case 2:
		m_amout = V60_RD(V60_RD(PC + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5));
		break;
	}

	return 9;
}

static uint32_t bam1PCDoubleDisplacement32(void)
{
	m_bamoffset = OpRead32(m_modadd + 5);
	m_amout = V60_RD(V60_RD(PC + OpRead32(m_modadd + 1)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 9;
}

static uint32_t am1DirectAddress(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(OpRead32(m_modadd + 1));
		break;

	case 1:
		m_amout = V60_RW(OpRead32(m_modadd + 1));
		break;

	case 2:
		m_amout = V60_RD(OpRead32(m_modadd + 1));
		break;
	}

	return 5;
}

static uint32_t bam1DirectAddress(void)
{
	m_bamoffset = 0;
	m_amout = V60_RD(OpRead32(m_modadd + 1));
	return 5;
}

static uint32_t am1DirectAddressIndexed(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F]);
		break;

	case 1:
		m_amout = V60_RW(OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2);
		break;

	case 2:
		m_amout = V60_RD(OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 6;
}

static uint32_t bam1DirectAddressIndexed(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(OpRead32(m_modadd + 2) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 6;
}

static uint32_t am1DirectAddressDeferred(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(OpRead32(m_modadd + 1)));
		break;

	case 1:
		m_amout = V60_RW(V60_RD(OpRead32(m_modadd + 1)));
		break;

	case 2:
		m_amout = V60_RD(V60_RD(OpRead32(m_modadd + 1)));
		break;
	}

	return 5;
}

static uint32_t bam1DirectAddressDeferred(void)
{
	m_bamoffset = 0;
	m_amout = V60_RD(V60_RD(OpRead32(m_modadd + 1)));
	return 5;
}

static uint32_t am1DirectAddressDeferredIndexed(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RB(V60_RD(OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F]);
		break;

	case 1:
		m_amout = V60_RW(V60_RD(OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2);
		break;

	case 2:
		m_amout = V60_RD(V60_RD(OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4);
		break;
	}

	return 6;
}

static uint32_t bam1DirectAddressDeferredIndexed(void)
{
	m_bamoffset = m_reg[m_modval & 0x1F];
	m_amout = V60_RD(V60_RD(OpRead32(m_modadd + 2)) + m_bamoffset / 8);
	m_bamoffset&=7;
	return 6;
}

static uint32_t am1Immediate(void)
{
	switch (m_moddim)
	{
	case 0:
		m_amout = OpRead8(m_modadd + 1);
		return 2;

	case 1:
		m_amout = OpRead16(m_modadd + 1);
		return 3;

	case 2:
		m_amout = OpRead32(m_modadd + 1);
		return 5;
	}

	// It should not be here!  Written to avoid warning
	assert(0);
	return 1;
}

static uint32_t am1ImmediateQuick(void)
{
	m_amout = m_modval & 0xF;
	return 1;
}




// AM1 Tables (for ReadAM)
// ***********************

static uint32_t am1Error1(void)
{
	return v60_fatal("CPU - AM1 - 1 (PC=%06x)\n", PC);
}

static uint32_t bam1Error1(void)
{
	return v60_fatal("CPU - BAM1 - 1 (PC=%06x)\n", PC);
}

static uint32_t am1Error2(void)
{
	return v60_fatal("CPU - AM1 - 2 (PC=%06x)\n", PC);
}

static uint32_t bam1Error2(void)
{
	return v60_fatal("CPU - BAM1 - 2 (PC=%06x)\n", PC);
}

static uint32_t am1Error3(void)
{
	return v60_fatal("CPU - AM1 - 3 (PC=%06x)\n", PC);
}

static uint32_t bam1Error3(void)
{
	return v60_fatal("CPU - BAM1 - 3 (PC=%06x)\n", PC);
}

static uint32_t am1Error4(void)
{
	return v60_fatal("CPU - AM1 - 4 (PC=%06x)\n", PC);
}

static uint32_t bam1Error4(void)
{
	return v60_fatal("CPU - BAM1 - 4 (PC=%06x)\n", PC);
}

static uint32_t am1Error5(void)
{
	return v60_fatal("CPU - AM1 - 5 (PC=%06x)\n", PC);
}

static uint32_t bam1Error5(void)
{
	return v60_fatal("CPU - BAM1 - 5 (PC=%06x)\n", PC);
}

static uint32_t bam1Error6(void)
{
	return v60_fatal("CPU - BAM1 - 6 (PC=%06x)\n", PC);
}

static const am_func s_AMTable1_G7a[16] =
{
	am1PCDisplacementIndexed8,
	am1PCDisplacementIndexed16,
	am1PCDisplacementIndexed32,
	am1DirectAddressIndexed,
	am1Error5,
	am1Error5,
	am1Error5,
	am1Error5,
	am1PCDisplacementIndirectIndexed8,
	am1PCDisplacementIndirectIndexed16,
	am1PCDisplacementIndirectIndexed32,
	am1DirectAddressDeferredIndexed,
	am1Error5,
	am1Error5,
	am1Error5,
	am1Error5
};

static const am_func s_BAMTable1_G7a[16] =
{
	bam1PCDisplacementIndexed8,
	bam1PCDisplacementIndexed16,
	bam1PCDisplacementIndexed32,
	bam1DirectAddressIndexed,
	bam1Error5,
	bam1Error5,
	bam1Error5,
	bam1Error5,
	bam1PCDisplacementIndirectIndexed8,
	bam1PCDisplacementIndirectIndexed16,
	bam1PCDisplacementIndirectIndexed32,
	bam1DirectAddressDeferredIndexed,
	bam1Error5,
	bam1Error5,
	bam1Error5,
	bam1Error5
};


static uint32_t am1Group7a(void)
{
	if (!(m_modval2 & 0x10))
		return am1Error4();

	return (s_AMTable1_G7a[m_modval2 & 0xF])();
}

static uint32_t bam1Group7a(void)
{
	if (!(m_modval2 & 0x10))
		return bam1Error4();

	return (s_BAMTable1_G7a[m_modval2 & 0xF])();
}

static const am_func s_AMTable1_G7[32] =
{
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1ImmediateQuick,
	am1PCDisplacement8,
	am1PCDisplacement16,
	am1PCDisplacement32,
	am1DirectAddress,
	am1Immediate,
	am1Error2,
	am1Error2,
	am1Error2,
	am1PCDisplacementIndirect8,
	am1PCDisplacementIndirect16,
	am1PCDisplacementIndirect32,
	am1DirectAddressDeferred,
	am1PCDoubleDisplacement8,
	am1PCDoubleDisplacement16,
	am1PCDoubleDisplacement32,
	am1Error2
};

static const am_func s_BAMTable1_G7[32] =
{
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1Error6,
	bam1PCDisplacement8,
	bam1PCDisplacement16,
	bam1PCDisplacement32,
	bam1DirectAddress,
	bam1Error6,
	bam1Error2,
	bam1Error2,
	bam1Error2,
	bam1PCDisplacementIndirect8,
	bam1PCDisplacementIndirect16,
	bam1PCDisplacementIndirect32,
	bam1DirectAddressDeferred,
	bam1PCDoubleDisplacement8,
	bam1PCDoubleDisplacement16,
	bam1PCDoubleDisplacement32,
	bam1Error2
};



static const am_func s_AMTable1_G6[8] =
{
	am1DisplacementIndexed8,
	am1DisplacementIndexed16,
	am1DisplacementIndexed32,
	am1RegisterIndirectIndexed,
	am1DisplacementIndirectIndexed8,
	am1DisplacementIndirectIndexed16,
	am1DisplacementIndirectIndexed32,
	am1Group7a
};

static const am_func s_BAMTable1_G6[8] =
{
	bam1DisplacementIndexed8,
	bam1DisplacementIndexed16,
	bam1DisplacementIndexed32,
	bam1RegisterIndirectIndexed,
	bam1DisplacementIndirectIndexed8,
	bam1DisplacementIndirectIndexed16,
	bam1DisplacementIndirectIndexed32,
	bam1Group7a
};


static uint32_t am1Group6(void)
{
	m_modval2 = OpRead8(m_modadd + 1);
	return (s_AMTable1_G6[m_modval2 >> 5])();
}

static uint32_t bam1Group6(void)
{
	m_modval2 = OpRead8(m_modadd + 1);
	return (s_BAMTable1_G6[m_modval2 >> 5])();
}


static uint32_t am1Group7(void)
{
	return (s_AMTable1_G7[m_modval & 0x1F])();
}

static uint32_t bam1Group7(void)
{
	return (s_BAMTable1_G7[m_modval & 0x1F])();
}

static const am_func s_AMTable1[2][8] =
{
	{
		am1Displacement8,
		am1Displacement16,
		am1Displacement32,
		am1RegisterIndirect,
		am1DisplacementIndirect8,
		am1DisplacementIndirect16,
		am1DisplacementIndirect32,
		am1Group7
	},

	{
		am1DoubleDisplacement8,
		am1DoubleDisplacement16,
		am1DoubleDisplacement32,
		am1Register,
		am1Autoincrement,
		am1Autodecrement,
		am1Group6,
		am1Error1
	}
};


static const am_func s_BAMTable1[2][8] =
{
	{
		bam1Displacement8,
		bam1Displacement16,
		bam1Displacement32,
		bam1RegisterIndirect,
		bam1DisplacementIndirect8,
		bam1DisplacementIndirect16,
		bam1DisplacementIndirect32,
		bam1Group7
	},

	{
		bam1DoubleDisplacement8,
		bam1DoubleDisplacement16,
		bam1DoubleDisplacement32,
		bam1Error6,
		bam1Autoincrement,
		bam1Autodecrement,
		bam1Group6,
		bam1Error1
	}
};

// ---- am2.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
// AM2 Functions (for ReadAMAddress)
// *********************************

static uint32_t am2Register(void)
{
	m_amflag = 1;
	m_amout = m_modval & 0x1F;
	return 1;
}

static uint32_t am2RegisterIndirect(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval & 0x1F];
	return 1;
}

static uint32_t bam2RegisterIndirect(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval & 0x1F];
	m_bamoffset = 0;
	return 1;
}

static uint32_t am2RegisterIndirectIndexed(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = m_reg[m_modval2 & 0x1F] + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = m_reg[m_modval2 & 0x1F] + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = m_reg[m_modval2 & 0x1F] + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = m_reg[m_modval2 & 0x1F] + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 2;
}

static uint32_t bam2RegisterIndirectIndexed(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval2 & 0x1F];
	m_bamoffset = m_reg[m_modval & 0x1F];
	return 2;
}

static uint32_t am2Autoincrement(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval & 0x1F];

	switch (m_moddim)
	{
	case 0:
		m_reg[m_modval & 0x1F] += 1;
		break;
	case 1:
		m_reg[m_modval & 0x1F] += 2;
		break;
	case 2:
		m_reg[m_modval & 0x1F] += 4;
		break;
	case 3:
		m_reg[m_modval & 0x1F] += 8;
		break;
	}

	return 1;
}

static uint32_t bam2Autoincrement(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval & 0x1F];
	m_bamoffset = 0;

	switch (m_moddim)
	{
	case 10:
		m_reg[m_modval & 0x1F] +=1;
		break;
	case 11:
		m_reg[m_modval & 0x1F] +=4;
		break;
	default:
		return v60_fatal("CPU - AM2 - 7 (t0 PC=%x)\n", PC);
		break;
	}

	return 1;
}

static uint32_t am2Autodecrement(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_reg[m_modval & 0x1F] -= 1;
		break;
	case 1:
		m_reg[m_modval & 0x1F] -= 2;
		break;
	case 2:
		m_reg[m_modval & 0x1F] -= 4;
		break;
	case 3:
		m_reg[m_modval & 0x1F] -= 8;
		break;
	}

	m_amout = m_reg[m_modval & 0x1F];
	return 1;
}

static uint32_t bam2Autodecrement(void)
{
	m_amflag = 0;
	m_bamoffset = 0;

	switch (m_moddim)
	{
	case 10:
		m_reg[m_modval & 0x1F]-=1;
		break;
	case 11:
		m_reg[m_modval & 0x1F]-=4;
		break;
	default:
		return v60_fatal("CPU - BAM2 - 7 (PC=%06x)\n", PC);
		break;
	}

	m_amout = m_reg[m_modval & 0x1F];
	return 1;
}


static uint32_t am2Displacement8(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1);

	return 2;
}

static uint32_t bam2Displacement8(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval & 0x1F];
	m_bamoffset = (int8_t)OpRead8(m_modadd + 1);

	return 2;
}

static uint32_t am2Displacement16(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1);

	return 3;
}

static uint32_t bam2Displacement16(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval & 0x1F];
	m_bamoffset = (int16_t)OpRead16(m_modadd + 1);

	return 3;
}

static uint32_t am2Displacement32(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1);

	return 5;
}

static uint32_t bam2Displacement32(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval & 0x1F];
	m_bamoffset = OpRead32(m_modadd + 1);

	return 5;
}

static uint32_t am2DisplacementIndexed8(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 3;
}

static uint32_t bam2DisplacementIndexed8(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2);
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 3;
}

static uint32_t am2DisplacementIndexed16(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 4;
}

static uint32_t bam2DisplacementIndexed16(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2);
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 4;
}

static uint32_t am2DisplacementIndexed32(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 6;
}

static uint32_t bam2DisplacementIndexed32(void)
{
	m_amflag = 0;
	m_amout = m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2);
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 6;
}

static uint32_t am2PCDisplacement8(void)
{
	m_amflag = 0;
	m_amout = PC + (int8_t)OpRead8(m_modadd + 1);

	return 2;
}

static uint32_t bam2PCDisplacement8(void)
{
	m_amflag = 0;
	m_amout = PC;
	m_bamoffset = (int8_t)OpRead8(m_modadd + 1);

	return 2;
}

static uint32_t am2PCDisplacement16(void)
{
	m_amflag = 0;
	m_amout = PC + (int16_t)OpRead16(m_modadd + 1);

	return 3;
}

static uint32_t bam2PCDisplacement16(void)
{
	m_amflag = 0;
	m_amout = PC;
	m_bamoffset = (int16_t)OpRead16(m_modadd + 1);

	return 3;
}

static uint32_t am2PCDisplacement32(void)
{
	m_amflag = 0;
	m_amout = PC + OpRead32(m_modadd + 1);

	return 5;
}

static uint32_t bam2PCDisplacement32(void)
{
	m_amflag = 0;
	m_amout = PC;
	m_bamoffset = OpRead32(m_modadd + 1);

	return 5;
}


static uint32_t am2PCDisplacementIndexed8(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = PC + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = PC + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = PC + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = PC + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 3;
}

static uint32_t bam2PCDisplacementIndexed8(void)
{
	m_amflag = 0;
	m_amout = PC + (int8_t)OpRead8(m_modadd + 2);
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 3;
}

static uint32_t am2PCDisplacementIndexed16(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = PC + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = PC + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = PC + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = PC + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 4;
}

static uint32_t bam2PCDisplacementIndexed16(void)
{
	m_amflag = 0;
	m_amout = PC + (int16_t)OpRead16(m_modadd + 2);
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 4;
}

static uint32_t am2PCDisplacementIndexed32(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = PC + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = PC + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = PC + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = PC + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 6;
}

static uint32_t bam2PCDisplacementIndexed32(void)
{
	m_amflag = 0;
	m_amout = PC + OpRead32(m_modadd + 2);
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 6;
}

static uint32_t am2DisplacementIndirect8(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1));

	return 2;
}

static uint32_t bam2DisplacementIndirect8(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1));
	m_bamoffset = 0;
	return 2;
}

static uint32_t am2DisplacementIndirect16(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1));

	return 3;
}

static uint32_t bam2DisplacementIndirect16(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1));
	m_bamoffset = 0;
	return 3;
}

static uint32_t am2DisplacementIndirect32(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1));

	return 5;
}

static uint32_t bam2DisplacementIndirect32(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1));
	m_bamoffset = 0;

	return 5;
}

static uint32_t am2DisplacementIndirectIndexed8(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 3;
}

static uint32_t bam2DisplacementIndirectIndexed8(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2));
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 3;
}

static uint32_t am2DisplacementIndirectIndexed16(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 4;
}

static uint32_t bam2DisplacementIndirectIndexed16(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2));
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 4;
}

static uint32_t am2DisplacementIndirectIndexed32(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 6;
}

static uint32_t bam2DisplacementIndirectIndexed32(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2));
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 6;
}

static uint32_t am2PCDisplacementIndirect8(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 1));

	return 2;
}

static uint32_t bam2PCDisplacementIndirect8(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 1));
	m_bamoffset = 0;

	return 2;
}

static uint32_t am2PCDisplacementIndirect16(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 1));

	return 3;
}

static uint32_t bam2PCDisplacementIndirect16(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 1));
	m_bamoffset = 0;

	return 3;
}

static uint32_t am2PCDisplacementIndirect32(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + OpRead32(m_modadd + 1));

	return 5;
}

static uint32_t bam2PCDisplacementIndirect32(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + OpRead32(m_modadd + 1));
	m_bamoffset = 0;

	return 5;
}

static uint32_t am2PCDisplacementIndirectIndexed8(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 3;
}

static uint32_t bam2PCDisplacementIndirectIndexed8(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 2));
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 3;
}

static uint32_t am2PCDisplacementIndirectIndexed16(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 4;
}


static uint32_t bam2PCDisplacementIndirectIndexed16(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 2));
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 4;
}


static uint32_t am2PCDisplacementIndirectIndexed32(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RD(PC + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = V60_RD(PC + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = V60_RD(PC + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = V60_RD(PC + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 6;
}

static uint32_t bam2PCDisplacementIndirectIndexed32(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + OpRead32(m_modadd + 2));
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 6;
}

static uint32_t am2DoubleDisplacement8(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2);

	return 3;
}

static uint32_t bam2DoubleDisplacement8(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1));
	m_bamoffset = (int8_t)OpRead8(m_modadd + 2);

	return 3;
}

static uint32_t am2DoubleDisplacement16(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3);

	return 5;
}

static uint32_t bam2DoubleDisplacement16(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1));
	m_bamoffset = (int8_t)OpRead8(m_modadd + 3);

	return 5;
}

static uint32_t am2DoubleDisplacement32(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5);

	return 9;
}

static uint32_t bam2DoubleDisplacement32(void)
{
	m_amflag = 0;
	m_amout = V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1));
	m_bamoffset = OpRead32(m_modadd + 5);

	return 9;
}


static uint32_t am2PCDoubleDisplacement8(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2);

	return 3;
}

static uint32_t bam2PCDoubleDisplacement8(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + (int8_t)OpRead8(m_modadd + 1));
	m_bamoffset = (int8_t)OpRead8(m_modadd + 2);

	return 3;
}

static uint32_t am2PCDoubleDisplacement16(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3);

	return 5;
}

static uint32_t bam2PCDoubleDisplacement16(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + (int16_t)OpRead16(m_modadd + 1));
	m_bamoffset = (int8_t)OpRead8(m_modadd + 3);

	return 5;
}

static uint32_t am2PCDoubleDisplacement32(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5);

	return 9;
}

static uint32_t bam2PCDoubleDisplacement32(void)
{
	m_amflag = 0;
	m_amout = V60_RD(PC + OpRead32(m_modadd + 1));
	m_bamoffset = OpRead32(m_modadd + 5);

	return 9;
}

static uint32_t am2DirectAddress(void)
{
	m_amflag = 0;
	m_amout = OpRead32(m_modadd + 1);

	return 5;
}

static uint32_t bam2DirectAddress(void)
{
	m_amflag = 0;
	m_amout = OpRead32(m_modadd + 1);
	m_bamoffset = 0;

	return 5;
}

static uint32_t am2DirectAddressIndexed(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 6;
}

static uint32_t bam2DirectAddressIndexed(void)
{
	m_amflag = 0;
	m_amout = OpRead32(m_modadd + 2);
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 6;
}

static uint32_t am2DirectAddressDeferred(void)
{
	m_amflag = 0;
	m_amout = V60_RD(OpRead32(m_modadd + 1));

	return 5;
}

static uint32_t bam2DirectAddressDeferred(void)
{
	m_amflag = 0;
	m_amout = V60_RD(OpRead32(m_modadd + 1));
	m_bamoffset = 0;

	return 5;
}

static uint32_t am2DirectAddressDeferredIndexed(void)
{
	m_amflag = 0;

	switch (m_moddim)
	{
	case 0:
		m_amout = V60_RD(OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F];
		break;
	case 1:
		m_amout = V60_RD(OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2;
		break;
	case 2:
		m_amout = V60_RD(OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4;
		break;
	case 3:
		m_amout = V60_RD(OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 8;
		break;
	}

	return 6;
}

static uint32_t bam2DirectAddressDeferredIndexed(void)
{
	m_amflag = 0;
	m_amout = V60_RD(OpRead32(m_modadd + 2));
	m_bamoffset = m_reg[m_modval & 0x1F];

	return 6;
}

static uint32_t am2Immediate(void)
{
	// ignore LDPR
	return am1Immediate();
}

static uint32_t am2ImmediateQuick(void)
{
	// ignore LDPR
	return am1ImmediateQuick();
}


// AM2 Tables (for ReadAMAddress)
// ******************************

static uint32_t am2Error1(void)
{
	// f1lap trips this, why?
	v60_log("CPU - AM2 - 1 (PC=%06x)", PC);
	return 0;
}

static uint32_t am2Error2(void)
{
	return v60_fatal("CPU - AM2 - 2 (PC=%06x)\n", PC);
}

static uint32_t am2Error3(void)
{
	return v60_fatal("CPU - AM2 - 3 (PC=%06x)\n", PC);
}

static uint32_t am2Error4(void)
{
	return v60_fatal("CPU - AM2 - 4 (PC=%06x)\n", PC);
}

static uint32_t am2Error5(void)
{
	return v60_fatal("CPU - AM2 - 5 (PC=%06x)\n", PC);
}

static uint32_t bam2Error1(void)
{
	return v60_fatal("CPU - BAM2 - 1 (PC=%06x)\n", PC);
}

static uint32_t bam2Error2(void)
{
	return v60_fatal("CPU - BAM2 - 2 (PC=%06x)\n", PC);
}

static uint32_t bam2Error3(void)
{
	return v60_fatal("CPU - BAM2 - 3 (PC=%06x)\n", PC);
}

static uint32_t bam2Error4(void)
{
	return v60_fatal("CPU - BAM2 - 4 (PC=%06x)\n", PC);
}

static uint32_t bam2Error5(void)
{
	return v60_fatal("CPU - BAM2 - 5 (PC=%06x)\n", PC);
}

static uint32_t bam2Error6(void)
{
	return v60_fatal("CPU - BAM2 - 6 (PC=%06x)\n", PC);
}


static const am_func s_AMTable2_G7a[16] =
{
	am2PCDisplacementIndexed8,
	am2PCDisplacementIndexed16,
	am2PCDisplacementIndexed32,
	am2DirectAddressIndexed,
	am2Error5,
	am2Error5,
	am2Error5,
	am2Error5,
	am2PCDisplacementIndirectIndexed8,
	am2PCDisplacementIndirectIndexed16,
	am2PCDisplacementIndirectIndexed32,
	am2DirectAddressDeferredIndexed,
	am2Error5,
	am2Error5,
	am2Error5,
	am2Error5
};

static const am_func s_BAMTable2_G7a[16] =
{
	bam2PCDisplacementIndexed8,
	bam2PCDisplacementIndexed16,
	bam2PCDisplacementIndexed32,
	bam2DirectAddressIndexed,
	bam2Error5,
	bam2Error5,
	bam2Error5,
	bam2Error5,
	bam2PCDisplacementIndirectIndexed8,
	bam2PCDisplacementIndirectIndexed16,
	bam2PCDisplacementIndirectIndexed32,
	bam2DirectAddressDeferredIndexed,
	bam2Error5,
	bam2Error5,
	bam2Error5,
	bam2Error5
};

static uint32_t am2Group7a(void)
{
	if (!(m_modval2 & 0x10))
		return am2Error4();

	return (s_AMTable2_G7a[m_modval2 & 0xF])();
}

static uint32_t bam2Group7a(void)
{
	if (!(m_modval2 & 0x10))
		return bam2Error4();

	return (s_BAMTable2_G7a[m_modval2 & 0xF])();
}

static const am_func s_AMTable2_G7[32] =
{
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2ImmediateQuick,
	am2PCDisplacement8,
	am2PCDisplacement16,
	am2PCDisplacement32,
	am2DirectAddress,
	am2Immediate,
	am2Error2,
	am2Error2,
	am2Error2,
	am2PCDisplacementIndirect8,
	am2PCDisplacementIndirect16,
	am2PCDisplacementIndirect32,
	am2DirectAddressDeferred,
	am2PCDoubleDisplacement8,
	am2PCDoubleDisplacement16,
	am2PCDoubleDisplacement32,
	am2Error2
};

static const am_func s_BAMTable2_G7[32] =
{
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2Error6,
	bam2PCDisplacement8,
	bam2PCDisplacement16,
	bam2PCDisplacement32,
	bam2DirectAddress,
	bam2Error6,
	bam2Error2,
	bam2Error2,
	bam2Error2,
	bam2PCDisplacementIndirect8,
	bam2PCDisplacementIndirect16,
	bam2PCDisplacementIndirect32,
	bam2DirectAddressDeferred,
	bam2PCDoubleDisplacement8,
	bam2PCDoubleDisplacement16,
	bam2PCDoubleDisplacement32,
	bam2Error2
};

static const am_func s_AMTable2_G6[8] =
{
	am2DisplacementIndexed8,
	am2DisplacementIndexed16,
	am2DisplacementIndexed32,
	am2RegisterIndirectIndexed,
	am2DisplacementIndirectIndexed8,
	am2DisplacementIndirectIndexed16,
	am2DisplacementIndirectIndexed32,
	am2Group7a
};

static const am_func s_BAMTable2_G6[8] =
{
	bam2DisplacementIndexed8,
	bam2DisplacementIndexed16,
	bam2DisplacementIndexed32,
	bam2RegisterIndirectIndexed,
	bam2DisplacementIndirectIndexed8,
	bam2DisplacementIndirectIndexed16,
	bam2DisplacementIndirectIndexed32,
	bam2Group7a
};




static uint32_t am2Group6(void)
{
	m_modval2 = OpRead8(m_modadd + 1);
	return (s_AMTable2_G6[m_modval2 >> 5])();
}
static uint32_t bam2Group6(void)
{
	m_modval2 = OpRead8(m_modadd + 1);
	return (s_BAMTable2_G6[m_modval2 >> 5])();
}

static uint32_t am2Group7(void)
{
	return (s_AMTable2_G7[m_modval & 0x1F])();
}
static uint32_t bam2Group7(void)
{
	return (s_BAMTable2_G7[m_modval & 0x1F])();
}


static const am_func s_AMTable2[2][8] =
{
	{
		am2Displacement8,
		am2Displacement16,
		am2Displacement32,
		am2RegisterIndirect,
		am2DisplacementIndirect8,
		am2DisplacementIndirect16,
		am2DisplacementIndirect32,
		am2Group7
	},

	{
		am2DoubleDisplacement8,
		am2DoubleDisplacement16,
		am2DoubleDisplacement32,
		am2Register,
		am2Autoincrement,
		am2Autodecrement,
		am2Group6,
		am2Error1
	}
};

static const am_func s_BAMTable2[2][8] =
{
	{
		bam2Displacement8,
		bam2Displacement16,
		bam2Displacement32,
		bam2RegisterIndirect,
		bam2DisplacementIndirect8,
		bam2DisplacementIndirect16,
		bam2DisplacementIndirect32,
		bam2Group7
	},

	{
		bam2DoubleDisplacement8,
		bam2DoubleDisplacement16,
		bam2DoubleDisplacement32,
		bam2Error6,
		bam2Autoincrement,
		bam2Autodecrement,
		bam2Group6,
		bam2Error1
	}
};

// ---- am3.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
// AM3 Functions (for ReadAM)
// **************************

static uint32_t am3Register(void)
{
	switch (m_moddim)
	{
	case 0:
		SETREG8(m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		SETREG16(m_reg[m_modval & 0x1F], m_modwritevalh);
		break;
	case 2:
		m_reg[m_modval & 0x1F] = m_modwritevalw;
		break;
	}

	return 1;
}

static uint32_t am3RegisterIndirect(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(m_reg[m_modval & 0x1F], m_modwritevalh);
		break;
	case 2:
		V60_WD(m_reg[m_modval & 0x1F], m_modwritevalw);
		break;
	}

	return 1;
}

static uint32_t am3RegisterIndirectIndexed(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(m_reg[m_modval2 & 0x1F] + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(m_reg[m_modval2 & 0x1F] + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(m_reg[m_modval2 & 0x1F] + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 2;
}

static uint32_t am3Autoincrement(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(m_reg[m_modval & 0x1F], m_modwritevalb);
		m_reg[m_modval & 0x1F] += 1;
		break;
	case 1:
		V60_WW(m_reg[m_modval & 0x1F], m_modwritevalh);
		m_reg[m_modval & 0x1F] += 2;
		break;
	case 2:
		V60_WD(m_reg[m_modval & 0x1F], m_modwritevalw);
		m_reg[m_modval & 0x1F] += 4;
		break;
	}

	return 1;
}

static uint32_t am3Autodecrement(void)
{
	switch (m_moddim)
	{
	case 0:
		m_reg[m_modval & 0x1F] -= 1;
		V60_WB(m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		m_reg[m_modval & 0x1F] -= 2;
		V60_WW(m_reg[m_modval & 0x1F], m_modwritevalh);
		break;
	case 2:
		m_reg[m_modval & 0x1F] -= 4;
		V60_WD(m_reg[m_modval & 0x1F], m_modwritevalw);
		break;
	}

	return 1;
}

static uint32_t am3Displacement8(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1), m_modwritevalb);
		break;
	case 1:
		V60_WW(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1), m_modwritevalh);
		break;
	case 2:
		V60_WD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1), m_modwritevalw);
		break;
	}

	return 2;
}

static uint32_t am3Displacement16(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1), m_modwritevalb);
		break;
	case 1:
		V60_WW(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1), m_modwritevalh);
		break;
	case 2:
		V60_WD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1), m_modwritevalw);
		break;
	}

	return 3;
}

static uint32_t am3Displacement32(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1), m_modwritevalb);
		break;
	case 1:
		V60_WW(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1), m_modwritevalh);
		break;
	case 2:
		V60_WD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1), m_modwritevalw);
		break;
	}

	return 5;
}


static uint32_t am3DisplacementIndexed8(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 3;
}

static uint32_t am3DisplacementIndexed16(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 4;
}

static uint32_t am3DisplacementIndexed32(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 6;
}


static uint32_t am3PCDisplacement8(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(PC + (int8_t)OpRead8(m_modadd + 1), m_modwritevalb);
		break;
	case 1:
		V60_WW(PC + (int8_t)OpRead8(m_modadd + 1), m_modwritevalh);
		break;
	case 2:
		V60_WD(PC + (int8_t)OpRead8(m_modadd + 1), m_modwritevalw);
		break;
	}

	return 2;
}

static uint32_t am3PCDisplacement16(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(PC + (int16_t)OpRead16(m_modadd + 1), m_modwritevalb);
		break;
	case 1:
		V60_WW(PC + (int16_t)OpRead16(m_modadd + 1), m_modwritevalh);
		break;
	case 2:
		V60_WD(PC + (int16_t)OpRead16(m_modadd + 1), m_modwritevalw);
		break;
	}

	return 3;
}

static uint32_t am3PCDisplacement32(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(PC + OpRead32(m_modadd + 1), m_modwritevalb);
		break;
	case 1:
		V60_WW(PC + OpRead32(m_modadd + 1), m_modwritevalh);
		break;
	case 2:
		V60_WD(PC + OpRead32(m_modadd + 1), m_modwritevalw);
		break;
	}

	return 5;
}

static uint32_t am3PCDisplacementIndexed8(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(PC + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(PC + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(PC + (int8_t)OpRead8(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 3;
}

static uint32_t am3PCDisplacementIndexed16(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(PC + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(PC + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(PC + (int16_t)OpRead16(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 4;
}

static uint32_t am3PCDisplacementIndexed32(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(PC + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(PC + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(PC + OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 6;
}

static uint32_t am3DisplacementIndirect8(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)), m_modwritevalw);
		break;
	}

	return 2;
}

static uint32_t am3DisplacementIndirect16(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)), m_modwritevalw);
		break;
	}

	return 3;
}

static uint32_t am3DisplacementIndirect32(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)), m_modwritevalw);
		break;
	}

	return 5;
}


static uint32_t am3DisplacementIndirectIndexed8(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(m_reg[m_modval2 & 0x1F] + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 3;
}

static uint32_t am3DisplacementIndirectIndexed16(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(m_reg[m_modval2 & 0x1F] + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 4;
}

static uint32_t am3DisplacementIndirectIndexed32(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(m_reg[m_modval2 & 0x1F] + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 6;
}

static uint32_t am3PCDisplacementIndirect8(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)), m_modwritevalw);
		break;
	}

	return 2;
}

static uint32_t am3PCDisplacementIndirect16(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)), m_modwritevalw);
		break;
	}

	return 3;
}

static uint32_t am3PCDisplacementIndirect32(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(PC + OpRead32(m_modadd + 1)), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(PC + OpRead32(m_modadd + 1)), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(PC + OpRead32(m_modadd + 1)), m_modwritevalw);
		break;
	}

	return 5;
}


static uint32_t am3PCDisplacementIndirectIndexed8(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(PC + (int8_t)OpRead8(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 3;
}

static uint32_t am3PCDisplacementIndirectIndexed16(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(PC + (int16_t)OpRead16(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 4;
}

static uint32_t am3PCDisplacementIndirectIndexed32(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(PC + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(PC + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(PC + OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 6;
}


static uint32_t am3DoubleDisplacement8(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(m_reg[m_modval & 0x1F] + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2), m_modwritevalw);
		break;
	}

	return 3;
}

static uint32_t am3DoubleDisplacement16(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(m_reg[m_modval & 0x1F] + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3), m_modwritevalw);
		break;
	}

	return 5;
}

static uint32_t am3DoubleDisplacement32(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(m_reg[m_modval & 0x1F] + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5), m_modwritevalw);
		break;
	}

	return 9;
}


static uint32_t am3PCDoubleDisplacement8(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(PC + (int8_t)OpRead8(m_modadd + 1)) + (int8_t)OpRead8(m_modadd + 2), m_modwritevalw);
		break;
	}

	return 3;
}

static uint32_t am3PCDoubleDisplacement16(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(PC + (int16_t)OpRead16(m_modadd + 1)) + (int16_t)OpRead16(m_modadd + 3), m_modwritevalw);
		break;
	}

	return 5;
}

static uint32_t am3PCDoubleDisplacement32(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(PC + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(PC + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(PC + OpRead32(m_modadd + 1)) + OpRead32(m_modadd + 5), m_modwritevalw);
		break;
	}

	return 9;
}

static uint32_t am3DirectAddress(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(OpRead32(m_modadd + 1), m_modwritevalb);
		break;
	case 1:
		V60_WW(OpRead32(m_modadd + 1), m_modwritevalh);
		break;
	case 2:
		V60_WD(OpRead32(m_modadd + 1), m_modwritevalw);
		break;
	}

	return 5;
}

static uint32_t am3DirectAddressIndexed(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 2, m_modwritevalh);
		break;
	case 2:
		V60_WD(OpRead32(m_modadd + 2) + m_reg[m_modval & 0x1F] * 4, m_modwritevalw);
		break;
	}

	return 6;
}

static uint32_t am3DirectAddressDeferred(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(OpRead32(m_modadd + 1)), m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(OpRead32(m_modadd + 1)), m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(OpRead32(m_modadd + 1)), m_modwritevalw);
		break;
	}

	return 5;
}

static uint32_t am3DirectAddressDeferredIndexed(void)
{
	switch (m_moddim)
	{
	case 0:
		V60_WB(V60_RD(OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F], m_modwritevalb);
		break;
	case 1:
		V60_WW(V60_RD(OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F], m_modwritevalh);
		break;
	case 2:
		V60_WD(V60_RD(OpRead32(m_modadd + 2)) + m_reg[m_modval & 0x1F], m_modwritevalw);
		break;
	}

	return 6;
}

static uint32_t am3Immediate(void)
{
	return v60_fatal("CPU - AM3 - IMM (PC=%06x)\n", PC);
}

static uint32_t am3ImmediateQuick(void)
{
	return v60_fatal("CPU - AM3 - IMMQ (PC=%06x)\n", PC);
}



// AM3 Tables (for ReadAMAddress)
// ******************************

static uint32_t am3Error1(void)
{
	return v60_fatal("CPU - AM3 - 1 (PC=%06x)\n", PC);
}

static uint32_t am3Error2(void)
{
	return v60_fatal("CPU - AM3 - 2 (PC=%06x)\n", PC);
}

static uint32_t am3Error3(void)
{
	return v60_fatal("CPU - AM3 - 3 (PC=%06x)\n", PC);
}

static uint32_t am3Error4(void)
{
	return v60_fatal("CPU - AM3 - 4 (PC=%06x)\n", PC);
}

static uint32_t am3Error5(void)
{
	return v60_fatal("CPU - AM3 - 5 (PC=%06x)\n", PC);
}

static const am_func s_AMTable3_G7a[16] =
{
	am3PCDisplacementIndexed8,
	am3PCDisplacementIndexed16,
	am3PCDisplacementIndexed32,
	am3DirectAddressIndexed,
	am3Error5,
	am3Error5,
	am3Error5,
	am3Error5,
	am3PCDisplacementIndirectIndexed8,
	am3PCDisplacementIndirectIndexed16,
	am3PCDisplacementIndirectIndexed32,
	am3DirectAddressDeferredIndexed,
	am3Error5,
	am3Error5,
	am3Error5,
	am3Error5
};

static uint32_t am3Group7a(void)
{
	if (!(m_modval2 & 0x10))
		return am3Error4();

	return (s_AMTable3_G7a[m_modval2 & 0xF])();
}

static const am_func s_AMTable3_G7[32] =
{
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3ImmediateQuick,
	am3PCDisplacement8,
	am3PCDisplacement16,
	am3PCDisplacement32,
	am3DirectAddress,
	am3Immediate,
	am3Error2,
	am3Error2,
	am3Error2,
	am3PCDisplacementIndirect8,
	am3PCDisplacementIndirect16,
	am3PCDisplacementIndirect32,
	am3DirectAddressDeferred,
	am3PCDoubleDisplacement8,
	am3PCDoubleDisplacement16,
	am3PCDoubleDisplacement32,
	am3Error2
};

static const am_func s_AMTable3_G6[8] =
{
	am3DisplacementIndexed8,
	am3DisplacementIndexed16,
	am3DisplacementIndexed32,
	am3RegisterIndirectIndexed,
	am3DisplacementIndirectIndexed8,
	am3DisplacementIndirectIndexed16,
	am3DisplacementIndirectIndexed32,
	am3Group7a
};




static uint32_t am3Group6(void)
{
	m_modval2 = OpRead8(m_modadd + 1);
	return (s_AMTable3_G6[m_modval2 >> 5])();
}


static uint32_t am3Group7(void)
{
	return (s_AMTable3_G7[m_modval & 0x1F])();
}



static const am_func s_AMTable3[2][8] =
{
	{
		am3Displacement8,
		am3Displacement16,
		am3Displacement32,
		am3RegisterIndirect,
		am3DisplacementIndirect8,
		am3DisplacementIndirect16,
		am3DisplacementIndirect32,
		am3Group7
	},

	{
		am3DoubleDisplacement8,
		am3DoubleDisplacement16,
		am3DoubleDisplacement32,
		am3Register,
		am3Autoincrement,
		am3Autodecrement,
		am3Group6,
		am3Error1
	}
};

// ---- am.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
// NOTE for bit string / field addressing
// ************************************
// m_moddim must be passed as 10 for bit string instructions,
// and as 11 for bit field instructions




// Addressing mode functions and tables

/*
  Input:
  m_modadd
    m_moddim

  Output:
    m_amout
    amLength
*/

static uint32_t ReadAM(void)
{
	m_modm = m_modm?1:0;
	m_modval = OpRead8(m_modadd);
	return (s_AMTable1[m_modm][m_modval >> 5])();
}

static uint32_t BitReadAM(void)
{
	m_modm = m_modm?1:0;
	m_modval = OpRead8(m_modadd);
	return (s_BAMTable1[m_modm][m_modval >> 5])();
}



/*
  Input:
  m_modadd
    m_moddim

  Output:
    m_amout
    m_amflag
    amLength
*/

static uint32_t ReadAMAddress(void)
{
	m_modm = m_modm?1:0;
	m_modval = OpRead8(m_modadd);
	return (s_AMTable2[m_modm][m_modval >> 5])();
}

static uint32_t BitReadAMAddress(void)
{
	m_modm = m_modm?1:0;
	m_modval = OpRead8(m_modadd);
	return (s_BAMTable2[m_modm][m_modval >> 5])();
}

/*
  Input:
  m_modadd
    m_moddim
    m_modwritevalb / H/W

  Output:
    m_amout
    amLength
*/

static uint32_t WriteAM(void)
{
	m_modm = m_modm?1:0;
	m_modval = OpRead8(m_modadd);
	return (s_AMTable3[m_modm][m_modval >> 5])();
}

// ---- op12.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
/*
 * DIVX: the second operand should be treated as dword instead of word
 * GETATE, GETPTE and GETRA should not be used
 * UPDPSW: _CY and _OV must be cleared or unchanged? I suppose
 *   cleared, like TEST being done on the mask operand.
 * MOVT: I cannot understand exactly what happens to the result
 *   when an overflow occurs
 *
 * Unimplemented opcodes:
 * ROTC, UPDATE, UPDPTE
 */


/*
 *  Macro to access data in operands decoded with ReadAMAddress()
 */

#define F12LOADOPBYTE(num)                          \
	if (m_flag##num)                                \
		appb = (uint8_t)m_reg[m_op##num];         \
	else                                                \
		appb = V60_RB(m_op##num);

#define F12LOADOPHALF(num)                          \
	if (m_flag##num)                                \
		apph = (uint16_t)m_reg[m_op##num];        \
	else                                                \
		apph = V60_RW(m_op##num);

#define F12LOADOPWORD(num)                          \
	if (m_flag##num)                                \
		appw = m_reg[m_op##num];                \
	else                                                \
		appw = V60_RD(m_op##num);

#define F12STOREOPBYTE(num)                         \
	if (m_flag##num)                                \
		SETREG8(m_reg[m_op##num], appb);        \
	else                                                \
		V60_WB(m_op##num, appb);

#define F12STOREOPHALF(num)                         \
	if (m_flag##num)                                \
		SETREG16(m_reg[m_op##num], apph);       \
	else                                                \
		V60_WW(m_op##num, apph);

#define F12STOREOPWORD(num)                         \
	if (m_flag##num)                                \
		m_reg[m_op##num] = appw;                \
	else                                                \
		V60_WD(m_op##num, appw);

#define F12LOADOP1BYTE()  F12LOADOPBYTE(1)
#define F12LOADOP1HALF()  F12LOADOPHALF(1)
#define F12LOADOP1WORD()  F12LOADOPWORD(1)

#define F12LOADOP2BYTE()  F12LOADOPBYTE(2)
#define F12LOADOP2HALF()  F12LOADOPHALF(2)
#define F12LOADOP2WORD()  F12LOADOPWORD(2)

#define F12STOREOP1BYTE()  F12STOREOPBYTE(1)
#define F12STOREOP1HALF()  F12STOREOPHALF(1)
#define F12STOREOP1WORD()  F12STOREOPWORD(1)

#define F12STOREOP2BYTE()  F12STOREOPBYTE(2)
#define F12STOREOP2HALF()  F12STOREOPHALF(2)
#define F12STOREOP2WORD()  F12STOREOPWORD(2)

#define F12END()                                  \
	return m_amlength1 + m_amlength2 + 2;


// Decode the first operand of the instruction and prepare
// writing to the second operand.
static void F12DecodeFirstOperand(am_func DecodeOp1, uint8_t dim1)
{
	m_instflags = OpRead8(PC + 1);

	// Check if F1 or F2
	if (m_instflags & 0x80)
	{
		m_moddim = dim1;
		m_modm = m_instflags & 0x40;
		m_modadd = PC + 2;
		m_amlength1 = (DecodeOp1)();
		m_op1 = m_amout;
		m_flag1 = m_amflag;
	}
	else
	{
		// Check D flag
		if (m_instflags & 0x20)
		{
			m_moddim = dim1;
			m_modm = m_instflags & 0x40;
			m_modadd = PC + 2;
			m_amlength1 = (DecodeOp1)();
			m_op1 = m_amout;
			m_flag1 = m_amflag;
		}
		else
		{
			if (DecodeOp1 == ReadAM)
			{
				switch (dim1)
				{
				case 0:
					m_op1 = (uint8_t)m_reg[m_instflags & 0x1F];
					break;
				case 1:
					m_op1 = (uint16_t)m_reg[m_instflags & 0x1F];
					break;
				case 2:
					m_op1 = m_reg[m_instflags & 0x1F];
					break;
				}

				m_flag1 = 0;
			}
			else
			{
				m_flag1 = 1;
				m_op1 = m_instflags & 0x1F;
			}

			m_amlength1 = 0;
		}
	}
}

static void F12WriteSecondOperand(uint8_t dim2)
{
	m_moddim = dim2;

	// Check if F1 or F2
	if (m_instflags & 0x80)
	{
		m_modm = m_instflags & 0x20;
		m_modadd = PC + 2 + m_amlength1;
		m_moddim = dim2;
		m_amlength2 = WriteAM();
	}
	else
	{
		// Check D flag
		if (m_instflags & 0x20)
		{
			switch (dim2)
			{
			case 0:
				SETREG8(m_reg[m_instflags & 0x1F], m_modwritevalb);
				break;
			case 1:
				SETREG16(m_reg[m_instflags & 0x1F], m_modwritevalh);
				break;
			case 2:
				m_reg[m_instflags & 0x1F] = m_modwritevalw;
				break;
			}

			m_amlength2 = 0;
		}
		else
		{
			m_modm = m_instflags & 0x40;
			m_modadd = PC + 2;
			m_moddim = dim2;
			m_amlength2 = WriteAM();
		}
	}
}



// Decode both format 1 / 2 operands
static void F12DecodeOperands(am_func DecodeOp1, uint8_t dim1, am_func DecodeOp2, uint8_t dim2)
{
	uint8_t _if12 = OpRead8(PC + 1);

	// Check if F1 or F2
	if (_if12 & 0x80)
	{
		m_moddim = dim1;
		m_modm = _if12 & 0x40;
		m_modadd = PC + 2;
		m_amlength1 = (DecodeOp1)();
		m_op1 = m_amout;
		m_flag1 = m_amflag;

		m_moddim = dim2;
		m_modm = _if12 & 0x20;
		m_modadd = PC + 2 + m_amlength1;
		m_amlength2 = (DecodeOp2)();
		m_op2 = m_amout;
		m_flag2 = m_amflag;
	}
	else
	{
		// Check D flag
		if (_if12 & 0x20)
		{
			if (DecodeOp2 == ReadAMAddress)
			{
				m_op2 = _if12 & 0x1F;
				m_flag2 = 1;
			}
			else
			{
				switch (dim2)
				{
				case 0:
					m_op2 = (uint8_t)m_reg[_if12 & 0x1F];
					break;
				case 1:
					m_op2 = (uint16_t)m_reg[_if12 & 0x1F];
					break;
				case 2:
					m_op2 = m_reg[_if12 & 0x1F];
					break;
				}
			}

			m_amlength2 = 0;

			m_moddim = dim1;
			m_modm = _if12 & 0x40;
			m_modadd = PC + 2;
			m_amlength1 = (DecodeOp1)();
			m_op1 = m_amout;
			m_flag1 = m_amflag;
		}
		else
		{
			if (DecodeOp1 == ReadAMAddress)
			{
				m_op1 = _if12 & 0x1F;
				m_flag1 = 1;
			}
			else
			{
				switch (dim1)
				{
				case 0:
					m_op1 = (uint8_t)m_reg[_if12 & 0x1F];
					break;
				case 1:
					m_op1 = (uint16_t)m_reg[_if12 & 0x1F];
					break;
				case 2:
					m_op1 = m_reg[_if12 & 0x1F];
					break;
				}
			}
			m_amlength1 = 0;

			m_moddim = dim2;
			m_modm = _if12 & 0x40;
			m_modadd = PC + 2 + m_amlength1;
			m_amlength2 = (DecodeOp2)();
			m_op2 = m_amout;
			m_flag2 = m_amflag;
		}
	}
}

static uint32_t opADDB(void) /* TRUSTED (C too!)*/
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	ADDB(appb, (uint8_t)m_op1, 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opADDH(void) /* TRUSTED (C too!)*/
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	ADDW(apph, (uint16_t)m_op1, 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opADDW(void) /* TRUSTED (C too!) */
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	ADDL(appw, (uint32_t)m_op1, 0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opADDCB(void)
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	ADDB(appb, (uint8_t)m_op1, _CY?1:0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opADDCH(void)
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	ADDW(apph, (uint16_t)m_op1, _CY?1:0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opADDCW(void)
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	ADDL(appw, (uint32_t)m_op1, _CY?1:0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opANDB(void) /* TRUSTED */
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	appb &= m_op1;
	_OV = 0;
	_S = ((appb & 0x80) != 0);
	_Z = (appb == 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opANDH(void) /* TRUSTED */
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	apph &= m_op1;
	_OV = 0;
	_S = ((apph & 0x8000) != 0);
	_Z = (apph == 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opANDW(void) /* TRUSTED */
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	appw &= m_op1;
	_OV = 0;
	_S = ((appw & 0x80000000) != 0);
	_Z = (appw == 0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opCALL(void) /* TRUSTED */
{
	F12DecodeOperands(ReadAMAddress, 0,ReadAMAddress, 2);

	SP -= 4;
	V60_WD(SP, AP);
	AP = m_op2;

	SP -= 4;
	V60_WD(SP, PC + m_amlength1 + m_amlength2 + 2);
	PC = m_op1;

	return 0;
}

static uint32_t opCHKAR(void)
{
	F12DecodeOperands(ReadAM, 0,ReadAM, 0);

	// No MMU and memory permissions yet @@@
	_Z = 1;
	_CY = 0;
	_S = 0;

	F12END();
}

static uint32_t opCHKAW(void)
{
	F12DecodeOperands(ReadAM, 0,ReadAM, 0);

	// No MMU and memory permissions yet @@@
	_Z = 1;
	_CY = 0;
	_S = 0;

	F12END();
}

static uint32_t opCHKAE(void)
{
	F12DecodeOperands(ReadAM, 0,ReadAM, 0);

	// No MMU and memory permissions yet @@@
	_Z = 1;
	_CY = 0;
	_S = 0;

	F12END();
}

static uint32_t opCHLVL(void)
{
	uint32_t oldPSW;

	F12DecodeOperands(ReadAM, 0,ReadAM, 0);

	if (m_op1 > 3)
	{
		return v60_fatal("Illegal data field on opCHLVL, PC=%x\n", PC);
	}

	oldPSW = v60_update_psw_for_exception(0, m_op1);

	SP -= 4;
	V60_WD(SP, m_op2);

	SP -= 4;
	V60_WD(SP, EXCEPTION_CODE_AND_SIZE(0x1800 + m_op1 * 0x100, 8));

	SP -= 4;
	V60_WD(SP, oldPSW);

	SP -= 4;
	V60_WD(SP, PC + m_amlength1 + m_amlength2 + 2);

	PC = GETINTVECT(24 + m_op1);

	return 0;
}

static uint32_t opCLR1(void) /* TRUSTED */
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	_CY = ((appw & (1 << m_op1)) != 0);
	_Z = !(_CY);

	appw &= ~(1 << m_op1);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opCMPB(void) /* TRUSTED (C too!) */
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAM, 0);

	appb = (uint8_t)m_op2;
	SUBB(appb, (uint8_t)m_op1, 0);

	F12END();
}

static uint32_t opCMPH(void) /* TRUSTED (C too!) */
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAM, 1);

	apph = (uint16_t)m_op2;
	SUBW(apph, (uint16_t)m_op1, 0);

	F12END();
}


static uint32_t opCMPW(void) /* TRUSTED (C too!)*/
{
	F12DecodeOperands(ReadAM, 2,ReadAM, 2);

	SUBL(m_op2, (uint32_t)m_op1, 0);

	F12END();
}

static uint32_t opDIVB(void) /* TRUSTED */
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	_OV = ((appb == 0x80) && (m_op1 == 0xFF));
	if (m_op1 && !_OV)
		appb= (int8_t)appb / (int8_t)m_op1;
	_Z = (appb == 0);
	_S = ((appb & 0x80) != 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opDIVH(void) /* TRUSTED */
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	_OV = ((apph == 0x8000) && (m_op1 == 0xFFFF));
	if (m_op1 && !_OV)
		apph = (int16_t)apph / (int16_t)m_op1;
	_Z = (apph == 0);
	_S = ((apph & 0x8000) != 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opDIVW(void) /* TRUSTED */
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	_OV = ((appw == 0x80000000) && (m_op1 == 0xFFFFFFFF));
	if (m_op1 && !_OV)
		appw = (int32_t)appw / (int32_t)m_op1;
	_Z = (appw == 0);
	_S = ((appw & 0x80000000) != 0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opDIVX(void)
{
	int32_t a, b;
	int64_t dv;

	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 3);

	if (m_flag2)
		dv = ((uint64_t)m_reg[(m_op2 & 0x1F) + 1] << 32) | m_reg[m_op2 & 0x1F];
	else
		dv = V60_RQ(m_op2);

	a = div_64x32_rem(dv, m_op1, &b);

	_S = ((a & 0x80000000) != 0);
	_Z = (a == 0);

	if (m_flag2)
	{
		m_reg[m_op2 & 0x1F] = a;
		m_reg[(m_op2 & 0x1F) + 1] = b;
	}
	else
	{
		V60_WD(m_op2, a);
		V60_WD(m_op2 + 4, b);
	}

	F12END();
}

static uint32_t opDIVUX(void)
{
	uint32_t a, b;
	uint64_t dv;

	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 3);

	if (m_flag2)
		dv = ((uint64_t)m_reg[(m_op2 & 0x1F) + 1] << 32) | m_reg[m_op2 & 0x1F];
	else
		dv = V60_RQ(m_op2);

	a = divu_64x32_rem(dv, m_op1, &b);

	_S = ((a & 0x80000000) != 0);
	_Z = (a == 0);

	if (m_flag2)
	{
		m_reg[m_op2 & 0x1F] = a;
		m_reg[(m_op2 & 0x1F) + 1] = b;
	}
	else
	{
		V60_WD(m_op2, a);
		V60_WD(m_op2 + 4, b);
	}

	F12END();
}


static uint32_t opDIVUB(void) /* TRUSTED */
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	_OV = 0;
	if (m_op1)  appb /= (uint8_t)m_op1;
	_Z = (appb == 0);
	_S = ((appb & 0x80) != 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opDIVUH(void) /* TRUSTED */
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	_OV = 0;
	if (m_op1)  apph /= (uint16_t)m_op1;
	_Z = (apph == 0);
	_S = ((apph & 0x8000) != 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opDIVUW(void) /* TRUSTED */
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	_OV = 0;
	if (m_op1)  appw /= m_op1;
	_Z = (appw == 0);
	_S = ((appw & 0x80000000) != 0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opINB(void)
{
	F12DecodeFirstOperand(ReadAMAddress, 0);
	m_modwritevalb = V60_IO_RB(m_op1);

	if ( m_stall_io )
	{
		m_stall_io = 0;
		return 0;
	}

	F12WriteSecondOperand(0);
	F12END();
}

static uint32_t opINH(void)
{
	F12DecodeFirstOperand(ReadAMAddress, 1);
	m_modwritevalh = V60_IO_RW(m_op1);

	if ( m_stall_io )
	{
		m_stall_io = 0;
		return 0;
	}

	F12WriteSecondOperand(1);
	F12END();
}

static uint32_t opINW(void)
{
	F12DecodeFirstOperand(ReadAMAddress, 2);
	m_modwritevalw = V60_IO_RD(m_op1);

	if ( m_stall_io )
	{
		m_stall_io = 0;
		return 0;
	}

	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opLDPR(void)
{
	F12DecodeOperands(ReadAMAddress, 2,ReadAM, 2);
	if (m_op2 <= 28)
	{
		if (m_flag1 &&(!(OpRead8(PC + 1)&0x80 && OpRead8(PC + 2) == 0xf4 ) ))
			m_reg[m_op2 + 36] = m_reg[m_op1];
		else
			m_reg[m_op2 + 36] = m_op1;
	}
	else
	{
		return v60_fatal("Invalid operand on LDPR PC=%x\n", PC);
	}
	F12END();
}

static uint32_t opLDTASK(void)
{
	int i;
	F12DecodeOperands(ReadAMAddress, 2,ReadAM, 2);

	v60WritePSW(v60ReadPSW() & 0xefffffff);

	TR = m_op2;

	TKCW = V60_RD(m_op2);
	m_op2 += 4;
	if(SYCW & 0x100) {
		L0SP = V60_RD(m_op2);
		m_op2 += 4;
	}
	if(SYCW & 0x200) {
		L1SP = V60_RD(m_op2);
		m_op2 += 4;
	}
	if(SYCW & 0x400) {
		L2SP = V60_RD(m_op2);
		m_op2 += 4;
	}
	if(SYCW & 0x800) {
		L3SP = V60_RD(m_op2);
		m_op2 += 4;
	}

	v60ReloadStack();

	// 31 registers supported, _not_ 32
	for(i = 0; i < 31; i++)
		if(m_op1 & (1 << i)) {
			m_reg[i] = V60_RD(m_op2);
			m_op2 += 4;
		}

	// #### Ignore the virtual addressing crap.

	F12END();
}

static uint32_t opMOVD(void) /* TRUSTED */
{
	uint64_t d;

	F12DecodeOperands(ReadAMAddress, 3,ReadAMAddress, 3);

	if (m_flag1)
		d = ((uint64_t)m_reg[(m_op1 & 0x1F) + 1] << 32) | m_reg[m_op1 & 0x1F];
	else
		d = V60_RQ(m_op1);

	if (m_flag2)
	{
		m_reg[m_op2 & 0x1F] = (uint32_t)d;
		m_reg[(m_op2 & 0x1F) + 1] = (uint32_t)(d >> 32);
	}
	else
		V60_WQ(m_op2, d);

	F12END();
}

static uint32_t opMOVB(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 0);
	m_modwritevalb = (uint8_t)m_op1;
	F12WriteSecondOperand(0);
	F12END();
}

static uint32_t opMOVH(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 1);
	m_modwritevalh = (uint16_t)m_op1;
	F12WriteSecondOperand(1);
	F12END();
}

static uint32_t opMOVW(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 2);
	m_modwritevalw = m_op1;
	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opMOVEAB(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAMAddress, 0);
	m_modwritevalw = m_op1;
	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opMOVEAH(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAMAddress, 1);
	m_modwritevalw = m_op1;
	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opMOVEAW(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAMAddress, 2);
	m_modwritevalw = m_op1;
	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opMOVSBH(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 0);
	m_modwritevalh = (int8_t)(m_op1 & 0xFF);
	F12WriteSecondOperand(1);
	F12END();
}

static uint32_t opMOVSBW(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 0);
	m_modwritevalw = (int8_t)(m_op1 & 0xFF);
	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opMOVSHW(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 1);
	m_modwritevalw = (int16_t)(m_op1 & 0xFFFF);
	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opMOVTHB(void)
{
	F12DecodeFirstOperand(ReadAM, 1);
	m_modwritevalb = (uint8_t)(m_op1 & 0xFF);

	// Check for overflow: the truncated bits must match the sign
	//  of the result, otherwise overflow
	if (((m_modwritevalb & 0x80) == 0x80 && ((m_op1 & 0xFF00) == 0xFF00)) ||
			((m_modwritevalb & 0x80) == 0 && ((m_op1 & 0xFF00) == 0x0000)))
		_OV = 0;
	else
		_OV = 1;

	F12WriteSecondOperand(0);
	F12END();
}

static uint32_t opMOVTWB(void)
{
	F12DecodeFirstOperand(ReadAM, 2);
	m_modwritevalb = (uint8_t)(m_op1 & 0xFF);

	// Check for overflow: the truncated bits must match the sign
	//  of the result, otherwise overflow
	if (((m_modwritevalb & 0x80) == 0x80 && ((m_op1 & 0xFFFFFF00) == 0xFFFFFF00)) ||
			((m_modwritevalb & 0x80) == 0 && ((m_op1 & 0xFFFFFF00) == 0x00000000)))
		_OV = 0;
	else
		_OV = 1;

	F12WriteSecondOperand(0);
	F12END();
}

static uint32_t opMOVTWH(void)
{
	F12DecodeFirstOperand(ReadAM, 2);
	m_modwritevalh = (uint16_t)(m_op1 & 0xFFFF);

	// Check for overflow: the truncated bits must match the sign
	//  of the result, otherwise overflow
	if (((m_modwritevalh & 0x8000) == 0x8000 && ((m_op1 & 0xFFFF0000) == 0xFFFF0000)) ||
			((m_modwritevalh & 0x8000) == 0 && ((m_op1 & 0xFFFF0000) == 0x00000000)))
		_OV = 0;
	else
		_OV = 1;

	F12WriteSecondOperand(1);
	F12END();
}


static uint32_t opMOVZBH(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 0);
	m_modwritevalh = (uint16_t)m_op1;
	F12WriteSecondOperand(1);
	F12END();
}

static uint32_t opMOVZBW(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 0);
	m_modwritevalw = m_op1;
	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opMOVZHW(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 1);
	m_modwritevalw = m_op1;
	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opMULB(void)
{
	uint8_t appb;
	uint32_t tmp;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	tmp = (int8_t)appb * (int32_t)(int8_t)m_op1;
	appb = tmp;
	_Z = (appb == 0);
	_S = ((appb & 0x80) != 0);
	_OV = ((tmp >> 8) != 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opMULH(void)
{
	uint16_t apph;
	uint32_t tmp;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	tmp = (int16_t)apph * (int32_t)(int16_t)m_op1;
	apph = tmp;
	_Z = (apph == 0);
	_S = ((apph & 0x8000) != 0);
	_OV = ((tmp >> 16) != 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opMULW(void)
{
	uint32_t appw;
	uint64_t tmp;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	tmp = mul_32x32(appw, m_op1);
	appw = tmp;
	_Z = (appw == 0);
	_S = ((appw & 0x80000000) != 0);
	_OV = ((tmp >> 32) != 0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opMULUB(void)
{
	uint8_t appb;
	uint32_t tmp;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	tmp = appb * (uint8_t)m_op1;
	appb = tmp;
	_Z = (appb == 0);
	_S = ((appb & 0x80) != 0);
	_OV = ((tmp >> 8) != 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opMULUH(void)
{
	uint16_t apph;
	uint32_t tmp;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	tmp = apph * (uint16_t)m_op1;
	apph = tmp;
	_Z = (apph == 0);
	_S = ((apph & 0x8000) != 0);
	_OV = ((tmp >> 16) != 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opMULUW(void)
{
	uint32_t appw;
	uint64_t tmp;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	tmp = mulu_32x32(appw, m_op1);
	appw = tmp;
	_Z = (appw == 0);
	_S = ((appw & 0x80000000) != 0);
	_OV = ((tmp >> 32) != 0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opNEGB(void) /* TRUSTED  (C too!)*/
{
	F12DecodeFirstOperand(ReadAM, 0);

	m_modwritevalb = 0;
	SUBB(m_modwritevalb, (int8_t)m_op1, 0);
	_CY = m_modwritevalb ? 1 : 0;

	F12WriteSecondOperand(0);
	F12END();
}

static uint32_t opNEGH(void) /* TRUSTED  (C too!)*/
{
	F12DecodeFirstOperand(ReadAM, 1);

	m_modwritevalh = 0;
	SUBW(m_modwritevalh, (int16_t)m_op1, 0);
	_CY = m_modwritevalh ? 1 : 0;

	F12WriteSecondOperand(1);
	F12END();
}

static uint32_t opNEGW(void) /* TRUSTED  (C too!)*/
{
	F12DecodeFirstOperand(ReadAM, 2);

	m_modwritevalw = 0;
	SUBL(m_modwritevalw, (int32_t)m_op1, 0);
	_CY = m_modwritevalw ? 1 : 0;

	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opNOTB(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 0);
	m_modwritevalb=~m_op1;

	_OV = 0;
	_S = ((m_modwritevalb & 0x80) != 0);
	_Z = (m_modwritevalb == 0);

	F12WriteSecondOperand(0);
	F12END();
}

static uint32_t opNOTH(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 1);
	m_modwritevalh=~m_op1;

	_OV = 0;
	_S = ((m_modwritevalh & 0x8000) != 0);
	_Z = (m_modwritevalh == 0);

	F12WriteSecondOperand(1);
	F12END();
}

static uint32_t opNOTW(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 2);
	m_modwritevalw=~m_op1;

	_OV = 0;
	_S = ((m_modwritevalw & 0x80000000) != 0);
	_Z = (m_modwritevalw == 0);

	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opNOT1(void) /* TRUSTED */
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	_CY = ((appw & (1 << m_op1)) != 0);
	_Z = !(_CY);

	if (_CY)
		appw &= ~(1 << m_op1);
	else
		appw |= (1 << m_op1);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opORB(void) /* TRUSTED  (C too!)*/
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	ORB(appb, (uint8_t)m_op1);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opORH(void) /* TRUSTED (C too!)*/
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	ORW(apph, (uint16_t)m_op1);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opORW(void) /* TRUSTED (C too!) */
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	ORL(appw, (uint32_t)m_op1);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opOUTB(void)
{
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 2);
	V60_IO_WB(m_op2,(uint8_t)m_op1);
	F12END();
}

static uint32_t opOUTH(void)
{
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 2);
	V60_IO_WW(m_op2,(uint16_t)m_op1);
	F12END();
}

static uint32_t opOUTW(void)
{
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);
	V60_IO_WD(m_op2, m_op1);
	F12END();
}

static uint32_t opREMB(void)
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	_OV = 0;
	if (m_op1)
		appb= (int8_t)appb % (int8_t)m_op1;
	_Z = (appb == 0);
	_S = ((appb & 0x80) != 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opREMH(void)
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	_OV = 0;
	if (m_op1)
		apph = (int16_t)apph % (int16_t)m_op1;
	_Z = (apph == 0);
	_S = ((apph & 0x8000) != 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opREMW(void)
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	_OV = 0;
	if (m_op1)
		appw = (int32_t)appw % (int32_t)m_op1;
	_Z = (appw == 0);
	_S = ((appw & 0x80000000) != 0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opREMUB(void)
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	_OV = 0;
	if (m_op1)
		appb %= (uint8_t)m_op1;
	_Z = (appb == 0);
	_S = ((appb & 0x80) != 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opREMUH(void)
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	_OV = 0;
	if (m_op1)
		apph %= (uint16_t)m_op1;
	_Z = (apph == 0);
	_S = ((apph & 0x8000) != 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opREMUW(void)
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	_OV = 0;
	if (m_op1)
		appw %= m_op1;
	_Z = (appw == 0);
	_S = ((appw & 0x80000000) != 0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opROTB(void) /* TRUSTED */
{
	uint8_t appb;
	int8_t i, count;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	count = (int8_t)(m_op1 & 0xFF);
	if (count > 0)
	{
		for (i = 0;i < count;i++)
			appb = (appb << 1) | ((appb & 0x80) >> 7);

		_CY = (appb & 0x1) != 0;
	}
	else if (count < 0)
	{
		count=-count;
		for (i = 0;i < count;i++)
			appb = (appb >> 1) | ((appb & 0x1) << 7);

		_CY = (appb & 0x80) != 0;
	}
	else
		_CY = 0;

	_OV = 0;
	_S = (appb & 0x80) != 0;
	_Z = (appb == 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opROTH(void) /* TRUSTED */
{
	uint16_t apph;
	int8_t i, count;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 1);

	F12LOADOP2HALF();

	count = (int8_t)(m_op1 & 0xFF);
	if (count > 0)
	{
		for (i = 0;i < count;i++)
			apph = (apph << 1) | ((apph & 0x8000) >> 15);

		_CY = (apph & 0x1) != 0;
	}
	else if (count < 0)
	{
		count=-count;
		for (i = 0;i < count;i++)
			apph = (apph >> 1) | ((apph & 0x1) << 15);

		_CY = (apph & 0x8000) != 0;
	}
	else
		_CY = 0;

	_OV = 0;
	_S = (apph & 0x8000) != 0;
	_Z = (apph == 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opROTW(void) /* TRUSTED */
{
	uint32_t appw;
	int8_t i, count;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 2);

	F12LOADOP2WORD();

	count = (int8_t)(m_op1 & 0xFF);
	if (count > 0)
	{
		for (i = 0;i < count;i++)
			appw = (appw << 1) | ((appw & 0x80000000) >> 31);

		_CY = (appw & 0x1) != 0;
	}
	else if (count < 0)
	{
		count=-count;
		for (i = 0;i < count;i++)
			appw = (appw >> 1) | ((appw & 0x1) << 31);

		_CY = (appw & 0x80000000) != 0;
	}
	else
		_CY = 0;

	_OV = 0;
	_S = (appw & 0x80000000) != 0;
	_Z = (appw == 0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opROTCB(void) /* TRUSTED */
{
	uint8_t appb;
	int8_t i, cy, count;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();
	NORMALIZEFLAGS();

	count = (int8_t)(m_op1 & 0xFF);
	if (count > 0)
	{
		for (i = 0;i < count;i++)
		{
			cy = _CY;
			_CY = (uint8_t)((appb & 0x80) >> 7);
			appb = (appb << 1) | cy;
		}
	}
	else if (count < 0)
	{
		count=-count;
		for (i = 0;i < count;i++)
		{
			cy = _CY;
			_CY = (appb & 1);
			appb = (appb >> 1) | (cy << 7);
		}
	}
	else
		_CY = 0;

	_OV = 0;
	_S = (appb & 0x80) != 0;
	_Z = (appb == 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opROTCH(void) /* TRUSTED */
{
	uint16_t apph;
	int8_t i, cy, count;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 1);

	F12LOADOP2HALF();
	NORMALIZEFLAGS();

	count = (int8_t)(m_op1 & 0xFF);
	if (count > 0)
	{
		for (i = 0;i < count;i++)
		{
			cy = _CY;
			_CY = (uint8_t)((apph & 0x8000) >> 15);
			apph = (apph << 1) | cy;
		}
	}
	else if (count < 0)
	{
		count=-count;
		for (i = 0;i < count;i++)
		{
			cy = _CY;
			_CY = (uint8_t)(apph & 1);
			apph = (apph >> 1) | ((uint16_t)cy << 15);
		}
	}
	else
		_CY = 0;

	_OV = 0;
	_S = (apph & 0x8000) != 0;
	_Z = (apph == 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opROTCW(void) /* TRUSTED */
{
	uint32_t appw;
	int8_t i, cy, count;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 2);

	F12LOADOP2WORD();
	NORMALIZEFLAGS();

	count = (int8_t)(m_op1 & 0xFF);
	if (count > 0)
	{
		for (i = 0;i < count;i++)
		{
			cy = _CY;
			_CY = (uint8_t)((appw & 0x80000000) >> 31);
			appw = (appw << 1) | cy;
		}
	}
	else if (count < 0)
	{
		count=-count;
		for (i = 0;i < count;i++)
		{
			cy = _CY;
			_CY = (uint8_t)(appw & 1);
			appw = (appw >> 1) | ((uint32_t)cy << 31);
		}
	}
	else
		_CY = 0;

	_OV = 0;
	_S = (appw & 0x80000000) != 0;
	_Z = (appw == 0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opRVBIT(void)
{
	F12DecodeFirstOperand(ReadAM, 0);

	m_modwritevalb = v60_bitswap8(m_op1, 0, 1, 2, 3, 4, 5, 6, 7);

	F12WriteSecondOperand(0);
	F12END();
}

static uint32_t opRVBYT(void) /* TRUSTED */
{
	F12DecodeFirstOperand(ReadAM, 2);

	m_modwritevalw = swapendian_int32(m_op1);

	F12WriteSecondOperand(2);
	F12END();
}

static uint32_t opSET1(void) /* TRUSTED */
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	_CY = ((appw & (1 << m_op1)) != 0);
	_Z = !(_CY);

	appw |= (1 << m_op1);

	F12STOREOP2WORD();
	F12END();
}


static uint32_t opSETF(void)
{
	F12DecodeFirstOperand(ReadAM, 0);

	// Normalize the flags
	NORMALIZEFLAGS();

	switch (m_op1 & 0xF)
	{
	case 0:
		if (!_OV) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 1:
		if (_OV) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 2:
		if (!_CY) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 3:
		if (_CY) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 4:
		if (!_Z) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 5:
		if (_Z) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 6:
		if (!(_CY | _Z)) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 7:
		if ((_CY | _Z)) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 8:
		if (!_S) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 9:
		if (_S) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 10:
		m_modwritevalb = 1;
		break;
	case 11:
		m_modwritevalb = 0;
		break;
	case 12:
		if (!(_S^_OV)) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 13:
		if ((_S^_OV)) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 14:
		if (!((_S^_OV)|_Z)) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	case 15:
		if (((_S^_OV)|_Z)) m_modwritevalb = 0;
		else m_modwritevalb = 1;
		break;
	}

	F12WriteSecondOperand(0);

	F12END();
}

/*
#define SHIFTLEFT_OY(val, count, bitsize) \
{\
    uint32_t tmp = ((val) >> (bitsize - 1)) & 1; \
    tmp <<= count; \
    tmp -= 1; \
    tmp <<= (bitsize - (count)); \
    _OV = (((val) & tmp) != tmp); \
    _CY = (((val) & (1 << (count - 1))) != 0); \
}
*/

// During the shift, the overflow is set if the sign bit changes at any point during the shift
#define SHIFTLEFT_OV(val, count, bitsize) \
{\
	uint32_t tmp; \
	if (count == 32) \
		tmp = 0xFFFFFFFF; \
	else \
		tmp = ((1 << (count)) - 1); \
	tmp <<= (bitsize - (count)); \
	if (((val) >> (bitsize - 1)) & 1) \
		_OV = (((val) & tmp) != tmp); \
	else \
		_OV = (((val) & tmp) != 0); \
}

#define SHIFTLEFT_CY(val, count, bitsize) \
	_CY = (uint8_t)(((val) >> (bitsize - count)) & 1);



#define SHIFTARITHMETICRIGHT_OV(val, count, bitsize) \
	_OV = 0;

#define SHIFTARITHMETICRIGHT_CY(val, count, bitsize) \
	_CY = (uint8_t)(((val) >> (count - 1)) & 1);



static uint32_t opSHAB(void)
{
	uint8_t appb;
	int8_t count;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	count = (int8_t)(m_op1 & 0xFF);

	// Special case: destination unchanged, flags set
	if (count == 0)
	{
		_CY = _OV = 0;
		SetSZPF_Byte(appb);
	}
	else if (count > 0)
	{
		SHIFTLEFT_OV(appb, count, 8);

		// @@@ Undefined what happens to CY when count >= bitsize
		SHIFTLEFT_CY(appb, count, 8);

		// do the actual shift...
		if (count >= 8)
			appb = 0;
		else
			appb <<= count;

		// and set zero and sign
		SetSZPF_Byte(appb);
	}
	else
	{
		count = -count;

		SHIFTARITHMETICRIGHT_OV(appb, count, 8);
		SHIFTARITHMETICRIGHT_CY(appb, count, 8);

		if (count >= 8)
			appb = (appb & 0x80) ? 0xFF : 0;
		else
			appb = ((int8_t)appb) >> count;

		SetSZPF_Byte(appb);
	}

//  osd_printf_debug("SHAB: %x _CY: %d _Z: %d _OV: %d _S: %d\n", appb, _CY, _Z, _OV, _S);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opSHAH(void)
{
	uint16_t apph;
	int8_t count;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 1);

	F12LOADOP2HALF();

	count = (int8_t)(m_op1 & 0xFF);

	// Special case: destination unchanged, flags set
	if (count == 0)
	{
		_CY = _OV = 0;
		SetSZPF_Word(apph);
	}
	else if (count > 0)
	{
		SHIFTLEFT_OV(apph, count, 16);

		// @@@ Undefined what happens to CY when count >= bitsize
		SHIFTLEFT_CY(apph, count, 16);

		// do the actual shift...
		if (count >= 16)
			apph = 0;
		else
			apph <<= count;

		// and set zero and sign
		SetSZPF_Word(apph);
	}
	else
	{
		count = -count;

		SHIFTARITHMETICRIGHT_OV(apph, count, 16);
		SHIFTARITHMETICRIGHT_CY(apph, count, 16);

		if (count >= 16)
			apph = (apph & 0x8000) ? 0xFFFF : 0;
		else
			apph = ((int16_t)apph) >> count;

		SetSZPF_Word(apph);
	}

//  osd_printf_debug("SHAH: %x >> %d = %x _CY: %d _Z: %d _OV: %d _S: %d\n", oldval, count, apph, _CY, _Z, _OV, _S);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opSHAW(void)
{
	uint32_t appw;
	int8_t count;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 2);

	F12LOADOP2WORD();

	count = (int8_t)(m_op1 & 0xFF);

	// Special case: destination unchanged, flags set
	if (count == 0)
	{
		_CY = _OV = 0;
		SetSZPF_Long(appw);
	}
	else if (count > 0)
	{
		SHIFTLEFT_OV(appw, count, 32);

		// @@@ Undefined what happens to CY when count >= bitsize
		SHIFTLEFT_CY(appw, count, 32);

		// do the actual shift...
		if (count >= 32)
			appw = 0;
		else
			appw <<= count;

		// and set zero and sign
		SetSZPF_Long(appw);
	}
	else
	{
		count = -count;

		SHIFTARITHMETICRIGHT_OV(appw, count, 32);
		SHIFTARITHMETICRIGHT_CY(appw, count, 32);

		if (count >= 32)
			appw = (appw & 0x80000000) ? 0xFFFFFFFF : 0;
		else
			appw = ((int32_t)appw) >> count;

		SetSZPF_Long(appw);
	}

//  osd_printf_debug("SHAW: %x >> %d = %x _CY: %d _Z: %d _OV: %d _S: %d\n", oldval, count, appw, _CY, _Z, _OV, _S);

	F12STOREOP2WORD();
	F12END();
}


static uint32_t opSHLB(void) /* TRUSTED */
{
	uint8_t appb;
	int8_t count;
	uint32_t tmp;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	count = (int8_t)(m_op1 & 0xFF);
	if (count > 0)
	{
		// left shift flags:
		// carry gets the last bit shifted out,
		// overflow is always CLEARed

		_OV = 0;  // default to no overflow

		// now handle carry
		tmp = appb & 0xff;
		tmp <<= count;
		SetCFB(tmp);    // set carry properly

		// do the actual shift...
		appb <<= count;

		// and set zero and sign
		SetSZPF_Byte(appb);
	}
	else
	{
		if (count == 0)
		{
			// special case: clear carry and overflow, do nothing else
			_CY = _OV = 0;
			SetSZPF_Byte(appb); // doc. is unclear if this is true...
		}
		else
		{
			// right shift flags:
			// carry = last bit shifted out
			// overflow always cleared
			tmp = appb & 0xff;
			tmp >>= ((-count) - 1);
			_CY = (uint8_t)(tmp & 0x1);
			_OV = 0;

			appb >>= -count;
			SetSZPF_Byte(appb);
		}
	}

//  osd_printf_debug("SHLB: %x _CY: %d _Z: %d _OV: %d _S: %d\n", appb, _CY, _Z, _OV, _S);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opSHLH(void) /* TRUSTED */
{
	uint16_t apph;
	int8_t count;
	uint32_t tmp;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 1);

	F12LOADOP2HALF();

	count = (int8_t)(m_op1 & 0xFF);
//  osd_printf_debug("apph: %x count: %d  ", apph, count);
	if (count > 0)
	{
		// left shift flags:
		// carry gets the last bit shifted out,
		// overflow is always CLEARed

		_OV = 0;

		// now handle carry
		tmp = apph & 0xffff;
		tmp <<= count;
		SetCFW(tmp);    // set carry properly

		// do the actual shift...
		apph <<= count;

		// and set zero and sign
		SetSZPF_Word(apph);
	}
	else
	{
		if (count == 0)
		{
			// special case: clear carry and overflow, do nothing else
			_CY = _OV = 0;
			SetSZPF_Word(apph); // doc. is unclear if this is true...
		}
		else
		{
			// right shift flags:
			// carry = last bit shifted out
			// overflow always cleared
			tmp = apph & 0xffff;
			tmp >>= ((-count) - 1);
			_CY = (uint8_t)(tmp & 0x1);
			_OV = 0;

			apph >>= -count;
			SetSZPF_Word(apph);
		}
	}

//  osd_printf_debug("SHLH: %x _CY: %d _Z: %d _OV: %d _S: %d\n", apph, _CY, _Z, _OV, _S);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opSHLW(void) /* TRUSTED */
{
	uint32_t appw;
	int8_t count;
	uint64_t tmp;

	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 2);

	F12LOADOP2WORD();

	count = (int8_t)(m_op1 & 0xFF);
	if (count > 0)
	{
		// left shift flags:
		// carry gets the last bit shifted out,
		// overflow is always CLEARed

		_OV = 0;

		// now handle carry
		tmp = appw & 0xffffffff;
		tmp <<= count;
		SetCFL(tmp);    // set carry properly

		// do the actual shift...
		appw <<= count;

		// and set zero and sign
		SetSZPF_Long(appw);
	}
	else
	{
		if (count == 0)
		{
			// special case: clear carry and overflow, do nothing else
			_CY = _OV = 0;
			SetSZPF_Long(appw); // doc. is unclear if this is true...
		}
		else
		{
			// right shift flags:
			// carry = last bit shifted out
			// overflow always cleared
			tmp = (uint64_t)(appw & 0xffffffff);
			tmp >>= ((-count) - 1);
			_CY = (uint8_t)(tmp & 0x1);
			_OV = 0;

			appw >>= -count;
			SetSZPF_Long(appw);
		}
	}

	//osd_printf_debug("SHLW: %x _CY: %d _Z: %d _OV: %d _S: %d\n", appw, _CY, _Z, _OV, _S);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opSTPR(void)
{
	F12DecodeFirstOperand(ReadAM, 2);
	if (m_op1 <= 28)
		m_modwritevalw = m_reg[m_op1 + 36];
	else
	{
		return v60_fatal("Invalid operand on STPR PC=%x\n", PC);
	}
	F12WriteSecondOperand(2);
	F12END();
}


static uint32_t opSUBB(void) /* TRUSTED (C too!) */
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	SUBB(appb, (uint8_t)m_op1, 0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opSUBH(void) /* TRUSTED (C too!) */
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	SUBW(apph, (uint16_t)m_op1, 0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opSUBW(void) /* TRUSTED (C too!) */
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	SUBL(appw, (uint32_t)m_op1, 0);

	F12STOREOP2WORD();
	F12END();
}


static uint32_t opSUBCB(void)
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	SUBB(appb, (uint8_t)m_op1, _CY?1:0);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opSUBCH(void)
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	SUBW(apph, (uint16_t)m_op1, _CY?1:0);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opSUBCW(void)
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	SUBL(appw, (uint32_t)m_op1, _CY?1:0);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opTEST1(void)
{
	F12DecodeOperands(ReadAM, 2,ReadAM, 2);

	_CY = ((m_op2 & (1 << m_op1)) != 0);
	_Z = !(_CY);

	F12END();
}

static uint32_t opUPDPSWW(void)
{
	F12DecodeOperands(ReadAM, 2,ReadAM, 2);

	/* can only modify condition code and control fields */
	m_op2 &= 0xFFFFFF;
	m_op1 &= 0xFFFFFF;
	v60WritePSW((v60ReadPSW() & (~m_op2)) | (m_op1 & m_op2));

	F12END();
}

static uint32_t opUPDPSWH(void)
{
	F12DecodeOperands(ReadAM, 2,ReadAM, 2);

	/* can only modify condition code fields */
	m_op2 &= 0xFFFF;
	m_op1 &= 0xFFFF;
	v60WritePSW((v60ReadPSW() & (~m_op2)) | (m_op1 & m_op2));

	F12END();
}

static uint32_t opXCHB(void) /* TRUSTED */
{
	uint8_t appb, temp;

	F12DecodeOperands(ReadAMAddress, 0,ReadAMAddress, 0);

	F12LOADOP1BYTE();
	temp = appb;
	F12LOADOP2BYTE();
	F12STOREOP1BYTE();
	appb = temp;
	F12STOREOP2BYTE();

	F12END()
}

static uint32_t opXCHH(void) /* TRUSTED */
{
	uint16_t apph, temp;

	F12DecodeOperands(ReadAMAddress, 1,ReadAMAddress, 1);

	F12LOADOP1HALF();
	temp = apph;
	F12LOADOP2HALF();
	F12STOREOP1HALF();
	apph = temp;
	F12STOREOP2HALF();

	F12END()
}

static uint32_t opXCHW(void) /* TRUSTED */
{
	uint32_t appw, temp;

	F12DecodeOperands(ReadAMAddress, 2,ReadAMAddress, 2);

	F12LOADOP1WORD();
	temp = appw;
	F12LOADOP2WORD();
	F12STOREOP1WORD();
	appw = temp;
	F12STOREOP2WORD();

	F12END()
}

static uint32_t opXORB(void) /* TRUSTED (C too!) */
{
	uint8_t appb;
	F12DecodeOperands(ReadAM, 0,ReadAMAddress, 0);

	F12LOADOP2BYTE();

	XORB(appb, (uint8_t)m_op1);

	F12STOREOP2BYTE();
	F12END();
}

static uint32_t opXORH(void) /* TRUSTED (C too!) */
{
	uint16_t apph;
	F12DecodeOperands(ReadAM, 1,ReadAMAddress, 1);

	F12LOADOP2HALF();

	XORW(apph, (uint16_t)m_op1);

	F12STOREOP2HALF();
	F12END();
}

static uint32_t opXORW(void) /* TRUSTED (C too!) */
{
	uint32_t appw;
	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 2);

	F12LOADOP2WORD();

	XORL(appw, (uint32_t)m_op1);

	F12STOREOP2WORD();
	F12END();
}

static uint32_t opMULX(void)
{
	int32_t a, b;
	int64_t res;

	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 3);

	if (m_flag2)
	{
		a = m_reg[m_op2 & 0x1F];
	}
	else
	{
		a = V60_RD(m_op2);
	}

	res = mul_32x32(a, m_op1);

	b = (int32_t)((res >> 32)&0xffffffff);
	a = (int32_t)(res & 0xffffffff);

	_S = ((b & 0x80000000) != 0);
	_Z = (a == 0 && b == 0);

	if (m_flag2)
	{
		m_reg[m_op2 & 0x1F] = a;
		m_reg[(m_op2 & 0x1F) + 1] = b;
	}
	else
		V60_WQ(m_op2, res);

	F12END();
}

static uint32_t opMULUX(void)
{
	int32_t a, b;
	uint64_t res;

	F12DecodeOperands(ReadAM, 2,ReadAMAddress, 3);

	if (m_flag2)
	{
		a = m_reg[m_op2 & 0x1F];
	}
	else
	{
		a = V60_RD(m_op2);
	}

	res = mulu_32x32(a, m_op1);
	b = (int32_t)((res >> 32)&0xffffffff);
	a = (int32_t)(res & 0xffffffff);

	_S = ((b & 0x80000000) != 0);
	_Z = (a == 0 && b == 0);

	if (m_flag2)
	{
		m_reg[m_op2 & 0x1F] = a;
		m_reg[(m_op2 & 0x1F) + 1] = b;
	}
	else
		V60_WQ(m_op2, res);

	F12END();
}

// ---- op2.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont

#define F2END() \
	return 2 + m_amlength1 + m_amlength2;

#define F2LOADOPFLOAT(num)                              \
	if (m_flag##num)                                    \
		appf = u2f(m_reg[m_op##num]);               \
	else                                                    \
		appf = u2f(V60_RD(m_op##num));

#define F2STOREOPFLOAT(num)                              \
	if (m_flag##num)                                    \
		m_reg[m_op##num] = f2u(appf);               \
	else                                                    \
		V60_WD(m_op##num, f2u(appf));

static void F2DecodeFirstOperand(am_func DecodeOp1, uint8_t dim1)
{
	m_moddim = dim1;
	m_modm = m_instflags & 0x40;
	m_modadd = PC + 2;
	m_amlength1 = (DecodeOp1)();
	m_op1 = m_amout;
	m_flag1 = m_amflag;
}

static void F2DecodeSecondOperand(am_func DecodeOp2, uint8_t dim2)
{
	m_moddim = dim2;
	m_modm = m_instflags & 0x20;
	m_modadd = PC + 2 + m_amlength1;
	m_amlength2 = (DecodeOp2)();
	m_op2 = m_amout;
	m_flag2 = m_amflag;
}

static void F2WriteSecondOperand(uint8_t dim2)
{
	m_moddim = dim2;
	m_modm = m_instflags & 0x20;
	m_modadd = PC + 2 + m_amlength1;
	m_amlength2 = WriteAM();
}

static uint32_t opCVTWS(void)
{
	float val;

	F2DecodeFirstOperand(ReadAM, 2);

	// Convert to float
	val = (float)(int32_t)m_op1;
	m_modwritevalw = f2u(val);

	_OV = 0;
	_CY = (val < 0.0f);
	_S = ((m_modwritevalw & 0x80000000) != 0);
	_Z = (val == 0.0f);

	F2WriteSecondOperand(2);
	F2END();
}

static uint32_t opCVTSW(void)
{
	float val;

	F2DecodeFirstOperand(ReadAM, 2);

	// Apply RDI rounding control
	val = u2f(m_op1);
	switch (TKCW & 7)
	{
	case 0: val = roundf(val); break;
	case 1: val = floorf(val); break;
	case 2: val = ceilf(val); break;
	default: val = truncf(val); break;
	}

	// Convert to uint32_t
	m_modwritevalw = (uint32_t)(int64_t)val;

	_S = ((m_modwritevalw & 0x80000000) != 0);
	_OV = (_S && val >= 0.0f) || (!_S && val <= -1.0f);
	_Z = (m_modwritevalw == 0);

	F2WriteSecondOperand(2);
	F2END();
}

static uint32_t opMOVFS(void)
{
	F2DecodeFirstOperand(ReadAM, 2);
	m_modwritevalw = m_op1;
	F2WriteSecondOperand(2);
	F2END();
}

static uint32_t opNEGFS(void)
{
	float appf;

	F2DecodeFirstOperand(ReadAM, 2);
	F2DecodeSecondOperand(ReadAMAddress, 2);

	appf = -u2f(m_op1);

	_OV = 0;
	_CY = (appf < 0.0f);
	_S = ((f2u(appf) & 0x80000000) != 0);
	_Z = (appf == 0.0f);

	F2STOREOPFLOAT(2);
	F2END()
}

static uint32_t opABSFS(void)
{
	float appf;

	F2DecodeFirstOperand(ReadAM, 2);
	F2DecodeSecondOperand(ReadAMAddress, 2);

	appf = u2f(m_op1);

	if (appf < 0)
		appf = -appf;

	_OV = 0;
	_CY = 0;
	_S = ((f2u(appf) & 0x80000000) != 0);
	_Z = (appf == 0.0f);

	F2STOREOPFLOAT(2);
	F2END()
}

static uint32_t opADDFS(void)
{
	uint32_t appw;
	float appf;

	F2DecodeFirstOperand(ReadAM, 2);
	F2DecodeSecondOperand(ReadAMAddress, 2);

	F2LOADOPFLOAT(2);

	appf += u2f(m_op1);

	appw = f2u(appf);
	_OV = _CY = 0;
	_S = ((appw & 0x80000000) != 0);
	_Z = (appw == 0);

	F2STOREOPFLOAT(2);
	F2END()
}

static uint32_t opSUBFS(void)
{
	uint32_t appw;
	float appf;

	F2DecodeFirstOperand(ReadAM, 2);
	F2DecodeSecondOperand(ReadAMAddress, 2);

	F2LOADOPFLOAT(2);

	appf -= u2f(m_op1);

	appw = f2u(appf);
	_OV = _CY = 0;
	_S = ((appw & 0x80000000) != 0);
	_Z = (appw == 0);

	F2STOREOPFLOAT(2);
	F2END()
}

static uint32_t opMULFS(void)
{
	uint32_t appw;
	float appf;

	F2DecodeFirstOperand(ReadAM, 2);
	F2DecodeSecondOperand(ReadAMAddress, 2);

	F2LOADOPFLOAT(2);

	appf *= u2f(m_op1);

	appw = f2u(appf);
	_OV = _CY = 0;
	_S = ((appw & 0x80000000) != 0);
	_Z = (appw == 0);

	F2STOREOPFLOAT(2);
	F2END()
}

static uint32_t opDIVFS(void)
{
	uint32_t appw;
	float appf;

	F2DecodeFirstOperand(ReadAM, 2);
	F2DecodeSecondOperand(ReadAMAddress, 2);

	F2LOADOPFLOAT(2);

	appf /= u2f(m_op1);

	appw = f2u(appf);
	_OV = _CY = 0;
	_S = ((appw & 0x80000000) != 0);
	_Z = (appw == 0);

	F2STOREOPFLOAT(2);
	F2END()
}

static uint32_t opSCLFS(void)
{
	uint32_t appw;
	float appf;

	F2DecodeFirstOperand(ReadAM, 1);
	F2DecodeSecondOperand(ReadAMAddress, 2);

	F2LOADOPFLOAT(2);

	if ((int16_t)m_op1 < 0)
		appf /= 1 << -(int16_t)m_op1;
	else
		appf *= 1 << m_op1;

	appw = f2u(appf);
	_OV = _CY = 0;
	_S = ((appw & 0x80000000) != 0);
	_Z = (appw == 0);

	F2STOREOPFLOAT(2);
	F2END()
}

static uint32_t opCMPF(void)
{
	float appf;

	F2DecodeFirstOperand(ReadAM, 2);
	F2DecodeSecondOperand(ReadAM, 2);

	appf = u2f(m_op2) - u2f(m_op1);

	_Z = (appf == 0);
	_S = (appf < 0);
	_OV = 0;
	_CY = 0;

	F2END();
}

static uint32_t op5FUNHANDLED(void)
{
	return v60_fatal("Unhandled 5F opcode at %08x\n", PC);
}

static uint32_t op5CUNHANDLED(void)
{
	return v60_fatal("Unhandled 5C opcode at %08x\n", PC);
}

static const am_func s_Op5FTable[32] =
{
	opCVTWS,
	opCVTSW,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED,
	op5FUNHANDLED
};

static const am_func s_Op5CTable[32] =
{
	opCMPF,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	opMOVFS,
	opNEGFS,
	opABSFS,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,

	opSCLFS,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	opADDFS,
	opSUBFS,
	opMULFS,
	opDIVFS,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED,
	op5CUNHANDLED
};


static uint32_t op5F(void)
{
	m_instflags = OpRead8(PC + 1);
	return (s_Op5FTable[m_instflags & 0x1F])();
}


static uint32_t op5C(void)
{
	m_instflags = OpRead8(PC + 1);
	return (s_Op5CTable[m_instflags & 0x1F])();
}

// ---- op3.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
static uint32_t opINCB(void) /* TRUSTED */
{
	uint8_t appb;
	m_modadd = PC + 1;
	m_moddim = 0;

	m_amlength1 = ReadAMAddress();

	if (m_amflag)
		appb = (uint8_t)m_reg[m_amout];
	else
		appb = V60_RB(m_amout);

	ADDB(appb, 1, 0);

	if (m_amflag)
		SETREG8(m_reg[m_amout], appb);
	else
		V60_WB(m_amout, appb);

	return m_amlength1 + 1;
}

static uint32_t opINCH(void) /* TRUSTED */
{
	uint16_t apph;
	m_modadd = PC + 1;
	m_moddim = 1;

	m_amlength1 = ReadAMAddress();

	if (m_amflag)
		apph = (uint16_t)m_reg[m_amout];
	else
		apph = V60_RW(m_amout);

	ADDW(apph, 1, 0);

	if (m_amflag)
		SETREG16(m_reg[m_amout], apph);
	else
		V60_WW(m_amout, apph);

	return m_amlength1 + 1;
}

static uint32_t opINCW(void) /* TRUSTED */
{
	uint32_t appw;
	m_modadd = PC + 1;
	m_moddim = 2;

	m_amlength1 = ReadAMAddress();

	if (m_amflag)
		appw = m_reg[m_amout];
	else
		appw = V60_RD(m_amout);

	ADDL(appw, 1, 0);

	if (m_amflag)
		m_reg[m_amout] = appw;
	else
		V60_WD(m_amout, appw);

	return m_amlength1 + 1;
}

static uint32_t opDECB(void) /* TRUSTED */
{
	uint8_t appb;
	m_modadd = PC + 1;
	m_moddim = 0;

	m_amlength1 = ReadAMAddress();

	if (m_amflag)
		appb = (uint8_t)m_reg[m_amout];
	else
		appb = V60_RB(m_amout);

	SUBB(appb, 1, 0);

	if (m_amflag)
		SETREG8(m_reg[m_amout], appb);
	else
		V60_WB(m_amout, appb);

	return m_amlength1 + 1;
}

static uint32_t opDECH(void) /* TRUSTED */
{
	uint16_t apph;
	m_modadd = PC + 1;
	m_moddim = 1;

	m_amlength1 = ReadAMAddress();

	if (m_amflag)
		apph = (uint16_t)m_reg[m_amout];
	else
		apph = V60_RW(m_amout);

	SUBW(apph, 1, 0);

	if (m_amflag)
		SETREG16(m_reg[m_amout], apph);
	else
		V60_WW(m_amout, apph);

	return m_amlength1 + 1;
}

static uint32_t opDECW(void) /* TRUSTED */
{
	uint32_t appw;
	m_modadd = PC + 1;
	m_moddim = 2;

	m_amlength1 = ReadAMAddress();

	if (m_amflag)
		appw = m_reg[m_amout];
	else
		appw = V60_RD(m_amout);

	SUBL(appw, 1, 0);

	if (m_amflag)
		m_reg[m_amout] = appw;
	else
		V60_WD(m_amout, appw);

	return m_amlength1 + 1;
}

static uint32_t opJMP(void) /* TRUSTED */
{
	m_modadd = PC + 1;
	m_moddim = 0;

	// Read the address of the operand
	ReadAMAddress();

	// It cannot be a register!!
	assert(m_amflag == 0);

	// Jump there
	PC = m_amout;

	return 0;
}

static uint32_t opJSR(void) /* TRUSTED */
{
	m_modadd = PC + 1;
	m_moddim = 0;

	// Read the address of the operand
	m_amlength1 = ReadAMAddress();

	// It cannot be a register!!
	assert(m_amflag == 0);

	// Save NextPC into the stack
	SP -= 4;
	V60_WD(SP, PC + m_amlength1 + 1);

	// Jump there
	PC = m_amout;

	return 0;
}

static uint32_t opPREPARE(void)    /* somewhat TRUSTED */
{
	m_modadd = PC + 1;
	m_moddim = 2;

	// Read the operand
	m_amlength1 = ReadAM();

	// step 1: save frame pointer on the stack
	SP -= 4;
	V60_WD(SP, FP);

	// step 2: FP = new SP
	FP = SP;

	// step 3: SP -= operand
	SP -= m_amout;

	return m_amlength1 + 1;
}

static uint32_t opRET(void) /* TRUSTED */
{
	m_modadd = PC + 1;
	m_moddim = 2;

	// Read the operand
	ReadAM();

	// Read return address from stack
	PC = V60_RD(SP);
	SP +=4;

	// Restore AP from stack
	AP = V60_RD(SP);
	SP +=4;

	// Skip stack frame
	SP += m_amout;

	return 0;
}

static uint32_t opTRAP(void)
{
	uint32_t oldPSW;

	m_modadd = PC + 1;
	m_moddim = 0;

	// Read the operand
	m_amlength1 = ReadAM();

	// Normalize the flags
	NORMALIZEFLAGS();

	switch ((m_amout >> 4) & 0xF)
	{
	case 0:
		if (!_OV) return m_amlength1 + 1;
		else break;
	case 1:
		if (_OV) return m_amlength1 + 1;
		else break;
	case 2:
		if (!_CY) return m_amlength1 + 1;
		else break;
	case 3:
		if (_CY) return m_amlength1 + 1;
		else break;
	case 4:
		if (!_Z) return m_amlength1 + 1;
		else break;
	case 5:
		if (_Z) return m_amlength1 + 1;
		else break;
	case 6:
		if (!(_CY | _Z)) return m_amlength1 + 1;
		else break;
	case 7:
		if ((_CY | _Z)) return m_amlength1 + 1;
		else break;
	case 8:
		if (!_S) return m_amlength1 + 1;
		else break;
	case 9:
		if (_S) return m_amlength1 + 1;
		else break;
	case 10:
		break;
	case 11:
		return m_amlength1 + 1;
	case 12:
		if (!(_S^_OV)) return m_amlength1 + 1;
		else break;
	case 13:
		if ((_S^_OV)) return m_amlength1 + 1;
		else break;
	case 14:
		if (!((_S^_OV)|_Z)) return m_amlength1 + 1;
		else break;
	case 15:
		if (((_S^_OV)|_Z)) return m_amlength1 + 1;
		else break;
	}

	oldPSW = v60_update_psw_for_exception(0, 0);

	// Issue the software trap with interrupts
	SP -= 4;
	V60_WD(SP, EXCEPTION_CODE_AND_SIZE(0x3000 + 0x100 * (m_amout & 0xF), 4));

	SP -= 4;
	V60_WD(SP, oldPSW);

	SP -= 4;
	V60_WD(SP, PC + m_amlength1 + 1);

	PC = GETINTVECT(48 + (m_amout & 0xF));

	return 0;
}

static uint32_t opRETIU(void) /* TRUSTED */
{
	uint32_t newPSW;
	m_modadd = PC + 1;
	m_moddim = 1;

	// Read the operand
	ReadAM();

	// Restore PC and PSW from stack
	PC = V60_RD(SP);
	SP += 4;

	newPSW = V60_RD(SP);
	SP += 4;

	// Destroy stack frame
	SP += m_amout;

	v60WritePSW(newPSW);

	return 0;
}

static uint32_t opRETIS(void)
{
	uint32_t newPSW;

	m_modadd = PC + 1;
	m_moddim = 1;

	// Read the operand
	ReadAM();

	// Restore PC and PSW from stack
	PC = V60_RD(SP);
	SP += 4;

	newPSW = V60_RD(SP);
	SP += 4;

	// Destroy stack frame
	SP += m_amout;

	v60WritePSW(newPSW);

	return 0;
}

static uint32_t opSTTASK(void)
{
	int i;
	uint32_t adr;

	m_modadd = PC + 1;
	m_moddim = 2;

	m_amlength1 = ReadAM();

	adr = TR;

	v60WritePSW(v60ReadPSW() | 0x10000000);
	v60SaveStack();

	V60_WD(adr, TKCW);
	adr += 4;
	if(SYCW & 0x100) {
		V60_WD(adr, L0SP);
		adr += 4;
	}
	if(SYCW & 0x200) {
		V60_WD(adr, L1SP);
		adr += 4;
	}
	if(SYCW & 0x400) {
		V60_WD(adr, L2SP);
		adr += 4;
	}
	if(SYCW & 0x800) {
		V60_WD(adr, L3SP);
		adr += 4;
	}

	// 31 registers supported, _not_ 32
	for(i = 0; i < 31; i++)
		if(m_amout & (1 << i)) {
			V60_WD(adr, m_reg[i]);
			adr += 4;
		}

	// #### Ignore the virtual addressing crap.

	return m_amlength1 + 1;
}

static uint32_t opGETPSW(void)
{
	m_modadd = PC + 1;
	m_moddim = 2;
	m_modwritevalw = v60ReadPSW();

	// Write PSW to the operand
	m_amlength1 = WriteAM();

	return m_amlength1 + 1;
}

static uint32_t opTASI(void)
{
	uint8_t appb;
	m_modadd = PC + 1;
	m_moddim = 0;

	// Load the address of the operand
	m_amlength1 = ReadAMAddress();

	// Load uint8_t from the address
	if (m_amflag)
		appb = (uint8_t)m_reg[m_amout & 0x1f];
	else
		appb = V60_RB(m_amout);

	// Set the flags for SUB appb, FF
	SUBB(appb, 0xff, 0);

	// Write FF in the operand
	if (m_amflag)
		SETREG8(m_reg[m_amout & 0x1f], 0xff);
	else
		V60_WB(m_amout, 0xff);

	return m_amlength1 + 1;
}

static uint32_t opCLRTLB(void)
{
	m_modadd = PC + 1;
	m_moddim = 2;

	// Read the operand
	m_amlength1 = ReadAM();

	// @@@ TLB not yet emulated

	return m_amlength1 + 1;
}

static uint32_t opPOPM(void)
{
	int i;

	m_modadd = PC + 1;
	m_moddim = 2;

	// Read the bit register list
	m_amlength1 = ReadAM();

	for (i = 0;i < 31;i++)
		if (m_amout & (1 << i))
		{
			m_reg[i] = V60_RD(SP);
			SP += 4;
		}

	if (m_amout & (1 << 31))
	{
		v60WritePSW((v60ReadPSW() & 0xffff0000) | V60_RW(SP));
		SP += 4;
	}

	return m_amlength1 + 1;
}

static uint32_t opPUSHM(void)
{
	int i;

	m_modadd = PC + 1;
	m_moddim = 2;

	// Read the bit register list
	m_amlength1 = ReadAM();

	if (m_amout & (1 << 31))
	{
		SP -= 4;
		V60_WD(SP, v60ReadPSW());
	}

	for (i = 0;i < 31;i++)
		if (m_amout & (1 << (30 - i)))
		{
			SP -= 4;
			V60_WD(SP, m_reg[(30 - i)]);
		}


	return m_amlength1 + 1;
}

static uint32_t opTESTB(void) /* TRUSTED */
{
	m_modadd = PC + 1;
	m_moddim = 0;

	// Read the operand
	m_amlength1 = ReadAM();

	_Z = (m_amout == 0);
	_S = ((m_amout & 0x80) != 0);
	_CY = 0;
	_OV = 0;

	return m_amlength1 + 1;
}

static uint32_t opTESTH(void) /* TRUSTED */
{
	m_modadd = PC + 1;
	m_moddim = 1;

	// Read the operand
	m_amlength1 = ReadAM();

	_Z = (m_amout == 0);
	_S = ((m_amout & 0x8000) != 0);
	_CY = 0;
	_OV = 0;

	return m_amlength1 + 1;
}

static uint32_t opTESTW(void) /* TRUSTED */
{
	m_modadd = PC + 1;
	m_moddim = 2;

	// Read the operand
	m_amlength1 = ReadAM();

	_Z = (m_amout == 0);
	_S = ((m_amout & 0x80000000) != 0);
	_CY = 0;
	_OV = 0;

	return m_amlength1 + 1;
}

static uint32_t opPUSH(void)
{
	m_modadd = PC + 1;
	m_moddim = 2;

	m_amlength1 = ReadAM();

	SP-=4;
	V60_WD(SP, m_amout);

	return m_amlength1 + 1;
}

static uint32_t opPOP(void)
{
	m_modadd = PC + 1;
	m_moddim = 2;
	m_modwritevalw = V60_RD(SP);
	SP +=4;
	m_amlength1 = WriteAM();

	return m_amlength1 + 1;
}


static uint32_t opINCB_0(void) { m_modm = 0; return opINCB(); }
static uint32_t opINCB_1(void) { m_modm = 1; return opINCB(); }
static uint32_t opINCH_0(void) { m_modm = 0; return opINCH(); }
static uint32_t opINCH_1(void) { m_modm = 1; return opINCH(); }
static uint32_t opINCW_0(void) { m_modm = 0; return opINCW(); }
static uint32_t opINCW_1(void) { m_modm = 1; return opINCW(); }

static uint32_t opDECB_0(void) { m_modm = 0; return opDECB(); }
static uint32_t opDECB_1(void) { m_modm = 1; return opDECB(); }
static uint32_t opDECH_0(void) { m_modm = 0; return opDECH(); }
static uint32_t opDECH_1(void) { m_modm = 1; return opDECH(); }
static uint32_t opDECW_0(void) { m_modm = 0; return opDECW(); }
static uint32_t opDECW_1(void) { m_modm = 1; return opDECW(); }

static uint32_t opJMP_0(void) { m_modm = 0; return opJMP(); }
static uint32_t opJMP_1(void) { m_modm = 1; return opJMP(); }

static uint32_t opJSR_0(void) { m_modm = 0; return opJSR(); }
static uint32_t opJSR_1(void) { m_modm = 1; return opJSR(); }

static uint32_t opPREPARE_0(void) { m_modm = 0; return opPREPARE(); }
static uint32_t opPREPARE_1(void) { m_modm = 1; return opPREPARE(); }

static uint32_t opRET_0(void) { m_modm = 0; return opRET(); }
static uint32_t opRET_1(void) { m_modm = 1; return opRET(); }

static uint32_t opTRAP_0(void) { m_modm = 0; return opTRAP(); }
static uint32_t opTRAP_1(void) { m_modm = 1; return opTRAP(); }

static uint32_t opRETIU_0(void) { m_modm = 0; return opRETIU(); }
static uint32_t opRETIU_1(void) { m_modm = 1; return opRETIU(); }

static uint32_t opRETIS_0(void) { m_modm = 0; return opRETIS(); }
static uint32_t opRETIS_1(void) { m_modm = 1; return opRETIS(); }

static uint32_t opGETPSW_0(void) { m_modm = 0; return opGETPSW(); }
static uint32_t opGETPSW_1(void) { m_modm = 1; return opGETPSW(); }

static uint32_t opTASI_0(void) { m_modm = 0; return opTASI(); }
static uint32_t opTASI_1(void) { m_modm = 1; return opTASI(); }

static uint32_t opCLRTLB_0(void) { m_modm = 0; return opCLRTLB(); }
static uint32_t opCLRTLB_1(void) { m_modm = 1; return opCLRTLB(); }

static uint32_t opPOPM_0(void) { m_modm = 0; return opPOPM(); }
static uint32_t opPOPM_1(void) { m_modm = 1; return opPOPM(); }

static uint32_t opPUSHM_0(void) { m_modm = 0; return opPUSHM(); }
static uint32_t opPUSHM_1(void) { m_modm = 1; return opPUSHM(); }

static uint32_t opTESTB_0(void) { m_modm = 0; return opTESTB(); }
static uint32_t opTESTB_1(void) { m_modm = 1; return opTESTB(); }

static uint32_t opTESTH_0(void) { m_modm = 0; return opTESTH(); }
static uint32_t opTESTH_1(void) { m_modm = 1; return opTESTH(); }

static uint32_t opTESTW_0(void) { m_modm = 0; return opTESTW(); }
static uint32_t opTESTW_1(void) { m_modm = 1; return opTESTW(); }

static uint32_t opPUSH_0(void) { m_modm = 0; return opPUSH(); }
static uint32_t opPUSH_1(void) { m_modm = 1; return opPUSH(); }

static uint32_t opPOP_0(void) { m_modm = 0; return opPOP(); }
static uint32_t opPOP_1(void) { m_modm = 1; return opPOP(); }

static uint32_t opSTTASK_0(void) { m_modm = 0; return opSTTASK(); }
static uint32_t opSTTASK_1(void) { m_modm = 1; return opSTTASK(); }

// ---- op4.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
/*
    FULLY TRUSTED
*/

static uint32_t opBGT8(void) /* TRUSTED */
{
	NORMALIZEFLAGS();

	if (!((_S ^ _OV) | _Z))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBGT16(void) /* TRUSTED */
{
	NORMALIZEFLAGS();

	if (!((_S ^ _OV) | _Z))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}


static uint32_t opBGE8(void) /* TRUSTED */
{
	NORMALIZEFLAGS();

	if (!(_S ^ _OV))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBGE16(void) /* TRUSTED */
{
	NORMALIZEFLAGS();

	if (!(_S ^ _OV))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBLT8(void) /* TRUSTED */
{
	NORMALIZEFLAGS();

	if ((_S ^ _OV))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBLT16(void) /* TRUSTED */
{
	NORMALIZEFLAGS();

	if ((_S ^ _OV))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}


static uint32_t opBLE8(void) /* TRUSTED */
{
	NORMALIZEFLAGS();

	if (((_S ^ _OV) | _Z))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBLE16(void) /* TRUSTED */
{
	NORMALIZEFLAGS();

	if (((_S ^ _OV) | _Z))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBH8(void) /* TRUSTED */
{
	if (!(_CY | _Z))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBH16(void) /* TRUSTED */
{
	if (!(_CY | _Z))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBNH8(void) /* TRUSTED */
{
	if ((_CY | _Z))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBNH16(void) /* TRUSTED */
{
	if ((_CY | _Z))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBNL8(void) /* TRUSTED */
{
	if (!(_CY))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBNL16(void) /* TRUSTED */
{
	if (!(_CY))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBL8(void) /* TRUSTED */
{
	if ((_CY))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBL16(void) /* TRUSTED */
{
	if ((_CY))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBNE8(void) /* TRUSTED */
{
	if (!(_Z))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBNE16(void) /* TRUSTED */
{
	if (!(_Z))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBE8(void) /* TRUSTED */
{
	if ((_Z))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBE16(void) /* TRUSTED */
{
	if ((_Z))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBNV8(void) /* TRUSTED */
{
	if (!(_OV))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBNV16(void) /* TRUSTED */
{
	if (!(_OV))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBV8(void) /* TRUSTED */
{
	if ((_OV))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBV16(void) /* TRUSTED */
{
	if ((_OV))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBP8(void) /* TRUSTED */
{
	if (!(_S))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBP16(void) /* TRUSTED */
{
	if (!(_S))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBN8(void) /* TRUSTED */
{
	if ((_S))
	{
		PC += (int8_t)OpRead8(PC + 1);
		return 0;
	}

	return 2;
}

static uint32_t opBN16(void) /* TRUSTED */
{
	if ((_S))
	{
		PC += (int16_t)OpRead16(PC + 1);
		return 0;
	}

	return 3;
}

static uint32_t opBR8(void) /* TRUSTED */
{
	PC += (int8_t)OpRead8(PC + 1);
	return 0;
}

static uint32_t opBR16(void) /* TRUSTED */
{
	PC += (int16_t)OpRead16(PC + 1);
	return 0;
}

static uint32_t opBSR(void) /* TRUSTED */
{
	// Save Next PC onto the stack
	SP -= 4;
	V60_WD(SP, PC + 3);

	// Jump to subroutine
	PC += (int16_t)OpRead16(PC + 1);
	return 0;
}

// ---- op5.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
/*
 *  HALT: must add log
 */

static uint32_t opBRK(void)
{
/*
    uint32_t oldPSW = v60_update_psw_for_exception(0, 0);

    SP -=4;
    V60_WD(SP, EXCEPTION_CODE_AND_SIZE(0x0d00, 4));
    SP -=4;
    V60_WD(SP, oldPSW);
    SP -=4;
    V60_WD(SP, PC + 1);
    PC = GETINTVECT(13);
*/
	v60_log("Skipping BRK opcode! PC=%x", PC);

	return 1;
}

static uint32_t opBRKV(void)
{
	uint32_t oldPSW = v60_update_psw_for_exception(0, 0);

	SP -=4;
	V60_WD(SP, PC);
	SP -=4;
	V60_WD(SP, EXCEPTION_CODE_AND_SIZE(0x1501, 4));
	SP -=4;
	V60_WD(SP, oldPSW);
	SP -=4;
	V60_WD(SP, PC + 1);
	PC = GETINTVECT(21);

	return 0;
}

static uint32_t opCLRTLBA(void)
{
	// @@@ TLB not yet supported
	v60_log("Skipping CLRTLBA opcode! PC=%x\n", PC);
	return 1;
}

static uint32_t opDISPOSE(void)
{
	SP = FP;
	FP = V60_RD(SP);
	SP +=4;

	return 1;
}

static uint32_t opHALT(void)
{
	// @@@ It should wait for an interrupt to occur
	//v60_log("HALT found: skipping");
	return 1;
}

static uint32_t opNOP(void) /* TRUSTED */
{
	return 1;
}

static uint32_t opRSR(void)
{
	PC = V60_RD(SP);
	SP +=4;

	return 0;
}

static uint32_t opTRAPFL(void)
{
	if ((TKCW & 0x1F0) & ((v60ReadPSW() & 0x1F00) >> 4))
	{
		// @@@ FPU exception
		return v60_fatal("Hit TRAPFL! PC=%x\n", PC);
	}

	return 1;
}

// ---- op6.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
/*
    FULLY TRUSTED
*/

static uint32_t opTB(int reg) /* TRUSTED */
{
	if (m_reg[reg] == 0)
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBGT(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	NORMALIZEFLAGS();
	if ((m_reg[reg] != 0) && !((_S ^ _OV) | _Z))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBLE(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	NORMALIZEFLAGS();
	if ((m_reg[reg] != 0) && ((_S ^ _OV) | _Z))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}


static uint32_t opDBGE(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	NORMALIZEFLAGS();
	if ((m_reg[reg] != 0) && !(_S ^ _OV))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBLT(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	NORMALIZEFLAGS();
	if ((m_reg[reg] != 0) && (_S ^ _OV))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBH(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if ((m_reg[reg] != 0) && !(_CY | _Z))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBNH(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if ((m_reg[reg] != 0) && (_CY | _Z))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}


static uint32_t opDBL(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if ((m_reg[reg] != 0) && (_CY))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBNL(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if ((m_reg[reg] != 0) && !(_CY))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBE(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if ((m_reg[reg] != 0) && (_Z))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBNE(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if ((m_reg[reg] != 0) && !(_Z))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBV(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if ((m_reg[reg] != 0) && (_OV))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBNV(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if ((m_reg[reg] != 0) && !(_OV))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBN(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if ((m_reg[reg] != 0) && (_S))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBP(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if ((m_reg[reg] != 0) && !(_S))
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static uint32_t opDBR(int reg) /* TRUSTED */
{
	m_reg[reg]--;

	if (m_reg[reg] != 0)
	{
		PC += (int16_t)OpRead16(PC + 2);
		return 0;
	}

	return 4;
}

static const op6_func s_OpC6Table[8] = /* TRUSTED */
{
	opDBV,
	opDBL,
	opDBE,
	opDBNH,
	opDBN,
	opDBR,
	opDBLT,
	opDBLE
};

static const op6_func s_OpC7Table[8] = /* TRUSTED */
{
	opDBNV,
	opDBNL,
	opDBNE,
	opDBH,
	opDBP,
	opTB,
	opDBGE,
	opDBGT
};


static uint32_t opC6(void) /* TRUSTED */
{
	uint8_t appb = OpRead8(PC + 1);
	return (s_OpC6Table[appb >> 5])(appb & 0x1f);
}

static uint32_t opC7(void) /* TRUSTED */
{
	uint8_t appb = OpRead8(PC + 1);
	return (s_OpC7Table[appb >> 5])(appb & 0x1f);
}

// ---- op7a.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
/*
 * CMPC: What happens to _S flag if the strings are identical?
 *   I suppose that it will be cleared. And is it set or cleared
 *   when the first one is a substring of the second? I suppose
 *   cleared (since _S should be (src > dst))
 * MOVC: Why MOVCS does not exist in downward version?
 * SHCHDB / SHCHDH: R27 is filled with the offset from the start or from the end?
 *
 * Strange stuff:
 *   SCHC opcodes does *not* modify _Z flag as stated in V60 manual:
 *   they do the opposite (set if not found, reset if found)
 */

#define F7AEND()  \
	return m_amlength1 + m_amlength2 + 4;

#define F7BEND()  \
	return m_amlength1 + m_amlength2 + 3;

#define F7CEND()  \
	return m_amlength1 + m_amlength2 + 3;

#define F7BCREATEBITMASK(x) \
	x = ((1 << (x)) - 1)

#define F7CCREATEBITMASK(x) \
	x = ((1 << (x)) - 1)

static void F7aDecodeOperands(am_func DecodeOp1, uint8_t dim1, am_func DecodeOp2, uint8_t dim2)
{
	uint8_t appb;
	// Decode first operand
	m_moddim = dim1;
	m_modm = m_subop & 0x40;
	m_modadd = PC + 2;
	m_amlength1 = (DecodeOp1)();
	m_flag1 = m_amflag;
	m_op1 = m_amout;

	// Decode length
	appb = OpRead8(PC + 2 + m_amlength1);
	if (appb & 0x80)
		m_lenop1 = m_reg[appb & 0x1F];
	else
		m_lenop1 = appb;

	// Decode second operand
	m_moddim = dim2;
	m_modm = m_subop & 0x20;
	m_modadd = PC + 3 + m_amlength1;
	m_amlength2 = (DecodeOp2)();
	m_flag2 = m_amflag;
	m_op2 = m_amout;

	// Decode length
	appb = OpRead8(PC + 3 + m_amlength1 + m_amlength2);
	if (appb & 0x80)
		m_lenop2 = m_reg[appb & 0x1F];
	else
		m_lenop2 = appb;
}

static void F7bDecodeFirstOperand(am_func DecodeOp1, uint8_t dim1)
{
	uint8_t appb;
	// Decode first operand
	m_moddim = dim1;
	m_modm = m_subop & 0x40;
	m_modadd = PC + 2;
	m_amlength1 = (DecodeOp1)();
	m_flag1 = m_amflag;
	m_op1 = m_amout;

	// Decode ext
	appb = OpRead8(PC + 2 + m_amlength1);
	if (appb & 0x80)
		m_lenop1 = m_reg[appb & 0x1F];
	else
		m_lenop1 = appb;
}


static void F7bWriteSecondOperand(uint8_t dim2)
{
	m_moddim = dim2;
	m_modm = m_subop & 0x20;
	m_modadd = PC + 3 + m_amlength1;
	m_amlength2 = WriteAM();
}


static void F7bDecodeOperands(am_func DecodeOp1, uint8_t dim1, am_func DecodeOp2, uint8_t dim2)
{
	// Decode first operand
	F7bDecodeFirstOperand(DecodeOp1, dim1);
	m_bamoffset1 = m_bamoffset;

	// Decode second operand
	m_moddim = dim2;
	m_modm = m_subop & 0x20;
	m_modadd = PC + 3 + m_amlength1;
	m_amlength2 = (DecodeOp2)();
	m_flag2 = m_amflag;
	m_op2 = m_amout;
	m_bamoffset2 = m_bamoffset;
}

static void F7cDecodeOperands(am_func DecodeOp1, uint8_t dim1, am_func DecodeOp2, uint8_t dim2)
{
	uint8_t appb;
	// Decode first operand
	m_moddim = dim1;
	m_modm = m_subop & 0x40;
	m_modadd = PC + 2;
	m_amlength1 = (DecodeOp1)();
	m_flag1 = m_amflag;
	m_op1 = m_amout;

	// Decode second operand
	m_moddim = dim2;
	m_modm = m_subop & 0x20;
	m_modadd = PC + 2 + m_amlength1;
	m_amlength2 = (DecodeOp2)();
	m_flag2 = m_amflag;
	m_op2 = m_amout;

	// Decode ext
	appb = OpRead8(PC + 2 + m_amlength1 + m_amlength2);
	if (appb & 0x80)
		m_lenop1 = m_reg[appb & 0x1F];
	else
		m_lenop1 = appb;
}

#define F7CLOADOP1BYTE(appb) \
	if (m_flag1) \
		appb = (uint8_t)(m_reg[m_op1]&0xFF); \
	else \
		appb = V60_RB(m_op1);

#define F7CLOADOP2BYTE(appb) \
	if (m_flag2) \
		appb = (uint8_t)(m_reg[m_op2]&0xFF); \
	else \
		appb = V60_RB(m_op2);


#define F7CSTOREOP2BYTE() \
	if (m_flag2) \
		SETREG8(m_reg[m_op2], appb); \
	else \
		V60_WB(m_op2, appb);

#define F7CSTOREOP2HALF() \
	if (m_flag2) \
		SETREG16(m_reg[m_op2], apph); \
	else \
		V60_WW(m_op2, apph);

static uint32_t opCMPSTRB(uint8_t bFill, uint8_t bStop)
{
	uint32_t i, dest;
	uint8_t c1, c2;

	F7aDecodeOperands(ReadAMAddress, 0,ReadAMAddress, 0);

	// Filling
	if (bFill)
	{
		if (m_lenop1 < m_lenop2)
		{
			for (i = m_lenop1; i < m_lenop2; i++)
				V60_WB(m_op1 + i,(uint8_t)R26);
		}
		else if (m_lenop2 < m_lenop1)
		{
			for (i = m_lenop2; i < m_lenop1; i++)
				V60_WB(m_op2 + i,(uint8_t)R26);
		}
	}

	dest = (m_lenop1 < m_lenop2 ? m_lenop1 : m_lenop2);

	_Z = 0;
	_S = 0;
	if (bStop) _CY = 1;

	for (i = 0; i < dest; i++)
	{
		c1 = V60_RB(m_op1 + i);
		c2 = V60_RB(m_op2 + i);

		if (c1 > c2)
		{
			_S = 1;   break;
		}
		else if (c2 > c1)
		{
			_S = 0;   break;
		}

		if (bStop)
			if (c1 == (uint8_t)R26 || c2 == (uint8_t)R26)
			{
				_CY = 0;
				break;
			}
	}

	R28 = m_lenop1 + i;
	R27 = m_lenop2 + i;

	if (i == dest)
	{
		if (m_lenop1 > m_lenop2)
			_S = 1;
		else if (m_lenop2 > m_lenop1)
			_S = 0;
		else
			_Z = 1;
	}

	F7AEND();
}

static uint32_t opCMPSTRH(uint8_t bFill, uint8_t bStop)
{
	uint32_t i, dest;
	uint16_t c1, c2;

	F7aDecodeOperands(ReadAMAddress, 0,ReadAMAddress, 0);

	// Filling
	if (bFill)
	{
		if (m_lenop1 < m_lenop2)
		{
			for (i = m_lenop1; i < m_lenop2; i++)
				V60_WW(m_op1 + i * 2,(uint16_t)R26);
		}
		else if (m_lenop2 < m_lenop1)
		{
			for (i = m_lenop2; i < m_lenop1; i++)
				V60_WW(m_op2 + i * 2,(uint16_t)R26);
		}
	}

	dest = (m_lenop1 < m_lenop2 ? m_lenop1 : m_lenop2);

	_Z = 0;
	_S = 0;
	if (bStop) _CY = 1;

	for (i = 0; i < dest; i++)
	{
		c1 = V60_RW(m_op1 + i * 2);
		c2 = V60_RW(m_op2 + i * 2);

		if (c1 > c2)
		{
			_S = 1;   break;
		}
		else if (c2 > c1)
		{
			_S = 0;   break;
		}

		if (bStop)
			if (c1 == (uint16_t)R26 || c2 == (uint16_t)R26)
			{
				_CY = 0;
				break;
			}
	}

	R28 = m_lenop1 + i * 2;
	R27 = m_lenop2 + i * 2;

	if (i == dest)
	{
		if (m_lenop1 > m_lenop2)
			_S = 1;
		else if (m_lenop2 > m_lenop1)
			_S = 0;
		else
			_Z = 1;
	}

	F7AEND();
}



static uint32_t opMOVSTRUB(uint8_t bFill, uint8_t bStop) /* TRUSTED (0, 0) (1, 0) */
{
	uint32_t i, dest;
	uint8_t c1;

//  if (bStop)
//  {
//      int a = 1;
//  }

	F7aDecodeOperands(ReadAMAddress, 0,ReadAMAddress, 0);

	dest = (m_lenop1 < m_lenop2 ? m_lenop1 : m_lenop2);

	for (i = 0; i < dest; i++)
	{
		V60_WB(m_op2 + i,(c1 = V60_RB(m_op1 + i)));

		if (bStop && c1 == (uint8_t)R26)
			break;
	}

	R28 = m_op1 + i;
	R27 = m_op2 + i;

	if (bFill && m_lenop1 < m_lenop2)
	{
		for (;i < m_lenop2; i++)
			V60_WB(m_op2 + i,(uint8_t)R26);

		R27 = m_op2 + i;
	}


	F7AEND();
}

static uint32_t opMOVSTRDB(uint8_t bFill, uint8_t bStop)
{
	uint32_t i, dest;
	uint8_t c1;

	F7aDecodeOperands(ReadAMAddress, 0,ReadAMAddress, 0);

	dest = (m_lenop1 < m_lenop2 ? m_lenop1 : m_lenop2);

	for (i = 0; i < dest; i++)
	{
		V60_WB(m_op2 + (dest - i - 1),(c1 = V60_RB(m_op1 + (dest - i - 1))));

		if (bStop && c1 == (uint8_t)R26)
			break;
	}

	R28 = m_op1 + (m_lenop1 - i - 1);
	R27 = m_op2 + (m_lenop2 - i - 1);

	if (bFill && m_lenop1 < m_lenop2)
	{
		for (;i < m_lenop2; i++)
			V60_WB(m_op2 + dest + (m_lenop2 - i - 1),(uint8_t)R26);

		R27 = m_op2 + (m_lenop2 - i - 1);
	}


	F7AEND();
}


static uint32_t opMOVSTRUH(uint8_t bFill, uint8_t bStop) /* TRUSTED (0, 0) (1, 0) */
{
	uint32_t i, dest;
	uint16_t c1;

//  if (bStop)
//  {   int a = 1; }

	F7aDecodeOperands(ReadAMAddress, 1,ReadAMAddress, 1);

	dest = (m_lenop1 < m_lenop2 ? m_lenop1 : m_lenop2);

	for (i = 0; i < dest; i++)
	{
		V60_WW(m_op2 + i * 2,(c1 = V60_RW(m_op1 + i * 2)));

		if (bStop && c1 == (uint16_t)R26)
			break;
	}

	R28 = m_op1 + i * 2;
	R27 = m_op2 + i * 2;

	if (bFill && m_lenop1 < m_lenop2)
	{
		for (;i < m_lenop2; i++)
			V60_WW(m_op2 + i * 2,(uint16_t)R26);

		R27 = m_op2 + i * 2;
	}

	F7AEND();
}

static uint32_t opMOVSTRDH(uint8_t bFill, uint8_t bStop)
{
	uint32_t i, dest;
	uint16_t c1;

//  if (bFill | bStop)
//  { int a = 1; }

	F7aDecodeOperands(ReadAMAddress, 1,ReadAMAddress, 1);

//  if (m_lenop1 != m_lenop2)
//  { int a = 1; }

	dest = (m_lenop1 < m_lenop2 ? m_lenop1 : m_lenop2);

	for (i = 0; i < dest; i++)
	{
		V60_WW(m_op2 + (dest - i - 1) * 2,(c1 = V60_RW(m_op1 + (dest - i - 1) * 2)));

		if (bStop && c1 == (uint16_t)R26)
			break;
	}

	R28 = m_op1 + (m_lenop1 - i - 1) * 2;
	R27 = m_op2 + (m_lenop2 - i - 1) * 2;

	if (bFill && m_lenop1 < m_lenop2)
	{
		for (;i < m_lenop2; i++)
			V60_WW(m_op2 + (m_lenop2 - i - 1) * 2,(uint16_t)R26);

		R27 = m_op2 + (m_lenop2 - i - 1) * 2;
	}

	F7AEND();
}

static uint32_t opSEARCHUB(uint8_t bSearch)
{
	uint8_t appb;
	uint32_t i;

	F7bDecodeOperands(ReadAMAddress, 0,ReadAM, 0);

	for (i = 0; i < m_lenop1; i++)
	{
		appb = (V60_RB(m_op1 + i) == (uint8_t)m_op2);
		if ((bSearch && appb) || (!bSearch && !appb))
			break;
	}

	R28 = m_op1 + i;
	R27 = i;

	// This is the opposite as stated in V60 manual...
	if (i != m_lenop1)
		_Z = 0;
	else
		_Z = 1;

	F7BEND();
}

static uint32_t opSEARCHUH(uint8_t bSearch)
{
	uint8_t appb;
	uint32_t i;

	F7bDecodeOperands(ReadAMAddress, 1,ReadAM, 1);

	for (i = 0; i < m_lenop1; i++)
	{
		appb = (V60_RW(m_op1 + i * 2) == (uint16_t)m_op2);
		if ((bSearch && appb) || (!bSearch && !appb))
			break;
	}

	R28 = m_op1 + i * 2;
	R27 = i;

	if (i != m_lenop1)
		_Z = 0;
	else
		_Z = 1;

	F7BEND();
}

static uint32_t opSEARCHDB(uint8_t bSearch)
{
	uint8_t appb;
	int32_t i;

	F7bDecodeOperands(ReadAMAddress, 0,ReadAM, 0);

	for (i = m_lenop1; i >= 0; i--)
	{
		appb = (V60_RB(m_op1 + i) == (uint8_t)m_op2);
		if ((bSearch && appb) || (!bSearch && !appb))
			break;
	}

	R28 = m_op1 + i;
	R27 = i;

	// This is the opposite as stated in V60 manual...
	if ((uint32_t)i != m_lenop1)
		_Z = 0;
	else
		_Z = 1;

	F7BEND();
}

static uint32_t opSEARCHDH(uint8_t bSearch)
{
	uint8_t appb;
	int32_t i;

	F7bDecodeOperands(ReadAMAddress, 1,ReadAM, 1);

	for (i = m_lenop1 - 1; i >= 0; i--)
	{
		appb = (V60_RW(m_op1 + i * 2) == (uint16_t)m_op2);
		if ((bSearch && appb) || (!bSearch && !appb))
			break;
	}

	R28 = m_op1 + i * 2;
	R27 = i;

	if ((uint32_t)i != m_lenop1)
		_Z = 0;
	else
		_Z = 1;

	F7BEND();
}


static uint32_t opSCHCUB(void) { return opSEARCHUB(1); }
static uint32_t opSCHCUH(void) { return opSEARCHUH(1); }
static uint32_t opSCHCDB(void) { return opSEARCHDB(1); }
static uint32_t opSCHCDH(void) { return opSEARCHDH(1); }
static uint32_t opSKPCUB(void) { return opSEARCHUB(0); }
static uint32_t opSKPCUH(void) { return opSEARCHUH(0); }
static uint32_t opSKPCDB(void) { return opSEARCHDB(0); }
static uint32_t opSKPCDH(void) { return opSEARCHDH(0); }

static uint32_t opCMPCB(void) { return opCMPSTRB(0, 0); }
static uint32_t opCMPCH(void) { return opCMPSTRH(0, 0); }
static uint32_t opCMPCFB(void) { return opCMPSTRB(1, 0); }
static uint32_t opCMPCFH(void) { return opCMPSTRH(1, 0); }
static uint32_t opCMPCSB(void) { return opCMPSTRB(0, 1); }
static uint32_t opCMPCSH(void) { return opCMPSTRH(0, 1); }

static uint32_t opMOVCUB(void) { return opMOVSTRUB(0, 0); }
static uint32_t opMOVCUH(void) { return opMOVSTRUH(0, 0); }
static uint32_t opMOVCFUB(void) { return opMOVSTRUB(1, 0); }
static uint32_t opMOVCFUH(void) { return opMOVSTRUH(1, 0); }
static uint32_t opMOVCSUB(void) { return opMOVSTRUB(0, 1); }
static uint32_t opMOVCSUH(void) { return opMOVSTRUH(0, 1); }

static uint32_t opMOVCDB(void) { return opMOVSTRDB(0, 0); }
static uint32_t opMOVCDH(void) { return opMOVSTRDH(0, 0); }
static uint32_t opMOVCFDB(void) { return opMOVSTRDB(1, 0); }
static uint32_t opMOVCFDH(void) { return opMOVSTRDH(1, 0); }

static uint32_t opEXTBFZ(void) /* TRUSTED */
{
	F7bDecodeFirstOperand(BitReadAM, 11);

	F7BCREATEBITMASK(m_lenop1);

	m_modwritevalw = (m_op1 >> m_bamoffset) & m_lenop1;

	F7bWriteSecondOperand(2);

	F7BEND();
}

static uint32_t opEXTBFS(void) /* TRUSTED */
{
	F7bDecodeFirstOperand(BitReadAM, 11);

	F7BCREATEBITMASK(m_lenop1);

	m_modwritevalw = (m_op1 >> m_bamoffset) & m_lenop1;
	if (m_modwritevalw & ((m_lenop1 + 1) >> 1))
		m_modwritevalw |= ~m_lenop1;

	F7bWriteSecondOperand(2);

	F7BEND();
}

static uint32_t opEXTBFL(void)
{
	uint32_t appw;

	F7bDecodeFirstOperand(BitReadAM, 11);

	appw = m_lenop1;
	F7BCREATEBITMASK(m_lenop1);

	m_modwritevalw = (m_op1 >> m_bamoffset) & m_lenop1;
	m_modwritevalw <<= 32 - appw;

	F7bWriteSecondOperand(2);

	F7BEND();
}

static uint32_t opSCHBS(uint32_t bSearch1)
{
	uint32_t i, data;
	uint32_t offset;

	F7bDecodeFirstOperand(BitReadAMAddress, 10);

	// Read first uint8_t
	m_op1 += m_bamoffset / 8;
	data = V60_RB(m_op1);
	offset = m_bamoffset & 7;

	// Scan bitstring
	for (i = 0; i < m_lenop1; i++)
	{
		// Update the work register
		R28 = m_op1;

		// There is a 0 / 1 at current offset?
		if ((bSearch1 && (data&(1 << offset))) ||
			(!bSearch1 && !(data&(1 << offset))))
			break;

		// Next bit please
		offset++;
		if (offset == 8)
		{
			// Next uint8_t please
			offset = 0;
			m_op1++;
			data = V60_RB(m_op1);
		}
	}

	// Set zero if bit not found
	_Z = (i == m_lenop1);

	// Write to destination the final offset
	m_modwritevalw = i;
	F7bWriteSecondOperand(2);

	F7BEND();
}

static uint32_t opSCH0BSU(void) { return opSCHBS(0); }
static uint32_t opSCH1BSU(void) { return opSCHBS(1); }

static uint32_t opINSBFR(void)
{
	uint32_t appw;
	F7cDecodeOperands(ReadAM, 2,BitReadAMAddress, 11);

	F7CCREATEBITMASK(m_lenop1);

	m_op2 += m_bamoffset / 8;
	appw = V60_RD(m_op2);
	m_bamoffset &= 7;

	appw &= ~(m_lenop1 << m_bamoffset);
	appw |=  (m_lenop1 & m_op1) << m_bamoffset;

	V60_WD(m_op2, appw);

	F7CEND();
}

static uint32_t opINSBFL(void)
{
	uint32_t appw;
	F7cDecodeOperands(ReadAM, 2,BitReadAMAddress, 11);

	m_op1 >>= (32 - m_lenop1);

	F7CCREATEBITMASK(m_lenop1);

	m_op2 += m_bamoffset / 8;
	appw = V60_RD(m_op2);
	m_bamoffset &= 7;

	appw &= ~(m_lenop1 << m_bamoffset);
	appw |=  (m_lenop1 & m_op1) << m_bamoffset;

	V60_WD(m_op2, appw);

	F7CEND();
}

static uint32_t opMOVBSD(void)
{
	uint32_t i;
	uint8_t srcdata, dstdata;

	F7bDecodeOperands(BitReadAMAddress, 10, BitReadAMAddress, 10);

//  if (m_lenop1 != 1)
//  { int a = 1; }

	m_bamoffset1 += m_lenop1 - 1;
	m_bamoffset2 += m_lenop1 - 1;

	m_op1 += m_bamoffset1 / 8;
	m_op2 += m_bamoffset2 / 8;

	m_bamoffset1 &= 7;
	m_bamoffset2 &= 7;

	srcdata = V60_RB(m_op1);
	dstdata = V60_RB(m_op2);

	for (i = 0; i < m_lenop1; i++)
	{
		// Update work registers
		R28 = m_op1;
		R27 = m_op2;

		dstdata &= ~(1 << m_bamoffset2);
		dstdata |= ((srcdata >> m_bamoffset1) & 1) << m_bamoffset2;

		if (m_bamoffset1 == 0)
		{
			m_bamoffset1 = 8;
			m_op1--;
			srcdata = V60_RB(m_op1);
		}
		if (m_bamoffset2 == 0)
		{
			V60_WB(m_op2, dstdata);
			m_bamoffset2 = 8;
			m_op2--;
			dstdata = V60_RB(m_op2);
		}

		m_bamoffset1--;
		m_bamoffset2--;
	}

	// Flush of the final data
	if (m_bamoffset2 != 7)
		V60_WB(m_op2, dstdata);

	F7BEND();
}

static uint32_t opMOVBSU(void)
{
	uint32_t i;
	uint8_t srcdata, dstdata;

	F7bDecodeOperands(BitReadAMAddress, 10, BitReadAMAddress, 10);

	m_op1 += m_bamoffset1 / 8;
	m_op2 += m_bamoffset2 / 8;

	m_bamoffset1 &= 7;
	m_bamoffset2 &= 7;

	srcdata = V60_RB(m_op1);
	dstdata = V60_RB(m_op2);

	for (i = 0; i < m_lenop1; i++)
	{
		// Update work registers
		R28 = m_op1;
		R27 = m_op2;

		dstdata &= ~(1 << m_bamoffset2);
		dstdata |= ((srcdata >> m_bamoffset1) & 1) << m_bamoffset2;

		m_bamoffset1++;
		m_bamoffset2++;
		if (m_bamoffset1 == 8)
		{
			m_bamoffset1 = 0;
			m_op1++;
			srcdata = V60_RB(m_op1);
		}
		if (m_bamoffset2 == 8)
		{
			V60_WB(m_op2, dstdata);
			m_bamoffset2 = 0;
			m_op2++;
			dstdata = V60_RB(m_op2);
		}
	}

	// Flush of the final data
	if (m_bamoffset2 != 0)
		V60_WB(m_op2, dstdata);

	F7BEND();
}

// RADM 0x20f4b8 holds the time left

static uint32_t opADDDC(void)
{
	uint8_t appb;
	uint8_t src, dst;

	F7cDecodeOperands(ReadAM, 0, ReadAMAddress, 0);

	if (m_lenop1 != 0)
	{
		v60_log("ADDDC %x (pat: %x)\n", m_op1, m_lenop1);
	}

	F7CLOADOP2BYTE(appb);

	src = (uint8_t)(m_op1 >> 4) * 10 + (uint8_t)(m_op1 & 0xF);
	dst = (appb >> 4) * 10 + (appb & 0xF);

	appb = src + dst + (_CY?1:0);

	if (appb >= 100)
	{
		appb -= 100;
		_CY = 1;
	}
	else
		_CY = 0;

	// compute z flag:
	// cleared if result non-zero or carry generated
	// unchanged otherwise
	if (appb != 0 || _CY)
		_Z = 0;

	appb = ((appb / 10) << 4) | (appb % 10);

	F7CSTOREOP2BYTE();
	F7CEND();
}

static uint32_t opSUBDC(void)
{
	int8_t appb;
	uint32_t src, dst;

	F7cDecodeOperands(ReadAM, 0, ReadAMAddress, 0);

	if (m_lenop1 != 0)
	{
		v60_log("SUBDC %x (pat: %x)\n", m_op1, m_lenop1);
	}

	F7CLOADOP2BYTE(appb);

	src = (uint32_t)(m_op1 >> 4) * 10 + (uint32_t)(m_op1 & 0xF);
	dst = ((appb & 0xF0) >> 4) * 10 + (appb & 0xF);

	// Note that this APPB must be SIGNED!
	appb = (int32_t)dst - (int32_t)src - (_CY?1:0);

	if (appb < 0)
	{
		appb += 100;
		_CY = 1;
	}
	else
		_CY = 0;

	// compute z flag:
	// cleared if result non-zero or carry generated
	// unchanged otherwise
	if (appb != 0 || _CY)
		_Z = 0;

	appb = ((appb / 10) << 4) | (appb % 10);

	F7CSTOREOP2BYTE();
	F7CEND();
}

static uint32_t opSUBRDC(void)
{
	int8_t appb;
	uint32_t src, dst;

	F7cDecodeOperands(ReadAM, 0, ReadAMAddress, 0);

	if (m_lenop1 != 0)
	{
		v60_log("SUBRDC %x (pat: %x)\n", m_op1, m_lenop1);
	}

	F7CLOADOP2BYTE(appb);

	src = (uint32_t)(m_op1 >> 4) * 10 + (uint32_t)(m_op1 & 0xF);
	dst = ((appb & 0xF0) >> 4) * 10 + (appb & 0xF);

	// Note that this APPB must be SIGNED!
	appb = (int32_t)src - (int32_t)dst - (_CY?1:0);

	if (appb < 0)
	{
		appb += 100;
		_CY = 1;
	}
	else
		_CY = 0;

	// compute z flag:
	// cleared if result non-zero or carry generated
	// unchanged otherwise
	if (appb != 0 || _CY)
		_Z = 0;

	appb = ((appb / 10) << 4) | (appb % 10);

	F7CSTOREOP2BYTE();
	F7CEND();
}

static uint32_t opCVTDPZ(void)
{
	uint16_t apph;

	F7cDecodeOperands(ReadAM, 0, ReadAMAddress, 1);

	apph = (uint16_t)(((m_op1 >> 4) & 0xF) | ((m_op1 & 0xF) << 8));
	apph |= (m_lenop1);
	apph |= (m_lenop1 << 8);

	// Z flag is unchanged if src is zero, cleared otherwise
	if (m_op1 != 0) _Z = 0;

	F7CSTOREOP2HALF();
	F7CEND();
}

static uint32_t opCVTDZP(void)
{
	uint8_t appb;
	F7cDecodeOperands(ReadAM, 1, ReadAMAddress, 0);

	if ((m_op1 & 0xF0) != (m_lenop1 & 0xF0) || ((m_op1 >> 8) & 0xF0) != (m_lenop1 & 0xF0))
	{
		// Decimal exception
		v60_log("CVTD.ZP Decimal exception #1!\n");
	}

	if ((m_op1 & 0xF) > 9 || ((m_op1 >> 8) & 0xF) > 9)
	{
		// Decimal exception
		v60_log("CVTD.ZP Decimal exception #2!\n");
	}

	appb = (uint8_t)(((m_op1 >> 8) & 0xF) | ((m_op1 & 0xF) << 4));
	if (appb != 0) _Z = 0;

	F7CSTOREOP2BYTE();
	F7CEND();
}

static uint32_t op58UNHANDLED(void)
{
	return v60_fatal("Unhandled 58 opcode at PC: /%06x\n", PC);
}

static uint32_t op5AUNHANDLED(void)
{
	return v60_fatal("Unhandled 5A opcode at PC: /%06x\n", PC);
}

static uint32_t op5BUNHANDLED(void)
{
	return v60_fatal("Unhandled 5B opcode at PC: /%06x\n", PC);
}

static uint32_t op5DUNHANDLED(void)
{
	return v60_fatal("Unhandled 5D opcode at PC: /%06x\n", PC);
}

static uint32_t op59UNHANDLED(void)
{
	return v60_fatal("Unhandled 59 opcode at PC: /%06x\n", PC);
}

static const am_func s_Op59Table[32] =
{
	opADDDC,
	opSUBDC,
	opSUBRDC,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	opCVTDPZ,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	opCVTDZP,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED,
	op59UNHANDLED
};


static const am_func s_Op5BTable[32] =
{
	opSCH0BSU,
	op5BUNHANDLED,
	opSCH1BSU,
		op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	opMOVBSU,
	opMOVBSD,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED,
	op5BUNHANDLED
};


static const am_func s_Op5DTable[32] =
{
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	opEXTBFS,
	opEXTBFZ,
	opEXTBFL,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	opINSBFR,
	opINSBFL,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED,
	op5DUNHANDLED
};

static const am_func s_Op58Table[32] =
{
	opCMPCB,
	opCMPCFB,
	opCMPCSB,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	opMOVCUB,
	opMOVCDB,
	opMOVCFUB,
	opMOVCFDB,
	opMOVCSUB,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	opSCHCUB,
	opSCHCDB,
	opSKPCUB,
	opSKPCDB,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED,
	op58UNHANDLED
};

static const am_func s_Op5ATable[32] =
{
	opCMPCH,
	opCMPCFH,
	opCMPCSH,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	opMOVCUH,
	opMOVCDH,
	opMOVCFUH,
	opMOVCFDH,
	opMOVCSUH,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	opSCHCUH,
	opSCHCDH,
	opSKPCUH,
	opSKPCDH,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED,
	op5AUNHANDLED
};

static uint32_t op58(void)
{
	m_subop = OpRead8(PC + 1);

	return (s_Op58Table[m_subop & 0x1F])();
}

static uint32_t op5A(void)
{
	m_subop = OpRead8(PC + 1);

	return (s_Op5ATable[m_subop & 0x1F])();
}

static uint32_t op5B(void)
{
	m_subop = OpRead8(PC + 1);

	return (s_Op5BTable[m_subop & 0x1F])();
}

static uint32_t op5D(void)
{
	m_subop = OpRead8(PC + 1);

	return (s_Op5DTable[m_subop & 0x1F])();
}

static uint32_t op59(void)
{
	m_subop = OpRead8(PC + 1);

	return (s_Op59Table[m_subop & 0x1F])();
}

static uint32_t opUNHANDLED(void)
{
	return v60_fatal("Unhandled OpCode found : %02x at %08x\n", OpRead16(PC), PC);
}

// ---- optable.hxx ----
// license:BSD-3-Clause
// copyright-holders:Farfetch'd, R. Belmont
static const am_func s_OpCodeTable[256] =
{
	/* 0x00 */ opHALT,
	/* 0x01 */ opLDTASK,
	/* 0x02 */ opSTPR,
	/* 0x03 */ opUNHANDLED,
	/* 0x04 */ opUNHANDLED,
	/* 0x05 */ opUNHANDLED,
	/* 0x06 */ opUNHANDLED,
	/* 0x07 */ opUNHANDLED,
	/* 0x08 */ opRVBIT,
	/* 0x09 */ opMOVB,
	/* 0x0a */ opMOVSBH,
	/* 0x0b */ opMOVZBH,
	/* 0x0c */ opMOVSBW,
	/* 0x0d */ opMOVZBW,
	/* 0x0e */ opUNHANDLED,
	/* 0x0f */ opUNHANDLED,
	/* 0x10 */ opCLRTLBA,
	/* 0x11 */ opUNHANDLED,
	/* 0x12 */ opLDPR,
	/* 0x13 */ opUPDPSWW,
	/* 0x14 */ opUNHANDLED,
	/* 0x15 */ opUNHANDLED,
	/* 0x16 */ opUNHANDLED,
	/* 0x17 */ opUNHANDLED,
	/* 0x18 */ opUNHANDLED,
	/* 0x19 */ opMOVTHB,
	/* 0x1a */ opUNHANDLED,
	/* 0x1b */ opMOVH,
	/* 0x1c */ opMOVSHW,
	/* 0x1d */ opMOVZHW,
	/* 0x1e */ opUNHANDLED,
	/* 0x1f */ opUNHANDLED,
	/* 0x20 */ opINB,
	/* 0x21 */ opOUTB,
	/* 0x22 */ opINH,
	/* 0x23 */ opOUTH,
	/* 0x24 */ opINW,
	/* 0x25 */ opOUTW,
	/* 0x26 */ opUNHANDLED,
	/* 0x27 */ opUNHANDLED,
	/* 0x28 */ opUNHANDLED,
	/* 0x29 */ opMOVTWB,
	/* 0x2a */ opUNHANDLED,
	/* 0x2b */ opMOVTWH,
	/* 0x2c */ opRVBYT,
	/* 0x2d */ opMOVW,
	/* 0x2e */ opUNHANDLED,
	/* 0x2f */ opUNHANDLED,
	/* 0x30 */ opUNHANDLED,
	/* 0x31 */ opUNHANDLED,
	/* 0x32 */ opUNHANDLED,
	/* 0x33 */ opUNHANDLED,
	/* 0x34 */ opUNHANDLED,
	/* 0x35 */ opUNHANDLED,
	/* 0x36 */ opUNHANDLED,
	/* 0x37 */ opUNHANDLED,
	/* 0x38 */ opNOTB,
	/* 0x39 */ opNEGB,
	/* 0x3a */ opNOTH,
	/* 0x3b */ opNEGH,
	/* 0x3c */ opNOTW,
	/* 0x3d */ opNEGW,
	/* 0x3e */ opUNHANDLED,
	/* 0x3f */ opMOVD,
	/* 0x40 */ opMOVEAB,
	/* 0x41 */ opXCHB,
	/* 0x42 */ opMOVEAH,
	/* 0x43 */ opXCHH,
	/* 0x44 */ opMOVEAW,
	/* 0x45 */ opXCHW,
	/* 0x46 */ opUNHANDLED,
	/* 0x47 */ opSETF,
	/* 0x48 */ opBSR,
	/* 0x49 */ opCALL,
	/* 0x4a */ opUPDPSWH,
	/* 0x4b */ opCHLVL,
	/* 0x4c */ opUNHANDLED,
	/* 0x4d */ opCHKAR,
	/* 0x4e */ opCHKAW,
	/* 0x4f */ opCHKAE,
	/* 0x50 */ opREMB,
	/* 0x51 */ opREMUB,
	/* 0x52 */ opREMH,
	/* 0x53 */ opREMUH,
	/* 0x54 */ opREMW,
	/* 0x55 */ opREMUW,
	/* 0x56 */ opUNHANDLED,
	/* 0x57 */ opUNHANDLED,
	/* 0x58 */ op58,
	/* 0x59 */ op59,
	/* 0x5a */ op5A,
	/* 0x5b */ op5B,
	/* 0x5c */ op5C,
	/* 0x5d */ op5D,
	/* 0x5e */ opUNHANDLED,
	/* 0x5f */ op5F,
	/* 0x60 */ opBV8,
	/* 0x61 */ opBNV8,
	/* 0x62 */ opBL8,
	/* 0x63 */ opBNL8,
	/* 0x64 */ opBE8,
	/* 0x65 */ opBNE8,
	/* 0x66 */ opBNH8,
	/* 0x67 */ opBH8,
	/* 0x68 */ opBN8,
	/* 0x69 */ opBP8,
	/* 0x6a */ opBR8,
	/* 0x6b */ opUNHANDLED,
	/* 0x6C */ opBLT8,
	/* 0x6c */ opBGE8,
	/* 0x6e */ opBLE8,
	/* 0x6f */ opBGT8,
	/* 0x70 */ opBV16,
	/* 0x71 */ opBNV16,
	/* 0x72 */ opBL16,
	/* 0x73 */ opBNL16,
	/* 0x74 */ opBE16,
	/* 0x75 */ opBNE16,
	/* 0x76 */ opBNH16,
	/* 0x77 */ opBH16,
	/* 0x78 */ opBN16,
	/* 0x79 */ opBP16,
	/* 0x7a */ opBR16,
	/* 0x7b */ opUNHANDLED,
	/* 0x7c */ opBLT16,
	/* 0x7d */ opBGE16,
	/* 0x7e */ opBLE16,
	/* 0x7f */ opBGT16,
	/* 0x80 */ opADDB,
	/* 0x81 */ opMULB,
	/* 0x82 */ opADDH,
	/* 0x83 */ opMULH,
	/* 0x84 */ opADDW,
	/* 0x85 */ opMULW,
	/* 0x86 */ opMULX,
	/* 0x87 */ opTEST1,
	/* 0x88 */ opORB,
	/* 0x89 */ opROTB,
	/* 0x8a */ opORH,
	/* 0x8b */ opROTH,
	/* 0x8c */ opORW,
	/* 0x8d */ opROTW,
	/* 0x8e */ opUNHANDLED,
	/* 0x8f */ opUNHANDLED,
	/* 0x90 */ opADDCB,
	/* 0x91 */ opMULUB,
	/* 0x92 */ opADDCH,
	/* 0x93 */ opMULUH,
	/* 0x94 */ opADDCW,
	/* 0x95 */ opMULUW,
	/* 0x96 */ opMULUX,
	/* 0x97 */ opSET1,
	/* 0x98 */ opSUBCB,
	/* 0x99 */ opROTCB,
	/* 0x9a */ opSUBCH,
	/* 0x9b */ opROTCH,
	/* 0x9c */ opSUBCW,
	/* 0x9d */ opROTCW,
	/* 0x9e */ opUNHANDLED,
	/* 0x9f */ opUNHANDLED,
	/* 0xa0 */ opANDB,
	/* 0xa1 */ opDIVB,
	/* 0xa2 */ opANDH,
	/* 0xa3 */ opDIVH,
	/* 0xa4 */ opANDW,
	/* 0xa5 */ opDIVW,
	/* 0xa6 */ opDIVX,
	/* 0xa7 */ opCLR1,
	/* 0xa8 */ opSUBB,
	/* 0xa9 */ opSHLB,
	/* 0xaa */ opSUBH,
	/* 0xab */ opSHLH,
	/* 0xac */ opSUBW,
	/* 0xad */ opSHLW,
	/* 0xae */ opUNHANDLED,
	/* 0xaf */ opUNHANDLED,
	/* 0xb0 */ opXORB,
	/* 0xb1 */ opDIVUB,
	/* 0xb2 */ opXORH,
	/* 0xb3 */ opDIVUH,
	/* 0xb4 */ opXORW,
	/* 0xb5 */ opDIVUW,
	/* 0xb6 */ opDIVUX,
	/* 0xb7 */ opNOT1,
	/* 0xb8 */ opCMPB,
	/* 0xb9 */ opSHAB,
	/* 0xba */ opCMPH,
	/* 0xbb */ opSHAH,
	/* 0xbc */ opCMPW,
	/* 0xbd */ opSHAW,
	/* 0xbe */ opUNHANDLED,
	/* 0xbf */ opUNHANDLED,
	/* 0xc0 */ opUNHANDLED,
	/* 0xc1 */ opUNHANDLED,
	/* 0xc2 */ opUNHANDLED,
	/* 0xc3 */ opUNHANDLED,
	/* 0xc4 */ opUNHANDLED,
	/* 0xc5 */ opUNHANDLED,
	/* 0xc6 */ opC6,
	/* 0xc7 */ opC7,
	/* 0xc8 */ opBRK,
	/* 0xc9 */ opBRKV,
	/* 0xca */ opRSR,
	/* 0xcb */ opTRAPFL,
	/* 0xcc */ opDISPOSE,
	/* 0xcd */ opNOP,
	/* 0xce */ opUNHANDLED,
	/* 0xcf */ opUNHANDLED,
	/* 0xd0 */ opDECB_0,
	/* 0xd1 */ opDECB_1,
	/* 0xd2 */ opDECH_0,
	/* 0xd3 */ opDECH_1,
	/* 0xd4 */ opDECW_0,
	/* 0xd5 */ opDECW_1,
	/* 0xd6 */ opJMP_0,
	/* 0xd7 */ opJMP_1,
	/* 0xd8 */ opINCB_0,
	/* 0xd9 */ opINCB_1,
	/* 0xda */ opINCH_0,
	/* 0xdb */ opINCH_1,
	/* 0xdc */ opINCW_0,
	/* 0xdd */ opINCW_1,
	/* 0xde */ opPREPARE_0,
	/* 0xdf */ opPREPARE_1,
	/* 0xe0 */ opTASI_0,
	/* 0xe1 */ opTASI_1,
	/* 0xe2 */ opRET_0,
	/* 0xe3 */ opRET_1,
	/* 0xe4 */ opPOPM_0,
	/* 0xe5 */ opPOPM_1,
	/* 0xe6 */ opPOP_0,
	/* 0xe7 */ opPOP_1,
	/* 0xe8 */ opJSR_0,
	/* 0xe9 */ opJSR_1,
	/* 0xea */ opRETIU_0,
	/* 0xeb */ opRETIU_1,
	/* 0xec */ opPUSHM_0,
	/* 0xed */ opPUSHM_1,
	/* 0xee */ opPUSH_0,
	/* 0xef */ opPUSH_1,
	/* 0xf0 */ opTESTB_0,
	/* 0xf1 */ opTESTB_1,
	/* 0xf2 */ opTESTH_0,
	/* 0xf3 */ opTESTH_1,
	/* 0xf4 */ opTESTW_0,
	/* 0xf5 */ opTESTW_1,
	/* 0xf6 */ opGETPSW_0,
	/* 0xf7 */ opGETPSW_1,
	/* 0xf8 */ opTRAP_0,
	/* 0xf9 */ opTRAP_1,
	/* 0xfa */ opRETIS_0,
	/* 0xfb */ opRETIS_1,
	/* 0xfc */ opSTTASK_0,
	/* 0xfd */ opSTTASK_1,
	/* 0xfe */ opCLRTLB_0,
	/* 0xff */ opCLRTLB_1,
};
