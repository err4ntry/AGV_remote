
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef MASTER_NODE_AUX_H
#define MASTER_NODE_AUX_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 master_node_aux_valueRangeTest (UNS8 typeValue, void * value);
const indextable * master_node_aux_scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode);

/* Master node data struct */
extern CO_Data master_node_aux_Data;
extern UNS16 cDeviceModle_aux;		/* Mapped at index 0x2000, subindex 0x00*/
extern UNS16 cHardwareVersion_aux;		/* Mapped at index 0x2001, subindex 0x00*/
extern UNS16 cFirmwareVersion_aux;		/* Mapped at index 0x2002, subindex 0x00*/
extern UNS16 cNode_ID_aux;		/* Mapped at index 0x2003, subindex 0x00*/
extern UNS16 cMOS_ProtectTemp_aux;		/* Mapped at index 0x2004, subindex 0x00*/
extern UNS16 cMotorProtectTemp_aux;		/* Mapped at index 0x2005, subindex 0x00*/
extern UNS16 cUnderVoltage_aux;		/* Mapped at index 0x2006, subindex 0x00*/
extern UNS16 cOverVoltage_aux;		/* Mapped at index 0x2007, subindex 0x00*/
extern UNS16 cDissipationOn_aux;		/* Mapped at index 0x2008, subindex 0x00*/
extern UNS16 cDissipationOff_aux;		/* Mapped at index 0x2009, subindex 0x00*/
extern UNS16 cOverCurrent_aux;		/* Mapped at index 0x200A, subindex 0x00*/
extern UNS16 cReductionRatio_aux;		/* Mapped at index 0x200B, subindex 0x00*/
extern UNS16 cMaxSpeed_aux;		/* Mapped at index 0x200C, subindex 0x00*/
extern UNS16 cMaxPosition_aux;		/* Mapped at index 0x200D, subindex 0x00*/
extern UNS16 cMaxCurrent_aux;		/* Mapped at index 0x200E, subindex 0x00*/
extern UNS16 cIdLoopKp_aux;		/* Mapped at index 0x200F, subindex 0x00*/
extern UNS16 cIdLoopKi_aux;		/* Mapped at index 0x2010, subindex 0x00*/
extern UNS16 cIdLoopKd_aux;		/* Mapped at index 0x2011, subindex 0x00*/
extern UNS16 cIdLoopKaw_aux;		/* Mapped at index 0x2012, subindex 0x00*/
extern UNS16 cIdLoopLimit_aux;		/* Mapped at index 0x2013, subindex 0x00*/
extern UNS16 cIqLoopKp_aux;		/* Mapped at index 0x2014, subindex 0x00*/
extern UNS16 cIqLoopKi_aux;		/* Mapped at index 0x2015, subindex 0x00*/
extern UNS16 cIqLoopKd_aux;		/* Mapped at index 0x2016, subindex 0x00*/
extern UNS16 cIqLoopKaw_aux;		/* Mapped at index 0x2017, subindex 0x00*/
extern UNS16 cIqLoopLimit_aux;		/* Mapped at index 0x2018, subindex 0x00*/
extern UNS16 cSpeedLoopKp_aux;		/* Mapped at index 0x2019, subindex 0x00*/
extern UNS16 cSpeedLoopKi_aux;		/* Mapped at index 0x201A, subindex 0x00*/
extern UNS16 cSpeedLoopKd_aux;		/* Mapped at index 0x201B, subindex 0x00*/
extern UNS16 cSpeedLoopKaw_aux;		/* Mapped at index 0x201C, subindex 0x00*/
extern UNS16 cSpeedLoopLimit_aux;		/* Mapped at index 0x201D, subindex 0x00*/
extern UNS16 cPositionLoopKp_aux;		/* Mapped at index 0x201E, subindex 0x00*/
extern UNS16 cPositionLoopKi_aux;		/* Mapped at index 0x201F, subindex 0x00*/
extern UNS16 cPositionLoopKd_aux;		/* Mapped at index 0x2020, subindex 0x00*/
extern UNS16 cPositionLoopKaw_aux;		/* Mapped at index 0x2021, subindex 0x00*/
extern UNS16 cPositionLoopLimit_aux;		/* Mapped at index 0x2022, subindex 0x00*/
extern UNS16 cErrorHistory_aux[10];		/* Mapped at index 0x2023, subindex 0x01 - 0x0A */
extern UNS16 cActualPosition_aux;		/* Mapped at index 0x2024, subindex 0x00*/
extern UNS16 cActualSpeed_aux;		/* Mapped at index 0x2025, subindex 0x00*/
extern UNS16 cActualCurrent_aux;		/* Mapped at index 0x2026, subindex 0x00*/
extern UNS16 cActualMotorTemp_aux;		/* Mapped at index 0x2027, subindex 0x00*/
extern UNS16 cActualMosTemp_aux;		/* Mapped at index 0x2028, subindex 0x00*/
extern UNS16 cActualBusVoltage_aux;		/* Mapped at index 0x2029, subindex 0x00*/
extern UNS16 cMotorStatus_aux;		/* Mapped at index 0x202A, subindex 0x00*/
extern UNS16 cMotorOperation_aux;		/* Mapped at index 0x202B, subindex 0x00*/
extern INTEGER32 cReferencePosition_aux;		/* Mapped at index 0x202C, subindex 0x00*/
extern UNS16 cReferenceSpeed_aux;		/* Mapped at index 0x202D, subindex 0x00*/
extern UNS16 cReferenceCurrent_aux;		/* Mapped at index 0x202E, subindex 0x00*/

#endif // MASTER_NODE_AUX_H
