#ifndef LIBLTE_X2AP_H
#define LIBLTE_X2AP_H


/*******************************************************************************
/* INCLUDES
********************************************************************************/

#include "liblte_common.h"

/*******************************************************************************
                              LOGGING
*******************************************************************************/

typedef void (*log_handler_t)(void *ctx, char *str);

void liblte_x2ap_log_register_handler(void *ctx, log_handler_t handler);


/*******************************************************************************
/* MAX defines
********************************************************************************/
#define LIBLTE_X2AP_MAXPRIVATEIES                                      65535
#define LIBLTE_X2AP_MAXPROTOCOLEXTENSIONS                              65535
#define LIBLTE_X2AP_MAXPROTOCOLIES                                     65535
#define LIBLTE_X2AP_MAXEARFCN                                          65535
#define LIBLTE_X2AP_EARFCNPLUSONE                                      65536
#define LIBLTE_X2AP_NEWMAXEARFCN                                       262143
#define LIBLTE_X2AP_MAXINTERFACES                                      16
#define LIBLTE_X2AP_MAXCELLINENB                                       256
#define LIBLTE_X2AP_MAXNOOFBANDS                                       16
#define LIBLTE_X2AP_MAXNOOFBEARERS                                     256
#define LIBLTE_X2AP_MAXNROFERRORS                                      256
#define LIBLTE_X2AP_MAXNOOFPDCP_SN                                     16
#define LIBLTE_X2AP_MAXNOOFEPLMNS                                      15
#define LIBLTE_X2AP_MAXNOOFEPLMNSPLUSONE                               16
#define LIBLTE_X2AP_MAXNOOFFORBLACS                                    4096
#define LIBLTE_X2AP_MAXNOOFFORBTACS                                    4096
#define LIBLTE_X2AP_MAXNOOFBPLMNS                                      6
#define LIBLTE_X2AP_MAXNOOFNEIGHBOURS                                  512
#define LIBLTE_X2AP_MAXNOOFPRBS                                        110
#define LIBLTE_X2AP_MAXPOOLS                                           16
#define LIBLTE_X2AP_MAXNOOFCELLS                                       16
#define LIBLTE_X2AP_MAXNOOFMBSFN                                       8
#define LIBLTE_X2AP_MAXFAILEDMEASOBJECTS                               32
#define LIBLTE_X2AP_MAXNOOFCELLIDFORMDT                                32
#define LIBLTE_X2AP_MAXNOOFTAFORMDT                                    8
#define LIBLTE_X2AP_MAXNOOFMBMSSERVICEAREAIDENTITIES                   256
#define LIBLTE_X2AP_MAXNOOFMDTPLMNS                                    16
/*******************************************************************************
/* Elementary Procedures
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_PROC_ID_HANDOVERPREPARATION                       = 0,
  LIBLTE_X2AP_PROC_ID_HANDOVERCANCEL                            = 1,
  LIBLTE_X2AP_PROC_ID_LOADINDICATION                            = 2,
  LIBLTE_X2AP_PROC_ID_ERRORINDICATION                           = 3,
  LIBLTE_X2AP_PROC_ID_SNSTATUSTRANSFER                          = 4,
  LIBLTE_X2AP_PROC_ID_UECONTEXTRELEASE                          = 5,
  LIBLTE_X2AP_PROC_ID_X2SETUP                                   = 6,
  LIBLTE_X2AP_PROC_ID_RESET                                     = 7,
  LIBLTE_X2AP_PROC_ID_ENBCONFIGURATIONUPDATE                    = 8,
  LIBLTE_X2AP_PROC_ID_RESOURCESTATUSREPORTINGINITIATION         = 9,
  LIBLTE_X2AP_PROC_ID_RESOURCESTATUSREPORTING                   = 10,
  LIBLTE_X2AP_PROC_ID_PRIVATEMESSAGE                            = 11,
  LIBLTE_X2AP_PROC_ID_MOBILITYSETTINGSCHANGE                    = 12,
  LIBLTE_X2AP_PROC_ID_RLFINDICATION                             = 13,
  LIBLTE_X2AP_PROC_ID_HANDOVERREPORT                            = 14,
  LIBLTE_X2AP_PROC_ID_CELLACTIVATION                            = 15,
  LIBLTE_X2AP_PROC_N_ITEMS,
}LIBLTE_X2AP_PROC_ENUM;

static const char liblte_x2ap_proc_text[LIBLTE_X2AP_PROC_N_ITEMS][64] = { 
  "id-handoverPreparation",
  "id-handoverCancel",
  "id-loadIndication",
  "id-errorIndication",
  "id-snStatusTransfer",
  "id-uEContextRelease",
  "id-x2Setup",
  "id-reset",
  "id-eNBConfigurationUpdate",
  "id-resourceStatusReportingInitiation",
  "id-resourceStatusReporting",
  "id-privateMessage",
  "id-mobilitySettingsChange",
  "id-rLFIndication",
  "id-handoverReport",
  "id-cellActivation",
};

/*******************************************************************************
/* ProtocolIE Ids
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_IE_ID_E_RABS_ADMITTED_ITEM                          = 0,
  LIBLTE_X2AP_IE_ID_E_RABS_ADMITTED_LIST                          = 1,
  LIBLTE_X2AP_IE_ID_E_RAB_ITEM                                    = 2,
  LIBLTE_X2AP_IE_ID_E_RABS_NOTADMITTED_LIST                       = 3,
  LIBLTE_X2AP_IE_ID_E_RABS_TOBESETUP_ITEM                         = 4,
  LIBLTE_X2AP_IE_ID_CAUSE                                         = 5,
  LIBLTE_X2AP_IE_ID_CELLINFORMATION                               = 6,
  LIBLTE_X2AP_IE_ID_CELLINFORMATION_ITEM                          = 7, 
  LIBLTE_X2AP_IE_ID_NEW_ENB_UE_X2AP_ID                            = 9,
  LIBLTE_X2AP_IE_ID_OLD_ENB_UE_X2AP_ID                            = 10,
  LIBLTE_X2AP_IE_ID_TARGETCELL_ID                                 = 11,
  LIBLTE_X2AP_IE_ID_TARGETENBTOSOURCE_ENBTRANSPARENTCONTAINER     = 12,
  LIBLTE_X2AP_IE_ID_TRACEACTIVATION                               = 13,
  LIBLTE_X2AP_IE_ID_UE_CONTEXTINFORMATION                         = 14,
  LIBLTE_X2AP_IE_ID_UE_HISTORYINFORMATION                         = 15,
  LIBLTE_X2AP_IE_ID_UE_X2AP_ID                                    = 16,
  LIBLTE_X2AP_IE_ID_CRITICALITYDIAGNOSTICS                        = 17,
  LIBLTE_X2AP_IE_ID_E_RABS_SUBJECTTOSTATUSTRANSFER_LIST           = 18,
  LIBLTE_X2AP_IE_ID_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM           = 19,
  LIBLTE_X2AP_IE_ID_SERVEDCELLS                                   = 20,
  LIBLTE_X2AP_IE_ID_GLOBALENB_ID                                  = 21,
  LIBLTE_X2AP_IE_ID_TIMETOWAIT                                    = 22,
  LIBLTE_X2AP_IE_ID_GUMMEI_ID                                     = 23,
  LIBLTE_X2AP_IE_ID_GUGROUPIDLIST                                 = 24,
  LIBLTE_X2AP_IE_ID_SERVEDCELLSTOADD                              = 25,
  LIBLTE_X2AP_IE_ID_SERVEDCELLSTOMODIFY                           = 26,
  LIBLTE_X2AP_IE_ID_SERVEDCELLSTODELETE                           = 27,
  LIBLTE_X2AP_IE_ID_REGISTRATION_REQUEST                          = 28,
  LIBLTE_X2AP_IE_ID_CELLTOREPORT                                  = 29,
  LIBLTE_X2AP_IE_ID_REPORTINGPERIODICITY                          = 30,
  LIBLTE_X2AP_IE_ID_CELLTOREPORT_ITEM                             = 31,
  LIBLTE_X2AP_IE_ID_CELLMEASUREMENTRESULT                         = 32,
  LIBLTE_X2AP_IE_ID_CELLMEASUREMENTRESULT_ITEM                    = 33,
  LIBLTE_X2AP_IE_GUGROUPIDTOADDLIST                               = 34,
  LIBLTE_X2AP_IE_ID_GUGROUPIDTODELETELIST                         = 35,
  LIBLTE_X2AP_IE_ID_SRVCCOPERATIONPOSSIBLE                        = 36,
  LIBLTE_X2AP_IE_ID_MEASUREMENT_ID                                = 37,
  LIBLTE_X2AP_IE_ID_REPORTCHARATERISTICS                          = 38,
  LIBLTE_X2AP_IE_ID_ENB1_MEASUREMENT_ID                           = 39,
  LIBLTE_X2AP_IE_ID_ENB2_MEASUREMNET_ID                           = 40,
  LIBLTE_X2AP_IE_ID_NUMBER_OF_ANTENNAPORTS                        = 41,
  LIBLTE_X2AP_IE_ID_COMPOSITEAVAILABLECAPACITYGROUP               = 42,
  LIBLTE_X2AP_IE_ID_ENB1_CELL_ID                                  = 43,
  LIBLTE_X2AP_IE_ID_ENB2_CELL_ID                                  = 44,
  LIBLTE_X2AP_IE_ID_ENB2_PROPOSED_MOBILITY_PARAMETERS             = 45,
  LIBLTE_X2AP_IE_ID_ENB1_MOBILITY_PARAMETERS                      = 46,
  LIBLTE_X2AP_IE_ID_ENB2_MOBILITY_PARAMETERS_MODIFICATION_RANGE   = 47,
  LIBLTE_X2AP_IE_ID_FAILURECELLPCI                                = 48,
  LIBLTE_X2AP_IE_ID_RE_ESTABLISHMENTCELLECGI                      = 49,
  LIBLTE_X2AP_IE_FAILURECELLCRNTI                                 = 50,
  LIBLTE_X2AP_IE_ID_SHORTMAC_I                                    = 51,
  LIBLTE_X2AP_IE_ID_SOURCECELLECGI                                = 52,
  LIBLTE_X2AP_IE_ID_FAILURECELLECGI                               = 53,
  LIBLTE_X2AP_IE_ID_HANDOVERREPORTTYPE                            = 54,
  LIBLTE_X2AP_IE_ID_PRACH_CONFIGURATION                           = 55,
  LIBLTE_X2AP_IE_ID_MBSFN_SUBFRAME_INFO                           = 56,
  LIBLTE_X2AP_IE_ID_SERVEDCELLSTOACTIVATE                         = 57,
  LIBLTE_X2AP_IE_ID_ACTIVATEDCELLLIST                             = 58,
  LIBLTE_X2AP_IE_ID_DEACTIVATIONINDICATION                        = 59,
  LIBLTE_X2AP_IE_ID_UE_RLF_REPORT_CONTAINER                       = 60,
  LIBLTE_X2AP_IE_ID_ABSINFORMATION                                = 61,
  LIBLTE_X2AP_IE_ID_INVOKEINDICATION                              = 62,
  LIBLTE_X2AP_IE_ID_ABS_STATUS                                    = 63,
  LIBLTE_X2AP_IE_ID_PARTIALSUCCESSINDICATOR                       = 64,
  LIBLTE_X2AP_IE_ID_MEASUREMENTINITIATIONRESULT_LIST              = 65,
  LIBLTE_X2AP_IE_ID_MEASUREMENTINITIATIONRESULT_ITEM              = 66,
  LIBLTE_X2AP_IE_ID_MEASUREMENTFAILURECAUSE_ITEM                  = 67,
  LIBLTE_X2AP_IE_ID_COMPLETEFAILURECAUSEINFORMATION_LIST          = 68,
  LIBLTE_X2AP_IE_ID_COMPLETEFAILURECAUSEINFORMATION_ITEM          = 69,
  LIBLTE_X2AP_IE_ID_CSG_ID                                        = 70,
  LIBLTE_X2AP_IE_ID_CSGMEMBERSHIPSTATUS                           = 71,
  LIBLTE_X2AP_IE_ID_MDTCONFIGURATION                              = 72,
  LIBLTE_X2AP_IE_ID_MANAGEMENTBASEDMDTALLOWED                     = 74,
  LIBLTE_X2AP_IE_ID_RRCCONNSETUPINDICATOR                         = 75,
  LIBLTE_X2AP_IE_ID_NEIGHBOURTAC                                  = 76,
  LIBLTE_X2AP_IE_ID_TIME_UE_STAYEDINCELL_ENHANCEDGRANULARITY      = 77,
  LIBLTE_X2AP_IE_ID_RRCCONNREESTABINDICATOR                       = 78,
  LIBLTE_X2AP_IE_ID_MBMS_SERVICE_AREA_LIST                        = 79,
  LIBLTE_X2AP_IE_ID_HO_CAUSE                                      = 80,
  LIBLTE_X2AP_IE_ID_TARGETCELLINUTRAN                             = 81,
  LIBLTE_X2AP_IE_ID_MOBILITYINFORMATION                           = 82,
  LIBLTE_X2AP_IE_ID_SOURCECELLCRNTI                               = 83,
  LIBLTE_X2AP_IE_ID_MULTIBANDINFOLIST                             = 84,
  LIBLTE_X2AP_IE_ID_M3CONFIGURATION                               = 85,
  LIBLTE_X2AP_IE_ID_M4CONFIGURATION                               = 86,
  LIBLTE_X2AP_IE_ID_M5CONFIGURATION                               = 87,
  LIBLTE_X2AP_IE_ID_MDT_LOCATION_INFO                             = 88,
  LIBLTE_X2AP_IE_ID_MANAGEMENTBASEDMDTPLMNLIST                    = 89,
  LIBLTE_X2AP_IE_ID_SIGNALLINGBASEDMDTPLMNLIST                    = 90,
  LIBLTE_X2AP_IE_ID_RECEIVESTATUSOFULPDCPSDUSEXTENDED             = 91,
  LIBLTE_X2AP_IE_ID_ULCOUNTVALUEEXTENDED                          = 92,
  LIBLTE_X2AP_IE_ID_DLCOUNTVALUEEXTENDED                          = 93,
  LIBLTE_X2AP_IE_ID_EARFCNEXTENSION                               = 94,
  LIBLTE_X2AP_IE_ID_UL_EARFCNEXTENSION                            = 95,
  LIBLTE_X2AP_IE_ID_DL_EARFCNEXTENSION                            = 96,
  LIBLTE_X2AP_IE_ID_ADDITIONALSPECIALSUBFRAME_INFO                = 97,
  LIBLTE_X2AP_IE_N_ITEMS,
}LIBLTE_X2AP_IE_ENUM;

static const char liblte_x2ap_ie_text[LIBLTE_X2AP_IE_N_ITEMS][64] = {
  "id-E-RABs-Admitted-Item",
  "id-E-RABS-Admitted-List",
  "id-E-RAB-Item",
  "id-E-RABs-NotAdmitted-List",
  "id-E-RABs-ToBeSetup-Item",
  "id-Cause",
  "id-CellInformation",
  "id-CellInformation-Item",
  "", // To be verified
  "id-New-eNB-UE-X2AP-ID",
  "id-Old-eNB-UE-X2AP-ID",
  "id-TargetCell-ID",
  "id-TargeteNBtoSource-eNBTransparentContainer",
  "id-TraceActivation",
  "id-UE-ContextInformation",
  "id-UE-HistoryInformation",
  "id-UE-X2AP-ID",
  "id-CriticalityDiagnostics",
  "id-E-RABs-SubjectToStatusTransfer-List",
  "id-E-RABs-SubjectToStatusTransfer-Item",
  "id-ServedCells",
  "id-GlobalENB-ID",
  "id-TimeToWait",
  "id-GUMMEI-ID",
  "id-GUGroupIDList",
  "id-ServedCellsToAdd",
  "id-ServedCellsToModify",
  "id-ServedCellsToDelete",
  "id-Registration-Request",
  "id-CellToReport",
  "id-ReportingPeriodicity",
  "id-CellToReport-Item",
  "id-CellMeasurementResult",
  "id-CellMeasurementResult-Item",
  "id-GUGroupIDToAddList",
  "id-GUGroupIDToDeleteList",
  "id-SRVCCOperationPossible",
  "id-Measurement-ID",
  "id-ReportCharacteristics",
  "id-ENB1-Measurement-ID",
  "id-ENB2-Measurement-ID",
  "id-Number-of-Antennaports",
  "id-CompositeAvailableCapacityGroup",
  "id-ENB1-Cell-ID",
  "id-ENB2-Cell-ID",
  "id-ENB2-Proposed-Mobility-Parameters",
  "id-ENB1-Mobility-Parameters",
  "id-ENB2-Mobility-Parameters-Modification-Range",
  "id-FailureCellPCI",
  "id-Re-establishmentCellECGI",
  "id-FailureCellCRNTI",
  "id-ShortMAC-I",
  "id-SourceCellECGI",
  "id-FailureCellECGI",
  "id-HandoverReportType",
  "id-PRACH-Configuration",
  "id-MBSFN-Subframe-Info",
  "id-ServedCellsToActivate",
  "id-ActivatedCellList",
  "id-DeactivationIndication",
  "id-UE-RLF-Report-Container",
  "id-ABSInformation",
  "id-InvokeIndication",
  "id-ABS-Status",
  "id-PartialSuccessIndicator",
  "id-MeasurementInitiationResult-List",
  "id-MeasurementInitiationResult-Item",
  "id-MeasurementFailureCause-Item",
  "id-CompleteFailureCauseInformation-List",
  "id-CompleteFailureCauseInformation-Item",
  "id-CSG-Id",
  "id-CSGMembershipStatus",
  "id-MDTConfiguration",
  "", // To be verified
  "id-ManagementBasedMDTallowed",
  "id-RRCConnSetupIndicator",
  "id-NeighbourTAC",
  "id-Time-UE-StayedInCell-EnhancedGranularity",
  "id-RRCConnReestabIndicator",
  "id-MBMS-Service-Area-List",
  "id-HO-cause",
  "id-TargetCellInUTRAN",
  "id-MobilityInformation",
  "id-SourceCellCRNTI",
  "id-MultibandInfoList",
  "id-M3Configuration",
  "id-M4Configuration",
  "id-M5Configuration",
  "id-MDT-Location-Info",
  "id-ManagementBasedMDTPLMNList",
  "id-SignallingBasedMDTPLMNList",
  "id-ReceiveStatusOfULPDCPSDUsExtended",
  "id-ULCOUNTValueExtended",
  "id-DLCOUNTValueExtended",
  "id-eARFCNExtension",
  "id-UL-EARFCNExtension",
  "id-DL-EARFCNExtension",
  "id-AdditionalSpecialSubframe-Info",
};

/*******************************************************************************
/* ProtocolIE Criticality ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_CRITICALITY_REJECT,
  LIBLTE_X2AP_CRITICALITY_IGNORE,
  LIBLTE_X2AP_CRITICALITY_NOTIFY,
  LIBLTE_X2AP_CRITICALITY_N_ITEMS,
}LIBLTE_X2AP_CRITICALITY_ENUM;
static const char liblte_x2ap_criticality_text[LIBLTE_X2AP_CRITICALITY_N_ITEMS][80] = {
  "reject",
  "ignore",
  "notify",
};

LIBLTE_ERROR_ENUM liblte_x2ap_pack_criticality(
  LIBLTE_X2AP_CRITICALITY_ENUM                                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_criticality(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CRITICALITY_ENUM                                 *ie);

/*******************************************************************************
/* ProtocolIE local INTEGER
********************************************************************************/
typedef struct{
uint16_t local;
}LIBLTE_X2AP_LOCAL_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_local(
  LIBLTE_X2AP_LOCAL_STRUCT                                     *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_local(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_LOCAL_STRUCT                                     *ie);

/*******************************************************************************
/* ProtocolIE PrivateIE_ID CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_PRIVATEIE_ID_CHOICE_LOCAL,
  LIBLTE_X2AP_PRIVATEIE_ID_CHOICE_GLOBAL,
  LIBLTE_X2AP_PRIVATEIE_ID_CHOICE_N_ITEMS,
}LIBLTE_X2AP_PRIVATEIE_ID_CHOICE_ENUM;
static const char liblte_x2ap_privateie_id_choice_text[LIBLTE_X2AP_PRIVATEIE_ID_CHOICE_N_ITEMS][50] = {
  "local",
  "global",
};

typedef union{
  LIBLTE_X2AP_LOCAL_STRUCT                                     local;
  LIBLTE_ASN1_OID_STRUCT                                       global;
}LIBLTE_X2AP_PRIVATEIE_ID_CHOICE_UNION;

typedef struct{
  LIBLTE_X2AP_PRIVATEIE_ID_CHOICE_UNION choice;
  LIBLTE_X2AP_PRIVATEIE_ID_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_PRIVATEIE_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_privateie_id(
  LIBLTE_X2AP_PRIVATEIE_ID_STRUCT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_privateie_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PRIVATEIE_ID_STRUCT                              *ie);

/* Do not need this for X2AP -- WT
/*******************************************************************************
/* ProtocolIE ProtocolExtensionID INTEGER
********************************************************************************
typedef struct{
uint16_t ProtocolExtensionID;
}LIBLTE_X2AP_PROTOCOLEXTENSIONID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolextensionid(
  LIBLTE_X2AP_PROTOCOLEXTENSIONID_STRUCT                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolextensionid(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLEXTENSIONID_STRUCT                       *ie);
*/

/*******************************************************************************
/* ProtocolIE TriggeringMessage ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_TRIGGERINGMESSAGE_INITIATING_MESSAGE,
  LIBLTE_X2AP_TRIGGERINGMESSAGE_SUCCESSFUL_OUTCOME,
  LIBLTE_X2AP_TRIGGERINGMESSAGE_UNSUCCESSFULL_OUTCOME,
  LIBLTE_X2AP_TRIGGERINGMESSAGE_N_ITEMS,
}LIBLTE_X2AP_TRIGGERINGMESSAGE_ENUM;
static const char liblte_x2ap_triggeringmessage_text[LIBLTE_X2AP_TRIGGERINGMESSAGE_N_ITEMS][80] = {
  "initiating-message",
  "successful-outcome",
  "unsuccessfull-outcome",
};

LIBLTE_ERROR_ENUM liblte_x2ap_pack_triggeringmessage(
  LIBLTE_X2AP_TRIGGERINGMESSAGE_ENUM                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_triggeringmessage(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TRIGGERINGMESSAGE_ENUM                           *ie);

/*******************************************************************************
/* ProtocolIE Presence ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_PRESENCE_OPTIONAL,
  LIBLTE_X2AP_PRESENCE_CONDITIONAL,
  LIBLTE_X2AP_PRESENCE_MANDATORY,
  LIBLTE_X2AP_PRESENCE_N_ITEMS,
}LIBLTE_X2AP_PRESENCE_ENUM;
static const char liblte_x2ap_presence_text[LIBLTE_X2AP_PRESENCE_N_ITEMS][80] = {
  "optional",
  "conditional",
  "mandatory",
};

LIBLTE_ERROR_ENUM liblte_x2ap_pack_presence(
  LIBLTE_X2AP_PRESENCE_ENUM                                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_presence(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PRESENCE_ENUM                                    *ie);

/*******************************************************************************
/* ProtocolIE ProtocolIE_ID INTEGER
********************************************************************************/
typedef struct{
uint16_t ProtocolIE_ID;
}LIBLTE_X2AP_PROTOCOLIE_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolie_id(
  LIBLTE_X2AP_PROTOCOLIE_ID_STRUCT                             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolie_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLIE_ID_STRUCT                             *ie);

/*******************************************************************************
/* ProtocolIE ProcedureCode INTEGER
********************************************************************************/
typedef struct{
uint8_t ProcedureCode;
}LIBLTE_X2AP_PROCEDURECODE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_procedurecode(
  LIBLTE_X2AP_PROCEDURECODE_STRUCT                             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_procedurecode(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROCEDURECODE_STRUCT                             *ie);


/*******************************************************************************
/* ProtocolIE ProtocolIE_Field SEQUENCE
********************************************************************************/
typedef struct{
  LIBLTE_X2AP_PROTOCOLIE_ID_STRUCT                             id;
  LIBLTE_X2AP_CRITICALITY_ENUM                                 criticality;
  LIBLTE_ASN1_OPEN_TYPE_STRUCT                                 value;
}LIBLTE_X2AP_PROTOCOLIE_FIELD_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolie_field(
  LIBLTE_X2AP_PROTOCOLIE_FIELD_STRUCT                          *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolie_field(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLIE_FIELD_STRUCT                          *ie);

/*******************************************************************************
/* ProtocolIE ProtocolIE_SingleContainer SEQUENCE
********************************************************************************/
typedef struct{
  LIBLTE_X2AP_PROTOCOLIE_ID_STRUCT                             id;
  LIBLTE_X2AP_CRITICALITY_ENUM                                 criticality;
  LIBLTE_ASN1_OPEN_TYPE_STRUCT                                 value;
}LIBLTE_X2AP_PROTOCOLIE_SINGLECONTAINER_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolie_singlecontainer(
  LIBLTE_X2AP_PROTOCOLIE_SINGLECONTAINER_STRUCT                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolie_singlecontainer(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLIE_SINGLECONTAINER_STRUCT                *ie);

/*******************************************************************************
/* ProtocolIE ProtocolIE_Container DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:65535
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_PROTOCOLIE_FIELD_STRUCT                    buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_PROTOCOLIE_CONTAINER_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolie_container(
  LIBLTE_X2AP_PROTOCOLIE_CONTAINER_STRUCT                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolie_container(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLIE_CONTAINER_STRUCT                *ie);
  
/*******************************************************************************
/* ProtocolIE ProtocolExtensionField SEQUENCE
********************************************************************************/
typedef struct{
  LIBLTE_X2AP_PROTOCOLIE_ID_STRUCT                             id; // changed by WT
  LIBLTE_X2AP_CRITICALITY_ENUM                                 criticality;
  LIBLTE_ASN1_OPEN_TYPE_STRUCT                                 extensionValue;
}LIBLTE_X2AP_PROTOCOLEXTENSIONFIELD_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolextensionfield(
  LIBLTE_X2AP_PROTOCOLEXTENSIONFIELD_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolextensionfield(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLEXTENSIONFIELD_STRUCT                    *ie);

/*******************************************************************************
/* ProtocolIE ProtocolExtensionContainer DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:65535
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_PROTOCOLEXTENSIONFIELD_STRUCT                    buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolextensioncontainer(
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolextensioncontainer(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                *ie);

/*******************************************************************************
/* ProtocolIE ProtocolIE_FieldPair SEQUENCE
********************************************************************************/
typedef struct{
  LIBLTE_X2AP_PROTOCOLIE_ID_STRUCT                             id;
  LIBLTE_X2AP_CRITICALITY_ENUM                                 firstCriticality;
  LIBLTE_ASN1_OPEN_TYPE_STRUCT                                 firstValue;
  LIBLTE_X2AP_CRITICALITY_ENUM                                 secondCriticality;
  LIBLTE_ASN1_OPEN_TYPE_STRUCT                                 secondValue;
}LIBLTE_X2AP_PROTOCOLIE_FIELDPAIR_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolie_fieldpair(
  LIBLTE_X2AP_PROTOCOLIE_FIELDPAIR_STRUCT                      *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolie_fieldpair(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLIE_FIELDPAIR_STRUCT                      *ie);

/*******************************************************************************
/* ProtocolIE ProtocolIE_ContainerPair DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:0, ub:65535
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_PROTOCOLIE_FIELDPAIR_STRUCT                      buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_PROTOCOLIE_CONTAINERPAIR_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolie_containerpair(
  LIBLTE_X2AP_PROTOCOLIE_CONTAINERPAIR_STRUCT                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolie_containerpair(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLIE_CONTAINERPAIR_STRUCT                  *ie);

/*******************************************************************************
/* ProtocolIE ProtocolIE_ContainerList DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:None, ub:None
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_PROTOCOLIE_CONTAINER_STRUCT                  buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_PROTOCOLIE_CONTAINERLIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolie_containerlist(
  LIBLTE_X2AP_PROTOCOLIE_CONTAINERLIST_STRUCT              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolie_containerlist(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLIE_CONTAINERLIST_STRUCT              *ie);

/*******************************************************************************
/* ProtocolIE ProtocolIE_ContainerPairList DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:None, ub:None
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_PROTOCOLIE_CONTAINERPAIR_STRUCT                  buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_PROTOCOLIE_CONTAINERPAIRLIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolie_containerpairlist(
  LIBLTE_X2AP_PROTOCOLIE_CONTAINERPAIRLIST_STRUCT              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolie_containerpairlist(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PROTOCOLIE_CONTAINERPAIRLIST_STRUCT              *ie);

/*******************************************************************************
/* ProtocolIE PrivateIE_Field SEQUENCE
********************************************************************************/
typedef struct{
  LIBLTE_X2AP_PRIVATEIE_ID_STRUCT                              id;
  LIBLTE_X2AP_CRITICALITY_ENUM                                 criticality;
  LIBLTE_ASN1_OPEN_TYPE_STRUCT                                 value;
}LIBLTE_X2AP_PRIVATEIE_FIELD_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_privateie_field(
  LIBLTE_X2AP_PRIVATEIE_FIELD_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_privateie_field(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PRIVATEIE_FIELD_STRUCT                           *ie);

/*******************************************************************************
/* ProtocolIE PrivateIE_Container DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:65535
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_PRIVATEIE_FIELD_STRUCT                           buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_PRIVATEIE_CONTAINER_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_privateie_container(
  LIBLTE_X2AP_PRIVATEIE_CONTAINER_STRUCT                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_privateie_container(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PRIVATEIE_CONTAINER_STRUCT                       *ie);

/* X2AP-IEs.asn */

/* Integer */

/*******************************************************************************
/* ProtocolIE DL_ABS_status INTEGER
********************************************************************************/
typedef struct{
uint32_t DL_ABS_Status;
}LIBLTE_X2AP_DL_ABS_STATUS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_dl_abs_status(
  LIBLTE_X2AP_DL_ABS_STATUS_STRUCT                                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_dl_abs_status(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_DL_ABS_STATUS_STRUCT                                   *ie);

/*******************************************************************************
/* ProtocolIE BitRate INTEGER
********************************************************************************/
typedef struct{
uint64_t BitRate;
}LIBLTE_X2AP_BITRATE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_bitrate(
  LIBLTE_X2AP_BITRATE_STRUCT                                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_bitrate(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_BITRATE_STRUCT                                   *ie);

/*******************************************************************************
/* ProtocolIE CapacityValue INTEGER
********************************************************************************/
typedef struct{
uint32_t CapacityValue;
}LIBLTE_X2AP_CAPACITYVALUE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_capacityvalue(
  LIBLTE_X2AP_CAPACITYVALUE_STRUCT                                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_capacityvalue(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CAPACITYVALUE_STRUCT                                   *ie);

/*******************************************************************************
/* ProtocolIE CellCapacityClassValue INTEGER
********************************************************************************/
typedef struct{
uint32_t CellCapacityClassValue;
}LIBLTE_X2AP_CELLCAPACITYCLASSVALUE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellcapacityclassvalue(
  LIBLTE_X2AP_CELLCAPACITYCLASSVALUE_STRUCT                                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellcapacityclassvalue(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELLCAPACITYCLASSVALUE_STRUCT                                   *ie);

/*******************************************************************************
/* ProtocolIE DL_GBR_PRB_usage INTEGER
********************************************************************************/
typedef struct{
uint16_t DL_GBR_PRB_usage;
}LIBLTE_X2AP_DL_GBR_PRB_USAGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_dl_gbr_prb_usage(
  LIBLTE_X2AP_DL_GBR_PRB_USAGE_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_dl_gbr_prb_usage(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_DL_GBR_PRB_USAGE_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE DL_non_GBR_PRB_usage INTEGER
********************************************************************************/
typedef struct{
uint16_t DL_non_GBR_PRB_usage;
}LIBLTE_X2AP_DL_NON_GBR_PRB_USAGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_dl_non_gbr_prb_usage(
  LIBLTE_X2AP_DL_NON_GBR_PRB_USAGE_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_dl_non_gbr_prb_usage(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_DL_NON_GBR_PRB_USAGE_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE DL_Total_PRB_usage INTEGER
********************************************************************************/
typedef struct{
uint16_t DL_Total_PRB_usage;
}LIBLTE_X2AP_DL_TOTAL_PRB_USAGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_dl_total_prb_usage(
  LIBLTE_X2AP_DL_TOTAL_PRB_USAGE_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_dl_total_prb_usage(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_DL_TOTAL_PRB_USAGE_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE EARFCN INTEGER
********************************************************************************/
typedef struct{
uint16_t EARFCN;
}LIBLTE_X2AP_EARFCN_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_earfcn(
  LIBLTE_X2AP_EARFCN_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_earfcn(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_EARFCN_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE EARFCNExtension INTEGER
********************************************************************************/
typedef struct{
uint16_t EARFCNExtension;
}LIBLTE_X2AP_EARFCNEXTENSION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_earfcnextension(
  LIBLTE_X2AP_EARFCNEXTENSION_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_earfcnextension(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_EARFCNEXTENSION_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE E_RAB_ID INTEGER
********************************************************************************/
typedef struct{
  bool     ext;
uint8_t E_RAB_ID;
}LIBLTE_X2AP_E_RAB_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rab_id(
  LIBLTE_X2AP_E_RAB_ID_STRUCT                                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rab_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_E_RAB_ID_STRUCT                                  *ie);

/*******************************************************************************
/* ProtocolIE FreqBandIndicator INTEGER
********************************************************************************/
typedef struct{
uint32_t FreqBandIndicator;
}LIBLTE_X2AP_FREQBANDINDICATOR_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_freqbandindicator(
  LIBLTE_X2AP_FREQBANDINDICATOR_STRUCT                                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_freqbandindicator(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_FREQBANDINDICATOR_STRUCT                                       *ie);

/*******************************************************************************
/* ProtocolIE HFN INTEGER
********************************************************************************/
typedef struct{
uint32_t HFN;
}LIBLTE_X2AP_HFN_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_hfn(
  LIBLTE_X2AP_HFN_STRUCT                                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_hfn(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_HFN_STRUCT                                       *ie);

/*******************************************************************************
/* ProtocolIE HFNModified INTEGER
********************************************************************************/
typedef struct{
uint32_t HFNModified;
}LIBLTE_X2AP_HFNMODIFIED_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_hfnmodified(
  LIBLTE_X2AP_HFNMODIFIED_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_hfnmodified(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_HFNMODIFIED_STRUCT                               *ie);

/*******************************************************************************
/* ProtocolIE Measurement_ID INTEGER
********************************************************************************/
typedef struct{
uint32_t Measurement_ID;
}LIBLTE_X2AP_MEASUREMENT_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurement_id(
  LIBLTE_X2AP_MEASUREMENT_ID_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurement_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MEASUREMENT_ID_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE NextHopChainingCount INTEGER
********************************************************************************/
typedef struct{
uint32_t NextHopChainingCount;
}LIBLTE_X2AP_NEXTHOPCHAININGCOUNT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_nexthopchainingcount(
  LIBLTE_X2AP_NEXTHOPCHAININGCOUNT_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_nexthopchainingcount(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_NEXTHOPCHAININGCOUNT_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE PDCP_SN INTEGER
********************************************************************************/
typedef struct{
uint16_t PDCP_SN;
}LIBLTE_X2AP_PDCP_SN_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_pdcp_sn(
  LIBLTE_X2AP_PDCP_SN_STRUCT                                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_pdcp_sn(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PDCP_SN_STRUCT                                   *ie);

/*******************************************************************************
/* ProtocolIE PDCP_SNExtended INTEGER
********************************************************************************/
typedef struct{
uint16_t PDCP_SNExtended;
}LIBLTE_X2AP_PDCP_SNEXTENDED_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_pdcp_snextended(
  LIBLTE_X2AP_PDCP_SNEXTENDED_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_pdcp_snextended(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PDCP_SNEXTENDED_STRUCT                           *ie);

/*******************************************************************************
/* ProtocolIE PCI INTEGER
********************************************************************************/
typedef struct{
uint16_t PCI;
}LIBLTE_X2AP_PCI_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_pci(
  LIBLTE_X2AP_PCI_STRUCT                                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_pci(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PCI_STRUCT                                   *ie);

/*******************************************************************************
/* ProtocolIE PriorityLevel INTEGER
********************************************************************************/
typedef struct{
uint8_t PriorityLevel;
}LIBLTE_X2AP_PRIORITYLEVEL_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_prioritylevel(
  LIBLTE_X2AP_PRIORITYLEVEL_STRUCT                             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_prioritylevel(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PRIORITYLEVEL_STRUCT                             *ie);

/*******************************************************************************
/* ProtocolIE QCI INTEGER
********************************************************************************/
typedef struct{
uint8_t QCI;
}LIBLTE_X2AP_QCI_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_qci(
  LIBLTE_X2AP_QCI_STRUCT                                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_qci(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_QCI_STRUCT                                       *ie);

/*******************************************************************************
/* ProtocolIE RadioframeAllocationOffset INTEGER
********************************************************************************/
typedef struct{
uint8_t RadioframeAllocationOffset;
}LIBLTE_X2AP_RADIOFRAMEALLOCATIONOFFSET_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_radioframeallocationoffset(
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONOFFSET_STRUCT                                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_radioframeallocationoffset(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONOFFSET_STRUCT                                       *ie);

/*******************************************************************************
/* ProtocolIE subscriberProfileIDforRFP INTEGER
********************************************************************************/
typedef struct{
uint8_t SubscribeProfileIDforRFP;
}LIBLTE_X2AP_SUBSCRIBERPROFILEIDFORRFP_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_subscribeprofileidforrfp(
  LIBLTE_X2AP_SUBSCRIBERPROFILEIDFORRFP_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_subscribeprofileidforrfp(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SUBSCRIBERPROFILEIDFORRFP_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE Threshold_RSRP INTEGER
********************************************************************************/
typedef struct{
uint8_t Threshold_RSRP;
}LIBLTE_X2AP_THRESHOLD_RSRP_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_threshold_rsrp(
  LIBLTE_X2AP_THRESHOLD_RSRP_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_threshold_rsrp(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_THRESHOLD_RSRP_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE Threshold_RSRQ INTEGER
********************************************************************************/
typedef struct{
uint8_t Threshold_RSRQ;
}LIBLTE_X2AP_THRESHOLD_RSRQ_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_threshold_rsrq(
  LIBLTE_X2AP_THRESHOLD_RSRQ_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_threshold_rsrq(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_THRESHOLD_RSRQ_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE Time_UE_StayedInCell INTEGER
********************************************************************************/
typedef struct{
uint16_t Time_UE_StayedInCell;
}LIBLTE_X2AP_TIME_UE_STAYEDINCELL_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_time_ue_stayedincell(
  LIBLTE_X2AP_TIME_UE_STAYEDINCELL_STRUCT                      *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_time_ue_stayedincell(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TIME_UE_STAYEDINCELL_STRUCT                      *ie);

/*******************************************************************************
/* ProtocolIE Time_UE_StayedInCell_EnhancedGranularity INTEGER
********************************************************************************/
typedef struct{
uint16_t Time_UE_StayedInCell_EnhancedGranularity;
}LIBLTE_X2AP_TIME_UE_STAYEDINCELL_ENHANCEDGRANULARITY_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_time_ue_stayedincell_enhancedgranularity(
  LIBLTE_X2AP_TIME_UE_STAYEDINCELL_ENHANCEDGRANULARITY_STRUCT  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_time_ue_stayedincell_enhancedgranularity(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TIME_UE_STAYEDINCELL_ENHANCEDGRANULARITY_STRUCT  *ie);

/*******************************************************************************
/* ProtocolIE UE_S1AP_ID INTEGER
********************************************************************************/
typedef struct{
uint64_t UE_S1AP_ID;
}LIBLTE_X2AP_UE_S1AP_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ue_s1ap_id(
  LIBLTE_X2AP_UE_S1AP_ID_STRUCT                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ue_s1ap_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UE_S1AP_ID_STRUCT                  *ie);

/*******************************************************************************
/* ProtocolIE UE_X2AP_ID INTEGER
********************************************************************************/
typedef struct{
uint16_t UE_X2AP_ID;
}LIBLTE_X2AP_UE_X2AP_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ue_x2ap_id(
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ue_x2ap_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                  *ie);

/*******************************************************************************
/* ProtocolIE UL_GBR_PRB_usage INTEGER
********************************************************************************/
typedef struct{
uint16_t UL_GBR_PRB_usage;
}LIBLTE_X2AP_UL_GBR_PRB_USAGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ul_gbr_prb_usage(
  LIBLTE_X2AP_UL_GBR_PRB_USAGE_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ul_gbr_prb_usage(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UL_GBR_PRB_USAGE_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE UL_non_GBR_PRB_usage INTEGER
********************************************************************************/
typedef struct{
uint16_t UL_non_GBR_PRB_usage;
}LIBLTE_X2AP_UL_NON_GBR_PRB_USAGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ul_non_gbr_prb_usage(
  LIBLTE_X2AP_UL_NON_GBR_PRB_USAGE_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ul_non_gbr_prb_usage(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UL_NON_GBR_PRB_USAGE_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE UL_Total_PRB_usage INTEGER
********************************************************************************/
typedef struct{
uint16_t UL_Total_PRB_usage;
}LIBLTE_X2AP_UL_TOTAL_PRB_USAGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ul_total_prb_usage(
  LIBLTE_X2AP_UL_TOTAL_PRB_USAGE_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ul_total_prb_usage(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UL_TOTAL_PRB_USAGE_STRUCT                            *ie);

/* BIT STRING */

/*******************************************************************************
/* ProtocolIE CRNTI STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_CRNTI_BIT_STRING_LEN 16
typedef struct{
  uint8_t  buffer[16];
}LIBLTE_X2AP_CRNTI_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_crnti(
  LIBLTE_X2AP_CRNTI_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_crnti(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CRNTI_STRUCT                               *ie);

/*******************************************************************************
/* ProtocolIE CSG_Id STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_CSG_ID_BIT_STRING_LEN 27
typedef struct{
  uint8_t  buffer[27];
}LIBLTE_X2AP_CSG_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_csg_id(
  LIBLTE_X2AP_CSG_ID_STRUCT                                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_csg_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CSG_ID_STRUCT                                    *ie);

/*******************************************************************************
/* ProtocolIE macroENB_ID STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_MACROENB_ID_BIT_STRING_LEN 20
typedef struct{
  uint8_t  buffer[20];
}LIBLTE_X2AP_MACROENB_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_macroenb_id(
  LIBLTE_X2AP_MACROENB_ID_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_macroenb_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MACROENB_ID_STRUCT                               *ie);

/*******************************************************************************
/* ProtocolIE homeENB_ID STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_HOMEENB_ID_BIT_STRING_LEN 28
typedef struct{
  uint8_t  buffer[28];
}LIBLTE_X2AP_HOMEENB_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_homeenb_id(
  LIBLTE_X2AP_HOMEENB_ID_STRUCT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_homeenb_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_HOMEENB_ID_STRUCT                                *ie);

/*******************************************************************************
/* ProtocolIE EncryptionAlgorithms STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_ENCRYPTIONALGORITHMS_BIT_STRING_LEN 16
typedef struct{
  bool     ext;
  uint8_t  buffer[16];
}LIBLTE_X2AP_ENCRYPTIONALGORITHMS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_encryptionalgorithms(
  LIBLTE_X2AP_ENCRYPTIONALGORITHMS_STRUCT                      *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_encryptionalgorithms(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ENCRYPTIONALGORITHMS_STRUCT                      *ie);

/*******************************************************************************
/* ProtocolIE EUTRANCellIdentifier STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_EUTRANCELLIDENTIFIER_BIT_STRING_LEN 28
typedef struct{
  uint8_t  buffer[28];
}LIBLTE_X2AP_EUTRANCELLIDENTIFIER_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_eutrancellidentifier(
  LIBLTE_X2AP_EUTRANCELLIDENTIFIER_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_eutrancellidentifier(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_EUTRANCELLIDENTIFIER_STRUCT                               *ie);

/*******************************************************************************
/* ProtocolIE Fourframes STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_FOURFRAMES_BIT_STRING_LEN 24
typedef struct{
  uint8_t  buffer[24];
}LIBLTE_X2AP_FOURFRAMES_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_fourframes(
  LIBLTE_X2AP_FOURFRAMES_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_fourframes(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_FOURFRAMES_STRUCT                         *ie);

/*******************************************************************************
/* ProtocolIE IntegrityProtectionAlgorithms STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_INTEGRITYPROTECTIONALGORITHMS_BIT_STRING_LEN 16
typedef struct{
  bool     ext;
  uint8_t  buffer[16];
}LIBLTE_X2AP_INTEGRITYPROTECTIONALGORITHMS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_integrityprotectionalgorithms(
  LIBLTE_X2AP_INTEGRITYPROTECTIONALGORITHMS_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_integrityprotectionalgorithms(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_INTEGRITYPROTECTIONALGORITHMS_STRUCT             *ie);

/*******************************************************************************
/* ProtocolIE InterfacesToTrace STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_INTERFACESTOTRACE_BIT_STRING_LEN 8
typedef struct{
  uint8_t  buffer[8];
}LIBLTE_X2AP_INTERFACESTOTRACE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_interfacestotrace(
  LIBLTE_X2AP_INTERFACESTOTRACE_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_interfacestotrace(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_INTERFACESTOTRACE_STRUCT                         *ie);

/*******************************************************************************
/* ProtocolIE Key_eNodeB_Star STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_KEY_ENODEB_STAR_BIT_STRING_LEN 256
typedef struct{
  uint8_t  buffer[256];
}LIBLTE_X2AP_KEY_ENODEB_STAR_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_key_enodeb_star(
  LIBLTE_X2AP_KEY_ENODEB_STAR_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_key_enodeb_star(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_KEY_ENODEB_STAR_STRUCT                          *ie);

/*******************************************************************************
/* ProtocolIE MDT_Location_Info STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_MDT_LOCATION_INFO_BIT_STRING_LEN 8
typedef struct{
  uint8_t buffer[8];
}LIBLTE_X2AP_MDT_LOCATION_INFO_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mdt_location_info(
  LIBLTE_X2AP_MDT_LOCATION_INFO_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mdt_location_info(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MDT_LOCATION_INFO_STRUCT                         *ie);

/*******************************************************************************
/* ProtocolIE MeasurementsToActivate STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_MEASUREMENTSTOACTIVATE_BIT_STRING_LEN 8
typedef struct{
  uint8_t  buffer[8];
}LIBLTE_X2AP_MEASUREMENTSTOACTIVATE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurementstoactivate(
  LIBLTE_X2AP_MEASUREMENTSTOACTIVATE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurementstoactivate(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MEASUREMENTSTOACTIVATE_STRUCT                    *ie);

/*******************************************************************************
/* ProtocolIE Oneframe STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_ONEFRAME_BIT_STRING_LEN 6
typedef struct{
  uint8_t  buffer[6];
}LIBLTE_X2AP_ONEFRAME_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_oneframe(
  LIBLTE_X2AP_ONEFRAME_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_oneframe(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ONEFRAME_STRUCT                         *ie);

/*******************************************************************************
/* ProtocolIE ReceiveStatusofULPDCPSDUs STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_RECEIVESTATUSOFULPDCPSDUS_BIT_STRING_LEN 4096
typedef struct{
  uint8_t  buffer[4096];
}LIBLTE_X2AP_RECEIVESTATUSOFULPDCPSDUS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_receivestatusofulpdcpsdus(
  LIBLTE_X2AP_RECEIVESTATUSOFULPDCPSDUS_STRUCT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_receivestatusofulpdcpsdus(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_RECEIVESTATUSOFULPDCPSDUS_STRUCT                 *ie);

/*******************************************************************************
/* ProtocolIE ReceiveStatusOfULPDCPSDUsExtended DYNAMIC BIT STRING
********************************************************************************/
// lb:1, ub:16384
typedef struct{
  uint32_t n_bits;
  uint8_t  buffer[16384];
}LIBLTE_X2AP_RECEIVESTATUSOFULPDCPSDUSEXTENDED_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_receivestatusofulpdcpsdusextended(
  LIBLTE_X2AP_RECEIVESTATUSOFULPDCPSDUSEXTENDED_STRUCT         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_receivestatusofulpdcpsdusextended(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_RECEIVESTATUSOFULPDCPSDUSEXTENDED_STRUCT         *ie);

/*******************************************************************************
/* ProtocolIE ReportCharateristics STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_REPORTCHARATERISTICS_BIT_STRING_LEN 32
typedef struct{
  uint8_t  buffer[32];
}LIBLTE_X2AP_REPORTCHARACTERISTICS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_reportcharacteristics(
  LIBLTE_X2AP_REPORTCHARACTERISTICS_STRUCT                     *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_reportcharacteristics(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_REPORTCHARACTERISTICS_STRUCT                     *ie);

/*******************************************************************************
/* ProtocolIE ShortMAC_I STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_SHORTMAC_I_BIT_STRING_LEN 16
typedef struct{
  uint8_t  buffer[16];
}LIBLTE_X2AP_SHORTMAC_I_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_shortmac_i(
  LIBLTE_X2AP_SHORTMAC_I_STRUCT                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_shortmac_i(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SHORTMAC_I_STRUCT                       *ie);

/*******************************************************************************
/* ProtocolIE TraceCollectionEntityIPAddress DYNAMIC BIT STRING
********************************************************************************/
// lb:1, ub:160
typedef struct{
  bool     ext;
  uint32_t n_bits;
  uint8_t  buffer[160];
}LIBLTE_X2AP_TRACECOLLECTIONENTITYIPADDRESS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tracecollectionentityipaddress(
  LIBLTE_X2AP_TRACECOLLECTIONENTITYIPADDRESS_STRUCT                     *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tracecollectionentityipaddress(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TRACECOLLECTIONENTITYIPADDRESS_STRUCT                     *ie);

/*******************************************************************************
/* ProtocolIE TransportLayerAddress DYNAMIC BIT STRING
********************************************************************************/
// lb:1, ub:160
typedef struct{
  bool     ext;
  uint32_t n_bits;
  uint8_t  buffer[160];
}LIBLTE_X2AP_TRANSPORTLAYERADDRESS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_transportlayeraddress(
  LIBLTE_X2AP_TRANSPORTLAYERADDRESS_STRUCT                     *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_transportlayeraddress(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TRANSPORTLAYERADDRESS_STRUCT                     *ie);

/*******************************************************************************
/* ProtocolIE UL_HighInterferenceIndication DYNAMIC BIT STRING
********************************************************************************/
// lb:1, ub:120
typedef struct{
  bool     ext;
  uint32_t n_bits;
  uint8_t  buffer[120];
}LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ul_highinterferenceindication(
  LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATION_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ul_highinterferenceindication(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATION_STRUCT                               *ie);

/* Octet String */

/*******************************************************************************
/* ProtocolIE TBCD_STRING STATIC OCTET STRING
********************************************************************************/
#define LIBLTE_X2AP_TBCD_STRING_OCTET_STRING_LEN 3
typedef struct{
  uint8_t  buffer[3];
}LIBLTE_X2AP_TBCD_STRING_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tbcd_string(
  LIBLTE_X2AP_TBCD_STRING_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tbcd_string(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TBCD_STRING_STRUCT                               *ie);

/*******************************************************************************
/* ProtocolIE EUTRANTraceID STATIC OCTET STRING
********************************************************************************/
#define LIBLTE_X2AP_EUTRANTRACEID_OCTET_STRING_LEN 8
typedef struct{
  uint8_t  buffer[8];
}LIBLTE_X2AP_EUTRANTRACEID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_eutrantraceid(
  LIBLTE_X2AP_EUTRANTRACEID_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_eutrantraceid(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_EUTRANTRACEID_STRUCT                           *ie);

/*******************************************************************************
/* ProtocolIE GTP_TEI STATIC OCTET STRING
********************************************************************************/
#define LIBLTE_X2AP_GTP_TEI_OCTET_STRING_LEN 4
typedef struct{
  uint8_t  buffer[4];
}LIBLTE_X2AP_GTP_TEI_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_gtp_tei(
  LIBLTE_X2AP_GTP_TEI_STRUCT                                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_gtp_tei(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_GTP_TEI_STRUCT                                  *ie);

/*******************************************************************************
/* ProtocolIE LAC STATIC OCTET STRING
********************************************************************************/
#define LIBLTE_X2AP_LAC_OCTET_STRING_LEN 2
typedef struct{
  uint8_t  buffer[2];
}LIBLTE_X2AP_LAC_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_lac(
  LIBLTE_X2AP_LAC_STRUCT                                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_lac(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_LAC_STRUCT                                       *ie);

/*******************************************************************************
/* ProtocolIE LastVisitedUTRANCellInformation DYNAMIC OCTET STRING
********************************************************************************/
// lb:0, ub:16318
typedef struct{
  uint32_t n_octets;
  uint8_t  buffer[16318];
}LIBLTE_X2AP_LASTVISITEDUTRANCELLINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_lastvisitedutrancellinformation(
  LIBLTE_X2AP_LASTVISITEDUTRANCELLINFORMATION_STRUCT           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_lastvisitedutrancellinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_LASTVISITEDUTRANCELLINFORMATION_STRUCT           *ie);

/*******************************************************************************
/* ProtocolIE MME_Group_ID STATIC OCTET STRING
********************************************************************************/
#define LIBLTE_X2AP_MME_GROUP_ID_OCTET_STRING_LEN 2
typedef struct{
  uint8_t  buffer[2];
}LIBLTE_X2AP_MME_GROUP_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mme_group_id(
  LIBLTE_X2AP_MME_GROUP_ID_STRUCT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mme_group_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MME_GROUP_ID_STRUCT                              *ie);

/*******************************************************************************
/* ProtocolIE MME_Code STATIC OCTET STRING
********************************************************************************/
#define LIBLTE_X2AP_MME_CODE_OCTET_STRING_LEN 1
typedef struct{
  uint8_t  buffer[1];
}LIBLTE_X2AP_MME_CODE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mme_code(
  LIBLTE_X2AP_MME_CODE_STRUCT                                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mme_code(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MME_CODE_STRUCT                                  *ie);

/*******************************************************************************
/* ProtocolIE MBMS_Service_Area_Identity STATIC OCTET STRING
********************************************************************************/
#define LIBLTE_X2AP_MBMS_SERVICE_AREA_IDENTITY_OCTET_STRING_LEN 2
typedef struct{
  uint8_t  buffer[2];
}LIBLTE_X2AP_MBMS_SERVICE_AREA_IDENTITY_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mbms_service_area_identity(
  LIBLTE_X2AP_MBMS_SERVICE_AREA_IDENTITY_STRUCT                                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mbms_service_area_identity(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MBMS_SERVICE_AREA_IDENTITY_STRUCT                                  *ie);

/*******************************************************************************
/* ProtocolIE PLMN_identity STATIC OCTET STRING
********************************************************************************/
#define LIBLTE_X2AP_PLMN_IDENTITY_OCTET_STRING_LEN 3
typedef struct{
  uint8_t  buffer[3];
}LIBLTE_X2AP_PLMN_IDENTITY_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_plmn_identity(
  LIBLTE_X2AP_PLMN_IDENTITY_STRUCT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_plmn_identity(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PLMN_IDENTITY_STRUCT                              *ie);

/*******************************************************************************
/* ProtocolIE RRC_Context DYNAMIC OCTET STRING
********************************************************************************/
// lb:0, ub:16318
typedef struct{
  uint32_t n_octets;
  uint8_t  buffer[16318];
}LIBLTE_X2AP_RRC_CONTEXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_rrc_context(
  LIBLTE_X2AP_RRC_CONTEXT_STRUCT                             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_rrc_context(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_RRC_CONTEXT_STRUCT                             *ie);

/*******************************************************************************
/* ProtocolIE TAC STATIC OCTET STRING
********************************************************************************/
#define LIBLTE_X2AP_TAC_OCTET_STRING_LEN 2
typedef struct{
  uint8_t  buffer[2];
}LIBLTE_X2AP_TAC_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tac(
  LIBLTE_X2AP_TAC_STRUCT                                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tac(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TAC_STRUCT                                       *ie);

/*******************************************************************************
/* ProtocolIE TargetCellInUTRAN DYNAMIC OCTET STRING
********************************************************************************/
// lb:0, ub:16318
typedef struct{
  uint32_t n_octets;
  uint8_t  buffer[16318];
}LIBLTE_X2AP_TARGETCELLINUTRAN_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_targetcellinutran(
  LIBLTE_X2AP_TARGETCELLINUTRAN_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_targetcellinutran(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TARGETCELLINUTRAN_STRUCT                         *ie);

/*******************************************************************************
/* ProtocolIE TargeteNBtoSource_eNBTransparentContainer DYNAMIC OCTET STRING
********************************************************************************/
// lb:0, ub:16318
typedef struct{
  uint32_t n_octets;
  uint8_t  buffer[16318];
}LIBLTE_X2AP_TARGETENBTOSOURCE_ENBTRANSPARENTCONTAINER_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_targetenbtosource_enbtransparentcontainer(
  LIBLTE_X2AP_TARGETENBTOSOURCE_ENBTRANSPARENTCONTAINER_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_targetenbtosource_enbtransparentcontainer(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TARGETENBTOSOURCE_ENBTRANSPARENTCONTAINER_STRUCT                         *ie);

/*******************************************************************************
/* ProtocolIE UL_RLF_Report_Container DYNAMIC OCTET STRING
********************************************************************************/
// lb:0, ub:16318
typedef struct{
  uint32_t n_octets;
  uint8_t  buffer[16318];
}LIBLTE_X2AP_UE_RLF_REPORT_CONTAINER_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ue_rlf_report_container(
  LIBLTE_X2AP_UE_RLF_REPORT_CONTAINER_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ue_rlf_report_container(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UE_RLF_REPORT_CONTAINER_STRUCT                         *ie);

/* ENUMERATED */

/*******************************************************************************
/* ProtocolIE AdditionalSpecialSubframePatterns ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_SSP0,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_SSP1,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_SSP2,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_SSP3,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_SSP4,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_SSP5,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_SSP6,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_SSP7,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_SSP8,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_SSP9,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_N_ITEMS,
}LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_ENUM;
static const char liblte_x2ap_additionalspecialsubframepatterns_text[LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_N_ITEMS][80] = {
  "ssp0",
  "ssp1",
  "ssp2",
  "ssp3",
  "ssp4",
  "ssp5",
  "ssp6",
  "ssp7",
  "ssp8",
  "ssp9",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_ENUM e;
}LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_additionalspecialsubframepatterns(
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_ENUM_EXT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_additionalspecialsubframepatterns(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_ENUM_EXT                               *ie);

/*******************************************************************************
/* ProtocolIE CauseMisc ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_CAUSEMISC_CONTROL_PROCESSING_OVERLOAD,
  LIBLTE_X2AP_CAUSEMISC_HARDWARE_FAILURE,
  LIBLTE_X2AP_CAUSEMISC_OM_INTERVENTION,
  LIBLTE_X2AP_CAUSEMISC_NOT_ENOUGH_USER_PLANE_PROCESSING_RESOURCES,
  LIBLTE_X2AP_CAUSEMISC_UNSPECIFIED,
  LIBLTE_X2AP_CAUSEMISC_N_ITEMS,
}LIBLTE_X2AP_CAUSEMISC_ENUM;
static const char liblte_x2ap_causemisc_text[LIBLTE_X2AP_CAUSEMISC_N_ITEMS][80] = {
  "control-processing-overload",
  "hardware-failure",
  "om-intervention",
  "not-enough-user-plane-processing-resources",
  "unspecified",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CAUSEMISC_ENUM e;
}LIBLTE_X2AP_CAUSEMISC_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_causemisc(
  LIBLTE_X2AP_CAUSEMISC_ENUM_EXT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_causemisc(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CAUSEMISC_ENUM_EXT                               *ie);

/*******************************************************************************
/* ProtocolIE CauseProtocol ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_CAUSEPROTOCOL_TRANSFER_SYNTAX_ERROR,
  LIBLTE_X2AP_CAUSEPROTOCOL_ABSTRACT_SYNTAX_ERROR_REJECT,
  LIBLTE_X2AP_CAUSEPROTOCOL_ABSTRACT_SYNTAX_ERROR_IGNORE_AND_NOTIFY,
  LIBLTE_X2AP_CAUSEPROTOCOL_MESSAGE_NOT_COMPATIBLE_WITH_RECEIVER_STATE,
  LIBLTE_X2AP_CAUSEPROTOCOL_SEMANTIC_ERROR,
  LIBLTE_X2AP_CAUSEPROTOCOL_UNSPECIFIED,
  LIBLTE_X2AP_CAUSEPROTOCOL_ABSTRACT_SYNTAX_ERROR_FALSELY_CONSTRUCTED_MESSAGE,
  LIBLTE_X2AP_CAUSEPROTOCOL_N_ITEMS,
}LIBLTE_X2AP_CAUSEPROTOCOL_ENUM;
static const char liblte_x2ap_causeprotocol_text[LIBLTE_X2AP_CAUSEPROTOCOL_N_ITEMS][80] = {
  "transfer-syntax-error",
  "abstract-syntax-error-reject",
  "abstract-syntax-error-ignore-and-notify",
  "message-not-compatible-with-receiver-state",
  "semantic-error",
  "unspecified",
  "abstract-syntax-error-falsely-constructed-message",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CAUSEPROTOCOL_ENUM e;
}LIBLTE_X2AP_CAUSEPROTOCOL_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_causeprotocol(
  LIBLTE_X2AP_CAUSEPROTOCOL_ENUM_EXT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_causeprotocol(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CAUSEPROTOCOL_ENUM_EXT                           *ie);

/*******************************************************************************
/* ProtocolIE CauseRadioNetwork ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_CAUSERADIONETWORK_HANDOVER_DESIRABLE_FOR_RADIO_REASONS,
  LIBLTE_X2AP_CAUSERADIONETWORK_TIME_CRITICAL_HANDOVER,
  LIBLTE_X2AP_CAUSERADIONETWORK_RESOURCE_OPTIMISATION_HANDOVER,
  LIBLTE_X2AP_CAUSERADIONETWORK_REDUCE_LOAD_IN_SERVING_CELL,
  LIBLTE_X2AP_CAUSERADIONETWORK_PARTIAL_HANDOVER,
  LIBLTE_X2AP_CAUSERADIONETWORK_UNKNOWN_NEW_ENB_UE_X2AP_ID,
  LIBLTE_X2AP_CAUSERADIONETWORK_UNKNOWN_OLD_ENB_UE_X2AP_ID,
  LIBLTE_X2AP_CAUSERADIONETWORK_UNKNOWN_PAIR_OF_UE_X2AP_ID,
  LIBLTE_X2AP_CAUSERADIONETWORK_HO_TARGET_NOT_ALLOWED,
  LIBLTE_X2AP_CAUSERADIONETWORK_TX2RELOCOVERALL_EXPIRY,
  LIBLTE_X2AP_CAUSERADIONETWORK_TRELOCPREP_EXPIRY,
  LIBLTE_X2AP_CAUSERADIONETWORK_CELL_NOT_AVAILABLE,
  LIBLTE_X2AP_CAUSERADIONETWORK_NO_RADIO_RESOURCES_AVAILABLE_IN_TARGET_CELL,
  LIBLTE_X2AP_CAUSERADIONETWORK_INVALID_MME_GROUPID,
  LIBLTE_X2AP_CAUSERADIONETWORK_UNKNOWN_MME_CODE,
  LIBLTE_X2AP_CAUSERADIONETWORK_ENCRYPTION_AND_OR_INTEGRITY_PROTECTION_ALGORITHMS_NOT_SUPPORTED,
  LIBLTE_X2AP_CAUSERADIONETWORK_REPORTCHARACTERISTICSEMPTY,
  LIBLTE_X2AP_CAUSERADIONETWORK_NOREPORTPERIODICITY,
  LIBLTE_X2AP_CAUSERADIONETWORK_EXISTINGMEASUREMENTID,
  LIBLTE_X2AP_CAUSERADIONETWORK_UNKNOWN_ENB_MEASUREMENT_ID,
  LIBLTE_X2AP_CAUSERADIONETWORK_MEASUREMENT_TEMPORARILY_NOT_AVAILABLE,
  LIBLTE_X2AP_CAUSERADIONETWORK_UNSPECIFIED,
  LIBLTE_X2AP_CAUSERADIONETWORK_LOAD_BALANCING,
  LIBLTE_X2AP_CAUSERADIONETWORK_HANDOVER_OPTIMISATION,
  LIBLTE_X2AP_CAUSERADIONETWORK_VALUE_OUT_OF_ALLOWED_RANGE,
  LIBLTE_X2AP_CAUSERADIONETWORK_MULTIPLE_E_RAB_ID_INSTANCES,
  LIBLTE_X2AP_CAUSERADIONETWORK_SWITCH_OFF_ONGOING,
  LIBLTE_X2AP_CAUSERADIONETWORK_NOT_SUPPORTED_QCI_VALUE,
  LIBLTE_X2AP_CAUSERADIONETWORK_MEASUREMENT_NOT_SUPPORTED_FOR_THE_OBJECT,
  LIBLTE_X2AP_CAUSERADIONETWORK_N_ITEMS,
}LIBLTE_X2AP_CAUSERADIONETWORK_ENUM;
static const char liblte_x2ap_causeradionetwork_text[LIBLTE_X2AP_CAUSERADIONETWORK_N_ITEMS][80] = {
  "handover-desirable-for-radio-ressons",
  "time-critical-handover",
  "resource-optimisation-handover",
  "reduce-load-in-serving-cell",
  "partial-handover",
  "unknown-new-eNB-UE-X2AP-ID",
  "unknown-old-eNB-UE-X2AP-ID",
  "unknown-pair-of-UE-X2AP-ID",
  "ho-target-not-allowed",
  "tx2relocoverall-expiry",
  "trelocprep-expiry",
  "cell-not-available",
  "no-radio-resources-available-in-target-cell",
  "invalid-MME-GroupID",
  "unknown-MME-Code",
  "encryption-and-or-integrity-protection-algorithms-not-supported",
  "reportCharacteristicsEmpty",
  "noReportPeriodicity",
  "existingMeasurementID",
  "unknown-eNB-Measurement-ID",
  "measurement-temporarily-not-available",
  "unspecified",
  "load-balancing",
  "handover-optimisation",
  "value-out-of-allowed-range",
  "multiple-E-RAB-ID-instances",
  "switch-off-ongoing",
  "not-supported-QCI-value",
  "measurement-not-supported-for-the-object",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CAUSERADIONETWORK_ENUM e;
}LIBLTE_X2AP_CAUSERADIONETWORK_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_causeradionetwork(
  LIBLTE_X2AP_CAUSERADIONETWORK_ENUM_EXT                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_causeradionetwork(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CAUSERADIONETWORK_ENUM_EXT                       *ie);

/*******************************************************************************
/* ProtocolIE CauseTransport ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_CAUSETRANSPORT_TRANSPORT_RESOURCE_UNAVAILABLE,
  LIBLTE_X2AP_CAUSETRANSPORT_UNSPECIFIED,
  LIBLTE_X2AP_CAUSETRANSPORT_N_ITEMS,
}LIBLTE_X2AP_CAUSETRANSPORT_ENUM;
static const char liblte_x2ap_causetransport_text[LIBLTE_X2AP_CAUSETRANSPORT_N_ITEMS][80] = {
  "transport-resource-unavailable",
  "unspecified",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CAUSETRANSPORT_ENUM e;
}LIBLTE_X2AP_CAUSETRANSPORT_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_causetransport(
  LIBLTE_X2AP_CAUSETRANSPORT_ENUM_EXT                          *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_causetransport(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CAUSETRANSPORT_ENUM_EXT                          *ie);

/*******************************************************************************
/* ProtocolIE Cell_Size ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_CELL_SIZE_VERYSMALL,
  LIBLTE_X2AP_CELL_SIZE_SMALL,
  LIBLTE_X2AP_CELL_SIZE_MEDIUM,
  LIBLTE_X2AP_CELL_SIZE_LARGE,
  LIBLTE_X2AP_CELL_SIZE_N_ITEMS,
}LIBLTE_X2AP_CELL_SIZE_ENUM;
static const char liblte_x2ap_cell_size_text[LIBLTE_X2AP_CELL_SIZE_N_ITEMS][80] = {
  "verysmall",
  "small",
  "medium",
  "large",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CELL_SIZE_ENUM e;
}LIBLTE_X2AP_CELL_SIZE_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cell_size(
  LIBLTE_X2AP_CELL_SIZE_ENUM_EXT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cell_size(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELL_SIZE_ENUM_EXT                               *ie);

/*******************************************************************************
/* ProtocolIE CSGMembershipStatus ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_CSGMEMBERSHIPSTATUS_MEMBER,
  LIBLTE_X2AP_CSGMEMBERSHIPSTATUS_NOT_MEMBER,
  LIBLTE_X2AP_CSGMEMBERSHIPSTATUS_N_ITEMS,
}LIBLTE_X2AP_CSGMEMBERSHIPSTATUS_ENUM;
static const char liblte_x2ap_csgmembershipstatus_text[LIBLTE_X2AP_CSGMEMBERSHIPSTATUS_N_ITEMS][80] = {
  "member",
  "not-member",
};

LIBLTE_ERROR_ENUM liblte_x2ap_pack_csgmembershipstatus(
  LIBLTE_X2AP_CSGMEMBERSHIPSTATUS_ENUM                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_csgmembershipstatus(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CSGMEMBERSHIPSTATUS_ENUM                         *ie);

/*******************************************************************************
/* ProtocolIE CyclicPrefixDL ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_CYCLICPREFIXDL_NORMAL,
  LIBLTE_X2AP_CYCLICPREFIXDL_EXTENDED,
  LIBLTE_X2AP_CYCLICPREFIXDL_N_ITEMS,
}LIBLTE_X2AP_CYCLICPREFIXDL_ENUM;
static const char liblte_x2ap_cyclicprefixdl_text[LIBLTE_X2AP_CYCLICPREFIXDL_N_ITEMS][80] = {
  "normal",
  "extended",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CYCLICPREFIXDL_ENUM e;
}LIBLTE_X2AP_CYCLICPREFIXDL_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cyclicprefixdl(
  LIBLTE_X2AP_CYCLICPREFIXDL_ENUM_EXT                                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cyclicprefixdl(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CYCLICPREFIXDL_ENUM_EXT                                   *ie);

/*******************************************************************************
/* ProtocolIE CyclicPrefixUL ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_CYCLICPREFIXUL_NORMAL,
  LIBLTE_X2AP_CYCLICPREFIXUL_EXTENDED,
  LIBLTE_X2AP_CYCLICPREFIXUL_N_ITEMS,
}LIBLTE_X2AP_CYCLICPREFIXUL_ENUM;
static const char liblte_x2ap_cyclicprefixul_text[LIBLTE_X2AP_CYCLICPREFIXUL_N_ITEMS][80] = {
  "normal",
  "extended",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CYCLICPREFIXUL_ENUM e;
}LIBLTE_X2AP_CYCLICPREFIXUL_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cyclicprefixul(
  LIBLTE_X2AP_CYCLICPREFIXUL_ENUM_EXT                                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cyclicprefixul(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CYCLICPREFIXUL_ENUM_EXT                                   *ie);

/*******************************************************************************
/* ProtocolIE DL_Forwarding ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_DL_FORWARDING_DL_FORWARDING_PROPOSED,
  LIBLTE_X2AP_DL_FORWARDING_N_ITEMS,
}LIBLTE_X2AP_DL_FORWARDING_ENUM;
static const char liblte_x2ap_dl_forwarding_text[LIBLTE_X2AP_DL_FORWARDING_N_ITEMS][80] = {
  "dL-Forwarding-proposed",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_DL_FORWARDING_ENUM e;
}LIBLTE_X2AP_DL_FORWARDING_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_dl_forwarding(
  LIBLTE_X2AP_DL_FORWARDING_ENUM_EXT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_dl_forwarding(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_DL_FORWARDING_ENUM_EXT                           *ie);

/*******************************************************************************
/* ProtocolIE DeactivationIndication ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_DEACTIVATIONINDICATION_DEACTIVATED,
  LIBLTE_X2AP_DEACTIVATIONINDICATION_N_ITEMS,
}LIBLTE_X2AP_DEACTIVATIONINDICATION_ENUM;
static const char liblte_x2ap_deactivationindication_text[LIBLTE_X2AP_DEACTIVATIONINDICATION_N_ITEMS][80] = {
  "deactivated",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_DEACTIVATIONINDICATION_ENUM e;
}LIBLTE_X2AP_DEACTIVATIONINDICATION_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_deactivationindication(
  LIBLTE_X2AP_DEACTIVATIONINDICATION_ENUM_EXT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_deactivationindication(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_DEACTIVATIONINDICATION_ENUM_EXT            *ie);

/*******************************************************************************
/* ProtocolIE EventType ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_EVENTTYPE_CHANGE_OF_SERVING_CELL,
  LIBLTE_X2AP_EVENTTYPE_N_ITEMS,
}LIBLTE_X2AP_EVENTTYPE_ENUM;
static const char liblte_x2ap_eventtype_text[LIBLTE_X2AP_EVENTTYPE_N_ITEMS][80] = {
  "change-of-serving-cell",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_EVENTTYPE_ENUM e;
}LIBLTE_X2AP_EVENTTYPE_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_eventtype(
  LIBLTE_X2AP_EVENTTYPE_ENUM_EXT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_eventtype(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_EVENTTYPE_ENUM_EXT                               *ie);

/*******************************************************************************
/* ProtocolIE ForbiddenInterRATs ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_FORBIDDENINTERRATS_ALL,
  LIBLTE_X2AP_FORBIDDENINTERRATS_GERAN,
  LIBLTE_X2AP_FORBIDDENINTERRATS_UTRAN,
  LIBLTE_X2AP_FORBIDDENINTERRATS_CDMA2000,
  LIBLTE_X2AP_FORBIDDENINTERRATS_GERANANDUTRAN,
  LIBLTE_X2AP_FORBIDDENINTERRATS_CDMA2000ANDUTRAN,
  LIBLTE_X2AP_FORBIDDENINTERRATS_N_ITEMS,
}LIBLTE_X2AP_FORBIDDENINTERRATS_ENUM;
static const char liblte_x2ap_forbiddeninterrats_text[LIBLTE_X2AP_FORBIDDENINTERRATS_N_ITEMS][80] = {
  "all",
  "geran",
  "utran",
  "cdma2000",
  "geranandutran",
  "cdma2000andutran",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_FORBIDDENINTERRATS_ENUM e;
}LIBLTE_X2AP_FORBIDDENINTERRATS_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_forbiddeninterrats(
  LIBLTE_X2AP_FORBIDDENINTERRATS_ENUM_EXT                      *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_forbiddeninterrats(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_FORBIDDENINTERRATS_ENUM_EXT                      *ie);

/*******************************************************************************
/* ProtocolIE HandoverReportType ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_HANDOVERREPORTTYPE_HOTOOEARLY,
  LIBLTE_X2AP_HANDOVERREPORTTYPE_HOTOWRONGCELL,
  LIBLTE_X2AP_HANDOVERREPORTTYPE_INTERRATPINGPONG,
  LIBLTE_X2AP_HANDOVERREPORTTYPE_N_ITEMS,
}LIBLTE_X2AP_HANDOVERREPORTTYPE_ENUM;
static const char liblte_x2ap_handoverreporttype_text[LIBLTE_X2AP_HANDOVERREPORTTYPE_N_ITEMS][80] = {
  "hoTooEarly",
  "hoToWrongCell",
  "interRATpingpong",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_HANDOVERREPORTTYPE_ENUM e;
}LIBLTE_X2AP_HANDOVERREPORTTYPE_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_handoverreporttype(
  LIBLTE_X2AP_HANDOVERREPORTTYPE_ENUM_EXT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handoverreporttype(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_HANDOVERREPORTTYPE_ENUM_EXT                            *ie);

/*******************************************************************************
/* ProtocolIE InvokeIndication ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_INVOKEINDICATION_ABS_INFORMATION,
  LIBLTE_X2AP_INVOKEINDICATION_N_ITEMS,
}LIBLTE_X2AP_INVOKEINDICATION_ENUM;
static const char liblte_x2ap_invokeindication_text[LIBLTE_X2AP_INVOKEINDICATION_N_ITEMS][80] = {
  "abs-information",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_INVOKEINDICATION_ENUM e;
}LIBLTE_X2AP_INVOKEINDICATION_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_invokeindication(
  LIBLTE_X2AP_INVOKEINDICATION_ENUM_EXT                          *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_invokeindication(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_INVOKEINDICATION_ENUM_EXT                          *ie);
  
/*******************************************************************************
/* ProtocolIE Links_to_log ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_LINKS_TO_LOG_UPLINK,
  LIBLTE_X2AP_LINKS_TO_LOG_DOWNLINK,
  LIBLTE_X2AP_LINKS_TO_LOG_BOTH_UPLINK_AND_DOWNLINK,
  LIBLTE_X2AP_LINKS_TO_LOG_N_ITEMS,
}LIBLTE_X2AP_LINKS_TO_LOG_ENUM;
static const char liblte_x2ap_links_to_log_text[LIBLTE_X2AP_LINKS_TO_LOG_N_ITEMS][80] = {
  "uplink",
  "downlink",
  "both-uplink-and-downlink",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_LINKS_TO_LOG_ENUM e;
}LIBLTE_X2AP_LINKS_TO_LOG_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_links_to_log(
  LIBLTE_X2AP_LINKS_TO_LOG_ENUM_EXT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_links_to_log(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_LINKS_TO_LOG_ENUM_EXT                            *ie);

/*******************************************************************************
/* ProtocolIE LoadIndicator ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_LOADINDICATOR_LOWLOAD,
  LIBLTE_X2AP_LOADINDICATOR_MEDIUMLOAD,
  LIBLTE_X2AP_LOADINDICATOR_HIGHLOAD,
  LIBLTE_X2AP_LOADINDICATOR_OVERLOAD,
  LIBLTE_X2AP_LOADINDICATOR_N_ITEMS,
}LIBLTE_X2AP_LOADINDICATOR_ENUM;
static const char liblte_x2ap_loadindicator_text[LIBLTE_X2AP_LOADINDICATOR_N_ITEMS][80] = {
  "lowLoad",
  "mediumLoad",
  "highLoad",
  "overLoad",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_LOADINDICATOR_ENUM e;
}LIBLTE_X2AP_LOADINDICATOR_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_loadindicator(
  LIBLTE_X2AP_LOADINDICATOR_ENUM_EXT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_loadindicator(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_LOADINDICATOR_ENUM_EXT                            *ie);

/*******************************************************************************
/* ProtocolIE M1ReportingTrigger ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_M1REPORTINGTRIGGER_PERIODIC,
  LIBLTE_X2AP_M1REPORTINGTRIGGER_A2EVENTTRIGGERED,
  LIBLTE_X2AP_M1REPORTINGTRIGGER_A2EVENTTRIGGERED_PERIODIC,
  LIBLTE_X2AP_M1REPORTINGTRIGGER_N_ITEMS,
}LIBLTE_X2AP_M1REPORTINGTRIGGER_ENUM;
static const char liblte_x2ap_m1reportingtrigger_text[LIBLTE_X2AP_M1REPORTINGTRIGGER_N_ITEMS][80] = {
  "periodic",
  "a2eventtriggered",
  "a2eventtriggered-periodic",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_M1REPORTINGTRIGGER_ENUM e;
}LIBLTE_X2AP_M1REPORTINGTRIGGER_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m1reportingtrigger(
  LIBLTE_X2AP_M1REPORTINGTRIGGER_ENUM_EXT                      *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m1reportingtrigger(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_M1REPORTINGTRIGGER_ENUM_EXT                      *ie);

/*******************************************************************************
/* ProtocolIE M3period ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_M3PERIOD_MS100,
  LIBLTE_X2AP_M3PERIOD_MS1000,
  LIBLTE_X2AP_M3PERIOD_MS10000,
  LIBLTE_X2AP_M3PERIOD_N_ITEMS,
}LIBLTE_X2AP_M3PERIOD_ENUM;
static const char liblte_x2ap_m3period_text[LIBLTE_X2AP_M3PERIOD_N_ITEMS][80] = {
  "ms100",
  "ms1000",
  "ms10000",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_M3PERIOD_ENUM e;
}LIBLTE_X2AP_M3PERIOD_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m3period(
  LIBLTE_X2AP_M3PERIOD_ENUM_EXT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m3period(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_M3PERIOD_ENUM_EXT                                *ie);

/*******************************************************************************
/* ProtocolIE M4period ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_M4PERIOD_MS1024,
  LIBLTE_X2AP_M4PERIOD_MS2048,
  LIBLTE_X2AP_M4PERIOD_MS5120,
  LIBLTE_X2AP_M4PERIOD_MS10240,
  LIBLTE_X2AP_M4PERIOD_MIN1,
  LIBLTE_X2AP_M4PERIOD_N_ITEMS,
}LIBLTE_X2AP_M4PERIOD_ENUM;
static const char liblte_x2ap_m4period_text[LIBLTE_X2AP_M4PERIOD_N_ITEMS][80] = {
  "ms1024",
  "ms2048",
  "ms5120",
  "ms10240",
  "min1",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_M4PERIOD_ENUM e;
}LIBLTE_X2AP_M4PERIOD_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m4period(
  LIBLTE_X2AP_M4PERIOD_ENUM_EXT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m4period(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_M4PERIOD_ENUM_EXT                                *ie);

/*******************************************************************************
/* ProtocolIE M5period ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_M5PERIOD_MS1024,
  LIBLTE_X2AP_M5PERIOD_MS2048,
  LIBLTE_X2AP_M5PERIOD_MS5120,
  LIBLTE_X2AP_M5PERIOD_MS10240,
  LIBLTE_X2AP_M5PERIOD_MIN1,
  LIBLTE_X2AP_M5PERIOD_N_ITEMS,
}LIBLTE_X2AP_M5PERIOD_ENUM;
static const char liblte_x2ap_m5period_text[LIBLTE_X2AP_M5PERIOD_N_ITEMS][80] = {
  "ms1024",
  "ms2048",
  "ms5120",
  "ms10240",
  "min1",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_M5PERIOD_ENUM e;
}LIBLTE_X2AP_M5PERIOD_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m5period(
  LIBLTE_X2AP_M5PERIOD_ENUM_EXT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m5period(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_M5PERIOD_ENUM_EXT                                *ie);

/*******************************************************************************
/* ProtocolIE MDT_Activation ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_MDT_ACTIVATION_IMMEDIATE_MDT_ONLY,
  LIBLTE_X2AP_MDT_ACTIVATION_IMMEDIATE_MDT_AND_TRACE,
  LIBLTE_X2AP_MDT_ACTIVATION_N_ITEMS,
}LIBLTE_X2AP_MDT_ACTIVATION_ENUM;
static const char liblte_x2ap_mdt_activation_text[LIBLTE_X2AP_MDT_ACTIVATION_N_ITEMS][80] = {
  "immediate-MDT-only",
  "immediate-MDT-and-Trace",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MDT_ACTIVATION_ENUM e;
}LIBLTE_X2AP_MDT_ACTIVATION_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mdt_activation(
  LIBLTE_X2AP_MDT_ACTIVATION_ENUM_EXT                          *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mdt_activation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MDT_ACTIVATION_ENUM_EXT                          *ie);

/*******************************************************************************
/* ProtocolIE ManagementBasedMDTAllowed ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_MANAGEMENTBASEDMDTALLOWED_ALLOWED,
  LIBLTE_X2AP_MANAGEMENTBASEDMDTALLOWED_N_ITEMS,
}LIBLTE_X2AP_MANAGEMENTBASEDMDTALLOWED_ENUM;
static const char liblte_x2ap_managementbasedmdtallowed_text[LIBLTE_X2AP_MANAGEMENTBASEDMDTALLOWED_N_ITEMS][80] = {
  "allowed",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MANAGEMENTBASEDMDTALLOWED_ENUM e;
}LIBLTE_X2AP_MANAGEMENTBASEDMDTALLOWED_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_managementbasedmdtallowed(
  LIBLTE_X2AP_MANAGEMENTBASEDMDTALLOWED_ENUM_EXT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_managementbasedmdtallowed(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MANAGEMENTBASEDMDTALLOWED_ENUM_EXT               *ie);

/*******************************************************************************
/* ProtocolIE Number_of_Antennaports ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_AN1,
  LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_AN2,
  LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_AN4,
  LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_N_ITEMS,
}LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_ENUM;
static const char liblte_x2ap_number_of_antennaports_text[LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_N_ITEMS][80] = {
  "an1",
  "an2",
  "an4",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_ENUM e;
}LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_number_of_antennaports(
  LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_ENUM_EXT                        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_number_of_antennaports(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_ENUM_EXT                        *ie);

/*******************************************************************************
/* ProtocolIE Pre_emptionCapability ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_PRE_EMPTIONCAPABILITY_SHALL_NOT_TRIGGER_PRE_EMPTION,
  LIBLTE_X2AP_PRE_EMPTIONCAPABILITY_MAY_TRIGGER_PRE_EMPTION,
  LIBLTE_X2AP_PRE_EMPTIONCAPABILITY_N_ITEMS,
}LIBLTE_X2AP_PRE_EMPTIONCAPABILITY_ENUM;
static const char liblte_x2ap_pre_emptioncapability_text[LIBLTE_X2AP_PRE_EMPTIONCAPABILITY_N_ITEMS][80] = {
  "shall-not-trigger-pre-emption",
  "may-trigger-pre-emption",
};

LIBLTE_ERROR_ENUM liblte_x2ap_pack_pre_emptioncapability(
  LIBLTE_X2AP_PRE_EMPTIONCAPABILITY_ENUM                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_pre_emptioncapability(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PRE_EMPTIONCAPABILITY_ENUM                       *ie);

/*******************************************************************************
/* ProtocolIE Pre_emptionVulnerability ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_PRE_EMPTIONVULNERABILITY_NOT_PRE_EMPTABLE,
  LIBLTE_X2AP_PRE_EMPTIONVULNERABILITY_PRE_EMPTABLE,
  LIBLTE_X2AP_PRE_EMPTIONVULNERABILITY_N_ITEMS,
}LIBLTE_X2AP_PRE_EMPTIONVULNERABILITY_ENUM;
static const char liblte_x2ap_pre_emptionvulnerability_text[LIBLTE_X2AP_PRE_EMPTIONVULNERABILITY_N_ITEMS][80] = {
  "not-pre-emptable",
  "pre-emptable",
};

LIBLTE_ERROR_ENUM liblte_x2ap_pack_pre_emptionvulnerability(
  LIBLTE_X2AP_PRE_EMPTIONVULNERABILITY_ENUM                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_pre_emptionvulnerability(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PRE_EMPTIONVULNERABILITY_ENUM                    *ie);

/*******************************************************************************
/* ProtocolIE RadioframeAllocationPeriod ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_N1,
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_N2,
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_N4,
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_N8,
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_N16,
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_N32,
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_N_ITEMS,
}LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_ENUM;
static const char liblte_x2ap_radioframeallocationperiod_text[LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_N_ITEMS][80] = {
  "n1",
  "n2",
  "n4",
  "n8",
  "n16",
  "n32",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_ENUM e;
}LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_radioframeallocationperiod(
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_ENUM                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_radioframeallocationperiod(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_ENUM                    *ie);

/*******************************************************************************
/* ProtocolIE Registration_Request ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_REGISTRATION_REQUEST_START,
  LIBLTE_X2AP_REGISTRATION_REQUESST_STOP,
  LIBLTE_X2AP_REGISTRATION_REQUEST_N_ITEMS,
}LIBLTE_X2AP_REGISTRATION_REQUEST_ENUM;
static const char liblte_x2ap_registration_request_text[LIBLTE_X2AP_REGISTRATION_REQUEST_N_ITEMS][80] = {
  "start",
  "stop",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_REGISTRATION_REQUEST_ENUM e;
}LIBLTE_X2AP_REGISTRATION_REQUEST_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_registration_request(
  LIBLTE_X2AP_REGISTRATION_REQUEST_ENUM_EXT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_registration_request(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_REGISTRATION_REQUEST_ENUM_EXT                              *ie);

/*******************************************************************************
/* ProtocolIE ReportAmountMDT ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_REPORTAMOUNTMDT_R1,
  LIBLTE_X2AP_REPORTAMOUNTMDT_R2,
  LIBLTE_X2AP_REPORTAMOUNTMDT_R4,
  LIBLTE_X2AP_REPORTAMOUNTMDT_R8,
  LIBLTE_X2AP_REPORTAMOUNTMDT_R16,
  LIBLTE_X2AP_REPORTAMOUNTMDT_R32,
  LIBLTE_X2AP_REPORTAMOUNTMDT_R64,
  LIBLTE_X2AP_REPORTAMOUNTMDT_RINFINITY,
  LIBLTE_X2AP_REPORTAMOUNTMDT_N_ITEMS,
}LIBLTE_X2AP_REPORTAMOUNTMDT_ENUM;
static const char liblte_x2ap_reportamountmdt_text[LIBLTE_X2AP_REPORTAMOUNTMDT_N_ITEMS][80] = {
  "r1",
  "r2",
  "r4",
  "r8",
  "r16",
  "r32",
  "r64",
  "rinfinity",
};

LIBLTE_ERROR_ENUM liblte_x2ap_pack_reportamountmdt(
  LIBLTE_X2AP_REPORTAMOUNTMDT_ENUM                             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_reportamountmdt(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_REPORTAMOUNTMDT_ENUM                             *ie);

/*******************************************************************************
/* ProtocolIE ReportArea ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_REPORTAREA_ECGI,
  LIBLTE_X2AP_REPORTAREA_N_ITEMS,
}LIBLTE_X2AP_REPORTAREA_ENUM;
static const char liblte_x2ap_reportarea_text[LIBLTE_X2AP_REPORTAREA_N_ITEMS][80] = {
  "ecgi",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_REPORTAREA_ENUM e;
}LIBLTE_X2AP_REPORTAREA_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_reportarea(
  LIBLTE_X2AP_REPORTAREA_ENUM_EXT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_reportarea(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_REPORTAREA_ENUM_EXT                              *ie);

/*******************************************************************************
/* ProtocolIE ReportIntervalMDT ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_REPORTINTERVALMDT_MS120,
  LIBLTE_X2AP_REPORTINTERVALMDT_MS240,
  LIBLTE_X2AP_REPORTINTERVALMDT_MS480,
  LIBLTE_X2AP_REPORTINTERVALMDT_MS640,
  LIBLTE_X2AP_REPORTINTERVALMDT_MS1024,
  LIBLTE_X2AP_REPORTINTERVALMDT_MS2048,
  LIBLTE_X2AP_REPORTINTERVALMDT_MS5120,
  LIBLTE_X2AP_REPORTINTERVALMDT_MS10240,
  LIBLTE_X2AP_REPORTINTERVALMDT_MIN1,
  LIBLTE_X2AP_REPORTINTERVALMDT_MIN6,
  LIBLTE_X2AP_REPORTINTERVALMDT_MIN12,
  LIBLTE_X2AP_REPORTINTERVALMDT_MIN30,
  LIBLTE_X2AP_REPORTINTERVALMDT_MIN60,
  LIBLTE_X2AP_REPORTINTERVALMDT_N_ITEMS,
}LIBLTE_X2AP_REPORTINTERVALMDT_ENUM;
static const char liblte_x2ap_reportintervalmdt_text[LIBLTE_X2AP_REPORTINTERVALMDT_N_ITEMS][80] = {
  "ms120",
  "ms240",
  "ms480",
  "ms640",
  "ms1024",
  "ms2048",
  "ms5120",
  "ms10240",
  "min1",
  "min6",
  "min12",
  "min30",
  "min60",
};

LIBLTE_ERROR_ENUM liblte_x2ap_pack_reportintervalmdt(
  LIBLTE_X2AP_REPORTINTERVALMDT_ENUM                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_reportintervalmdt(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_REPORTINTERVALMDT_ENUM                          *ie);


/*******************************************************************************
/* ProtocolIE RNTP_Threshold ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSINFINITY,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSELEVEN,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSTEN,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSNINE,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSEIGHT,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSSEVEN,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSSIX,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSFIVE,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSFOUR,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSTHREE,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSTWO,
  LIBLTE_X2AP_RNTP_THRESHOLD_MINUSONE,
  LIBLTE_X2AP_RNTP_THRESHOLD_ZERO,
  LIBLTE_X2AP_RNTP_THRESHOLD_ONE,
  LIBLTE_X2AP_RNTP_THRESHOLD_TWO,
  LIBLTE_X2AP_RNTP_THRESHOLD_THREE,
  LIBLTE_X2AP_RNTP_THRESHOLD_N_ITEMS,
}LIBLTE_X2AP_RNTP_THRESHOLD_ENUM;
static const char liblte_x2ap_rntp_threshold_text[LIBLTE_X2AP_RNTP_THRESHOLD_N_ITEMS][80] = {
  "minusInfinity",
  "minusEleven",
  "minusTen",
  "minusNine",
  "minusEight",
  "minusSeven",
  "minusSix",
  "minusFive",
  "minusFour",
  "minusThree",
  "minusTwo",
  "minusOne",
  "zero",
  "one",
  "two",
  "three",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_RNTP_THRESHOLD_ENUM e;
}LIBLTE_X2AP_RNTP_THRESHOLD_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_rntp_threshold(
  LIBLTE_X2AP_RNTP_THRESHOLD_ENUM                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_rntp_threshold(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_RNTP_THRESHOLD_ENUM                           *ie);


/*******************************************************************************
/* ProtocolIE RRCConnReestabIndicator ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_RRCCONNREESTABINDICATOR_RECONFIGURATIONFAILURE,
  LIBLTE_X2AP_RRCCONNREESTABINDICATOR_HANDOVERFAILURE, 
  LIBLTE_X2AP_RRCCONNREESTABINDICATOR_OTHERFAILURE, 
  LIBLTE_X2AP_RRCCONNREESTABINDICATOR_N_ITEMS,
}LIBLTE_X2AP_RRCCONNREESTABINDICATOR_ENUM;
static const char liblte_x2ap_rrcconnreestabindicator_text[LIBLTE_X2AP_RRCCONNREESTABINDICATOR_N_ITEMS][80] = {
  "reconfigurationFailure",
  "handoverFailure",
  "otherFailure",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_RRCCONNREESTABINDICATOR_ENUM e;
}LIBLTE_X2AP_RRCCONNREESTABINDICATOR_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_rrcconnreestabindicator(
  LIBLTE_X2AP_RRCCONNREESTABINDICATOR_ENUM_EXT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_rrcronnreestabindicator(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_RRCCONNREESTABINDICATOR_ENUM_EXT                 *ie);

/*******************************************************************************
/* ProtocolIE RRCConnSetupIndicator ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_RRCCONNSETUPINDICATOR_RRCCONNSETUP,
  LIBLTE_X2AP_RRCCONNSETUPINDICATOR_N_ITEMS,
}LIBLTE_X2AP_RRCCONNSETUPINDICATOR_ENUM;
static const char liblte_x2ap_rrcconnsetupindicator_text[LIBLTE_X2AP_RRCCONNSETUPINDICATOR_N_ITEMS][80] = {
  "rrcConnSetup",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_RRCCONNSETUPINDICATOR_ENUM e;
}LIBLTE_X2AP_RRCCONNSETUPINDICATOR_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_rrcconnsetupindicator(
  LIBLTE_X2AP_RRCCONNSETUPINDICATOR_ENUM_EXT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_rrcronnsetupindicator(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_RRCCONNSETUPINDICATOR_ENUM_EXT                 *ie);

/*******************************************************************************
/* ProtocolIE SRVCCOperationPossible ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_SRVCCOPERATIONPOSSIBLE_POSSIBLE,
  LIBLTE_X2AP_SRVCCOPERATIONPOSSIBLE_N_ITEMS,
}LIBLTE_X2AP_SRVCCOPERATIONPOSSIBLE_ENUM;
static const char liblte_x2ap_srvccoperationpossible_text[LIBLTE_X2AP_SRVCCOPERATIONPOSSIBLE_N_ITEMS][80] = {
  "possible",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_SRVCCOPERATIONPOSSIBLE_ENUM e;
}LIBLTE_X2AP_SRVCCOPERATIONPOSSIBLE_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_srvccoperationpossible(
  LIBLTE_X2AP_SRVCCOPERATIONPOSSIBLE_ENUM_EXT                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_srvccoperationpossible(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SRVCCOPERATIONPOSSIBLE_ENUM_EXT                  *ie);


/*******************************************************************************
/* ProtocolIE SubframeAssignment ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_SA0,
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_SA1,
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_SA2,
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_SA3,
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_SA4,
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_SA5,
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_SA6,
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_N_ITEMS,
}LIBLTE_X2AP_SUBFRAMEASSIGNMENT_ENUM;
static const char liblte_x2ap_subframeassignment_text[LIBLTE_X2AP_SUBFRAMEASSIGNMENT_N_ITEMS][80] = {
  "sa0",
  "s21",
  "sa2",
  "sa3",
  "sa4",
  "sa5",
  "sa6",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_ENUM e;
}LIBLTE_X2AP_SUBFRAMEASSIGNMENT_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_subframeassignment(
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_ENUM_EXT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_subframeassignment(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_ENUM_EXT                  *ie);

/*******************************************************************************
/* ProtocolIE SpecialSubframePatterns ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_SSP0,
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_SSP1,
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_SSP2,
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_SSP3,
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_SSP4,
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_SSP5,
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_SSP6,
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_SSP7,
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_SSP8,
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_N_ITEMS,
}LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_ENUM;
static const char liblte_x2ap_specialsubframepatterns_text[LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_N_ITEMS][80] = {
  "ssp0",
  "ssp1",
  "ssp2",
  "ssp3",
  "ssp4",
  "ssp5",
  "ssp6",
  "ssp7",
  "ssp8",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_ENUM e;
}LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_specialsubframepatterns(
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_ENUM_EXT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_specialsubframepatterns(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_ENUM_EXT                  *ie);

/*******************************************************************************
/* ProtocolIE TimeToWait ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_TIMETOWAIT_V1S,
  LIBLTE_X2AP_TIMETOWAIT_V2S,
  LIBLTE_X2AP_TIMETOWAIT_V5S,
  LIBLTE_X2AP_TIMETOWAIT_V10S,
  LIBLTE_X2AP_TIMETOWAIT_V20S,
  LIBLTE_X2AP_TIMETOWAIT_V60S,
  LIBLTE_X2AP_TIMETOWAIT_N_ITEMS,
}LIBLTE_X2AP_TIMETOWAIT_ENUM;
static const char liblte_x2ap_timetowait_text[LIBLTE_X2AP_TIMETOWAIT_N_ITEMS][80] = {
  "v1s",
  "v2s",
  "v5s",
  "v10s",
  "v20s",
  "v60s",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TIMETOWAIT_ENUM e;
}LIBLTE_X2AP_TIMETOWAIT_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_timetowait(
  LIBLTE_X2AP_TIMETOWAIT_ENUM_EXT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_timetowait(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TIMETOWAIT_ENUM_EXT                              *ie);

/*******************************************************************************
/* ProtocolIE TraceDepth ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_TRACEDEPTH_MINIMUM,
  LIBLTE_X2AP_TRACEDEPTH_MEDIUM,
  LIBLTE_X2AP_TRACEDEPTH_MAXIMUM,
  LIBLTE_X2AP_TRACEDEPTH_MINIMUMWITHOUTVENDORSPECIFICEXTENSION,
  LIBLTE_X2AP_TRACEDEPTH_MEDIUMWITHOUTVENDORSPECIFICEXTENSION,
  LIBLTE_X2AP_TRACEDEPTH_MAXIMUMWITHOUTVENDORSPECIFICEXTENSION,
  LIBLTE_X2AP_TRACEDEPTH_N_ITEMS,
}LIBLTE_X2AP_TRACEDEPTH_ENUM;
static const char liblte_x2ap_tracedepth_text[LIBLTE_X2AP_TRACEDEPTH_N_ITEMS][80] = {
  "minimum",
  "medium",
  "maximum",
  "minimumWithoutVendorSpecificExtension",
  "mediumWithoutVendorSpecificExtension",
  "maximumWithoutVendorSpecificExtension",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TRACEDEPTH_ENUM e;
}LIBLTE_X2AP_TRACEDEPTH_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tracedepth(
  LIBLTE_X2AP_TRACEDEPTH_ENUM_EXT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tracedepth(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TRACEDEPTH_ENUM_EXT                              *ie);

/*******************************************************************************
/* ProtocolIE Transmission_Bandwidth ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_BW6,
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_BW15,
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_BW25,
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_BW50,
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_BW75,
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_BW100,
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_N_ITEMS,
}LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_ENUM;
static const char liblte_x2ap_transmission_bandwidth_text[LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_N_ITEMS][80] = {
  "bw6",
  "bw15",
  "bw25",
  "bw50",
  "bw75",
  "bw100",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_ENUM e;
}LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_transmission_bandwidth(
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_ENUM_EXT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_transmission_bandwidth(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_ENUM_EXT                              *ie);


/*******************************************************************************
/* ProtocolIE TypeOfError ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_TYPEOFERROR_NOT_UNDERSTOOD,
  LIBLTE_X2AP_TYPEOFERROR_MISSING,
  LIBLTE_X2AP_TYPEOFERROR_N_ITEMS,
}LIBLTE_X2AP_TYPEOFERROR_ENUM;
static const char liblte_x2ap_typeoferror_text[LIBLTE_X2AP_TYPEOFERROR_N_ITEMS][80] = {
  "not-understood",
  "missing",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TYPEOFERROR_ENUM e;
}LIBLTE_X2AP_TYPEOFERROR_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_typeoferror(
  LIBLTE_X2AP_TYPEOFERROR_ENUM_EXT                             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_typeoferror(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TYPEOFERROR_ENUM_EXT                             *ie);

/*******************************************************************************
/* ProtocolIE UL_InterferenceOverloadIndication_Item ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_HIGH_INTERFERENCE,
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_MEDIUM_INTERFERENCE,
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_LOW_INTERFERENCE,
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_N_ITEMS,
}LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_ENUM;
static const char liblte_x2ap_ul_interferenceoverloadindication_item_text[LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_N_ITEMS][80] = {
  "high-interference",
  "medium_interference",
  "low_interference",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_ENUM e;
}LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ul_interferenceoverloadindication_item(
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_ENUM_EXT                     *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ul_interferenceoverloadindication_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_ENUM_EXT                     *ie);

/* SEQUENCE & CHOICE */

/*******************************************************************************
/* ProtocolIE ABSInformationFDD SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  uint8_t abs_pattern_info[40]; // BIT STRING
  enum numberofCellSpecificAntennaPorts {one, two, four};
  uint8_t measurement_subset[40]; // BIT STRING
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT iE_Extensions;
  bool iE_Extensions_present;
}LIBLTE_X2AP_ABSINFORMATIONFDD_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_absinformationfdd(
  LIBLTE_X2AP_ABSINFORMATIONFDD_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_absinformationfdd(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ABSINFORMATIONFDD_STRUCT            *ie);

/*******************************************************************************
/* Protocol Message ABSInformationFDD_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_ABSINFORMATIONFDD_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_absinformationfdd_ext(
  LIBLTE_X2AP_MESSAGE_ABSINFORMATIONFDD_EXT_STRUCT *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_absinformationfdd_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ABSINFORMATIONFDD_EXT_STRUCT *ie);

/*******************************************************************************
/* ProtocolIE ABSInformationTDD SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  uint8_t abs_pattern_info[70]; // BIT STRING
  enum numberofCellSpecificAntennaPorts {one, two, four};
  uint8_t measurement_subset[70]; // BIT STRING
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT iE_Extensions;
  bool iE_Extensions_present;
}LIBLTE_X2AP_ABSINFORMATIONTDD_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_absinformationtdd(
  LIBLTE_X2AP_ABSINFORMATIONTDD_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_absinformationtdd(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ABSINFORMATIONTDD_STRUCT            *ie);

/*******************************************************************************
/* Protocol Message ABSInformationTDD_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_ABSINFORMATIONTDD_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_absinformationtdd_ext(
  LIBLTE_X2AP_MESSAGE_ABSINFORMATIONTDD_EXT_STRUCT *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_absinformationtdd_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ABSINFORMATIONTDD_EXT_STRUCT *ie);

/*******************************************************************************
/* ProtocolIE ABSInformation CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_ABSINFORMATION_CHOICE_FDD,
  LIBLTE_X2AP_ABSINFORMATION_CHOICE_TDD,
  LIBLTE_X2AP_ABSINFORMATION_CHOICE_ABS_INACTIVE,
  LIBLTE_X2AP_ABSINFORMATION_CHOICE_N_ITEMS,
}LIBLTE_X2AP_ABSINFORMATION_CHOICE_ENUM;
static const char liblte_x2ap_absinformation_choice_text[LIBLTE_X2AP_ABSINFORMATION_CHOICE_N_ITEMS][50] = {
  "fdd",
  "tdd",
  "abs_inactive",
};

typedef union{
  LIBLTE_X2AP_ABSINFORMATIONFDD_STRUCT                         fdd;
  LIBLTE_X2AP_ABSINFORMATIONTDD_STRUCT                         tdd;
  //TODO: NULL                                                 abs_inactive;
}LIBLTE_X2AP_ABSINFORMATION_CHOICE_UNION;

typedef struct{
  bool                            ext;
  LIBLTE_X2AP_ABSINFORMATION_CHOICE_UNION choice;
  LIBLTE_X2AP_ABSINFORMATION_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_ABSINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_absinformation(
  LIBLTE_X2AP_ABSINFORMATION_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_absinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ABSINFORMATION_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE UsableABSInformationFDD SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  uint8_t usable_abs_pattern_info[40];
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_USABLEABSINFORMATIONFDD_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_usableabsinformationfdd(
  LIBLTE_X2AP_USABLEABSINFORMATIONFDD_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_usableabsinformationfdd(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_USABLEABSINFORMATIONFDD_STRUCT                               *ie);

/*******************************************************************************
/* Protocol Message UsableABSInformationFDD_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_USABLEABSINFORMATIONFDD_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_usableabsinformationfdd_ext(
  LIBLTE_X2AP_MESSAGE_USABLEABSINFORMATIONFDD_EXT_STRUCT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_usableabsinformationfdd_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_USABLEABSINFORMATIONFDD_EXT_STRUCT                   *ie);

/*******************************************************************************
/* ProtocolIE UsableABSInformationTDD SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  uint8_t usable_abs_pattern_info[70];
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_USABLEABSINFORMATIONTDD_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_usableabsinformationtdd(
  LIBLTE_X2AP_USABLEABSINFORMATIONTDD_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_usableabsinformationtdd(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_USABLEABSINFORMATIONTDD_STRUCT                               *ie);

/*******************************************************************************
/* Protocol Message UsableABSInformationTDD_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_USABLEABSINFORMATIONTDD_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_usableabsinformationtdd_ext(
  LIBLTE_X2AP_MESSAGE_USABLEABSINFORMATIONTDD_EXT_STRUCT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_usableabsinformationtdd_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_USABLEABSINFORMATIONTDD_EXT_STRUCT                   *ie);

/*******************************************************************************
/* ProtocolIE UsableABSInformation CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_USABLEABSINFORMATION_CHOICE_FDD,
  LIBLTE_X2AP_USABLEABSINFORMATION_CHOICE_TDD,
  LIBLTE_X2AP_USABLEABSINFORMATION_CHOICE_N_ITEMS,
}LIBLTE_X2AP_USABLEABSINFORMATION_CHOICE_ENUM;
static const char liblte_x2ap_overloadresponse_choice_text[LIBLTE_X2AP_USABLEABSINFORMATION_CHOICE_N_ITEMS][50] = {
  "fdd",
  "tdd",
};

typedef union{
  LIBLTE_X2AP_USABLEABSINFORMATIONFDD_STRUCT          fdd;
  LIBLTE_X2AP_USABLEABSINFORMATIONTDD_STRUCT          tdd;
}LIBLTE_X2AP_USABLEABSINFORMATION_CHOICE_UNION;

typedef struct{
  bool                            ext;
  LIBLTE_X2AP_USABLEABSINFORMATION_CHOICE_UNION choice;
  LIBLTE_X2AP_USABLEABSINFORMATION_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_USABLEABSINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_usableabsinformation(
  LIBLTE_X2AP_USABLEABSINFORMATION_STRUCT                          *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_usableabsinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_USABLEABSINFORMATION_STRUCT                          *ie);

/*******************************************************************************
/* ProtocolIE ABS_Status SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_DL_ABS_STATUS_STRUCT dL_ABS_status;
  LIBLTE_X2AP_USABLEABSINFORMATION_STRUCT usableABSInformation;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT iE_Extensions;
  bool iE_Extensions_present;
}LIBLTE_X2AP_ABS_STATUS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_abs_status(
  LIBLTE_X2AP_ABS_STATUS_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_abs_status(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ABS_STATUS_STRUCT            *ie);

/*******************************************************************************
/* Protocol Message ABS_Status_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_ABS_STATUS_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_abs_status_ext(
  LIBLTE_X2AP_MESSAGE_ABS_STATUS_EXT_STRUCT *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_abs_status_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ABS_STATUS_EXT_STRUCT *ie);

/*******************************************************************************
/* ProtocolIE AdditionalSpecialSubframe_Info SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAMEPATTERNS_ENUM_EXT additionalspecialSubframePatterns;
  LIBLTE_X2AP_CYCLICPREFIXDL_ENUM_EXT cyclicPrefixDL;
  LIBLTE_X2AP_CYCLICPREFIXUL_ENUM_EXT cyclicPrefixUL;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT iE_Extensions;
  bool iE_Extensions_present;
}LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAME_INFO_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_additionalspecialsubframe_info(
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAME_INFO_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_additionalspecialsubframe_info(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAME_INFO_STRUCT            *ie);

/*******************************************************************************
/* Protocol Message AdditionalSpecialSubframe_Info_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_ADDITIONALSPECIALSUBFRAME_INFO_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_additionalspecialsubframe_info_ext(
  LIBLTE_X2AP_MESSAGE_ADDITIONALSPECIALSUBFRAME_INFO_EXT_STRUCT *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_additionalspecialsubframe_info_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ADDITIONALSPECIALSUBFRAME_INFO_EXT_STRUCT *ie);

/*******************************************************************************
/* ProtocolIE AS_SecurityInformation SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_KEY_ENODEB_STAR_STRUCT key_eNodeB_star;
  LIBLTE_X2AP_NEXTHOPCHAININGCOUNT_STRUCT nextHopChainingCount;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT iE_Extensions;
  bool iE_Extensions_present;
}LIBLTE_X2AP_AS_SECURITYINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_as_securityinformation(
  LIBLTE_X2AP_AS_SECURITYINFORMATION_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_as_securityinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_AS_SECURITYINFORMATION_STRUCT            *ie);

/*******************************************************************************
/* Protocol Message AS_SecurityInformation_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_AS_SECURITYINFORMATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_as_securityinformation_ext(
  LIBLTE_X2AP_MESSAGE_AS_SECURITYINFORMATION_EXT_STRUCT *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_as_securityinformation_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_AS_SECURITYINFORMATION_EXT_STRUCT *ie);

/*******************************************************************************
/* ProtocolIE AllocationAndRetentionPriority SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_PRIORITYLEVEL_STRUCT priorityLevel;
  LIBLTE_X2AP_PRE_EMPTIONCAPABILITY_ENUM pre_emptioncapability;
  LIBLTE_X2AP_PRE_EMPTIONVULNERABILITY_ENUM pre_emptionVulnerability;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT iE_Extensions;
  bool iE_Extensions_present;
}LIBLTE_X2AP_ALLOCATIONANDRETENTIONPRIORITY_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_allocationandretentionpriority(
  LIBLTE_X2AP_ALLOCATIONANDRETENTIONPRIORITY_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_allocationandretentionpriority(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ALLOCATIONANDRETENTIONPRIORITY_STRUCT            *ie);

/*******************************************************************************
/* Protocol Message AllocationAndRetentionPriority_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_ALLOCATIONANDRETENTIONPRIORITY_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_allocationandretentionpriority_ext(
  LIBLTE_X2AP_MESSAGE_ALLOCATIONANDRETENTIONPRIORITY_EXT_STRUCT *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_allocationandretentionpriority_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ALLOCATIONANDRETENTIONPRIORITY_EXT_STRUCT *ie);

/*******************************************************************************
/* ProtocolIE BroadcastPLMNs_Item DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:6
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_TBCD_STRING_STRUCT                               buffer[6];
}LIBLTE_X2AP_BROADCASTPLMNS_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_broadcastplmns_item(
  LIBLTE_X2AP_BROADCASTPLMNS_ITEM_STRUCT                                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_broadcastplmns_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_BROADCASTPLMNS_ITEM_STRUCT                                    *ie);

/*******************************************************************************
/* ProtocolIE Cause CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_CAUSE_CHOICE_RADIONETWORK,
  LIBLTE_X2AP_CAUSE_CHOICE_TRANSPORT,
  LIBLTE_X2AP_CAUSE_CHOICE_PROTOCOL,
  LIBLTE_X2AP_CAUSE_CHOICE_MISC,
  LIBLTE_X2AP_CAUSE_CHOICE_N_ITEMS,
}LIBLTE_X2AP_CAUSE_CHOICE_ENUM;
static const char liblte_x2ap_cause_choice_text[LIBLTE_X2AP_CAUSE_CHOICE_N_ITEMS][50] = {
  "radioNetwork",
  "transport",
  "protocol",
  "misc",
};

typedef union{
  LIBLTE_X2AP_CAUSERADIONETWORK_ENUM_EXT                       radioNetwork;
  LIBLTE_X2AP_CAUSETRANSPORT_ENUM_EXT                          transport;
  LIBLTE_X2AP_CAUSEPROTOCOL_ENUM_EXT                           protocol;
  LIBLTE_X2AP_CAUSEMISC_ENUM_EXT                               misc;
}LIBLTE_X2AP_CAUSE_CHOICE_UNION;

typedef struct{
  bool                            ext;
  LIBLTE_X2AP_CAUSE_CHOICE_UNION choice;
  LIBLTE_X2AP_CAUSE_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_CAUSE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cause(
  LIBLTE_X2AP_CAUSE_STRUCT                                     *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cause(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CAUSE_STRUCT                                     *ie);

/*******************************************************************************
/* ProtocolIE ECGI SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TBCD_STRING_STRUCT                               pLMN_Identity;
  LIBLTE_X2AP_EUTRANCELLIDENTIFIER_STRUCT                   eUTRANcellIdentifier;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_ECGI_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ecgi(
  LIBLTE_X2AP_ECGI_STRUCT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ecgi(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ECGI_STRUCT                                *ie);

/*******************************************************************************
/* Protocol Message ECGI_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_ECGI_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ecgi_ext(
  LIBLTE_X2AP_MESSAGE_ECGI_EXT_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ecgi_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ECGI_EXT_STRUCT                    *ie);

/*******************************************************************************
/* ProtocolIE CellIdListforMDT DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:32
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_ECGI_STRUCT                                buffer[32];
}LIBLTE_X2AP_CELLIDLISTFORMDT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellidlistformdt(
  LIBLTE_X2AP_CELLIDLISTFORMDT_STRUCT                          *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellidlistformdt(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELLIDLISTFORMDT_STRUCT                          *ie);

/*******************************************************************************
/* ProtocolIE CellBasedMDT SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CELLIDLISTFORMDT_STRUCT cellIdListforMDT;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT iE_Extensions;
  bool iE_Extensions_present;
}LIBLTE_X2AP_CELLBASEDMDT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellbasedmdt(
  LIBLTE_X2AP_CELLBASEDMDT_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellbasedmdt(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELLBASEDMDT_STRUCT            *ie);

/*******************************************************************************
/* Protocol Message CellBasedMDT_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_CELLBASEDMDT_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellbasedmdt_ext(
  LIBLTE_X2AP_MESSAGE_CELLBASEDMDT_EXT_STRUCT *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellbasedmdt_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLBASEDMDT_EXT_STRUCT *ie);

/*******************************************************************************
/* ProtocolIE CellType SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CELL_SIZE_ENUM_EXT                               cell_Size;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_CELLTYPE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_celltype(
  LIBLTE_X2AP_CELLTYPE_STRUCT                                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_celltype(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELLTYPE_STRUCT                                  *ie);

/*******************************************************************************
/* Protocol Message CellType_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_CELLTYPE_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_celltype_ext(
  LIBLTE_X2AP_MESSAGE_CELLTYPE_EXT_STRUCT                      *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_celltype_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLTYPE_EXT_STRUCT                      *ie);

/*******************************************************************************
/* ProtocolIE CompositeAvailableCapacity SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CELLCAPACITYCLASSVALUE_STRUCT           cellCapacityClassValue;
  LIBLTE_X2AP_CAPACITYVALUE_STRUCT                    capacityValue;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_COMPOSITEAVAILABLECAPACITY_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_compositeavailablecapacity(
  LIBLTE_X2AP_COMPOSITEAVAILABLECAPACITY_STRUCT                                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_compositeavailablecapacity(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_COMPOSITEAVAILABLECAPACITY_STRUCT                                  *ie);

/*******************************************************************************
/* Protocol Message CompositeAvailableCapasity_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_COMPOSITEAVAILABLECAPACITY_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_compositeavailablecapacity_ext(
  LIBLTE_X2AP_MESSAGE_COMPOSITEAVAILABLECAPACITY_EXT_STRUCT                      *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_compositeavailablecapacity_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_COMPOSITEAVAILABLECAPACITY_EXT_STRUCT                      *ie);

/*******************************************************************************
/* ProtocolIE CompositeAvailableCapacityGroup SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_COMPOSITEAVAILABLECAPACITY_STRUCT           dL_CompositeAvailableCapacity;
  LIBLTE_X2AP_COMPOSITEAVAILABLECAPACITY_STRUCT           uL_CompositeAvailableCapacity;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_COMPOSITEAVAILABLECAPACITYGROUP_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_compositeavailablecapacitygroup(
  LIBLTE_X2AP_COMPOSITEAVAILABLECAPACITYGROUP_STRUCT                                  *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_compositeavailablecapacitygroup(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_COMPOSITEAVAILABLECAPACITYGROUP_STRUCT                                  *ie);

/*******************************************************************************
/* Protocol Message CompositeAvailableCapacityGroup_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_COMPOSITEAVAILABLECAPACITYGROUP_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_compositeavailablecapacitygroup_ext(
  LIBLTE_X2AP_MESSAGE_COMPOSITEAVAILABLECAPACITYGROUP_EXT_STRUCT                      *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_compositeavailablecapacitygroup_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_COMPOSITEAVAILABLECAPACITYGROUP_EXT_STRUCT                      *ie);

/*******************************************************************************
/* ProtocolIE COUNTvalue SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_PDCP_SN_STRUCT                                   pDCP_SN;
  LIBLTE_X2AP_HFN_STRUCT                                       hFN;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_COUNTVALUE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_countvalue(
  LIBLTE_X2AP_COUNTVALUE_STRUCT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_countvalue(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_COUNTVALUE_STRUCT                                *ie);

/*******************************************************************************
/* Protocol Message COUNTvalue_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_COUNTVALUE_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_countvalue_ext(
  LIBLTE_X2AP_MESSAGE_COUNTVALUE_EXT_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_countvalue_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_COUNTVALUE_EXT_STRUCT                    *ie);

/*******************************************************************************
/* ProtocolIE COUNTValueExtended SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_PDCP_SNEXTENDED_STRUCT                           pDCP_SNExtended;
  LIBLTE_X2AP_HFNMODIFIED_STRUCT                               hFNModified;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_COUNTVALUEEXTENDED_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_countvalueextended(
  LIBLTE_X2AP_COUNTVALUEEXTENDED_STRUCT                        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_countvalueextended(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_COUNTVALUEEXTENDED_STRUCT                        *ie);

/*******************************************************************************
/* Protocol Message COUNTValueExtended_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_COUNTVALUEEXTENDED_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_countvalueextended_ext(
  LIBLTE_X2AP_MESSAGE_COUNTVALUEEXTENDED_EXT_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_countvalueextended_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_COUNTVALUEEXTENDED_EXT_STRUCT            *ie);

/*******************************************************************************
/* ProtocolIE CriticalityDiagnostics_IE_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CRITICALITY_ENUM                                 iECriticality;
  LIBLTE_X2AP_PROTOCOLIE_ID_STRUCT                             iE_ID;
  LIBLTE_X2AP_TYPEOFERROR_ENUM_EXT                             typeOfError;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_IE_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_criticalitydiagnostics_ie_item(
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_IE_ITEM_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_criticalitydiagnostics_ie_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_IE_ITEM_STRUCT            *ie);

/*******************************************************************************
/* ProtocolIE CriticalityDiagnostics_IE_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_IE_ITEM_STRUCT            buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_IE_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_criticalitydiagnostics_ie_list(
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_IE_LIST_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_criticalitydiagnostics_ie_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_IE_LIST_STRUCT            *ie);

/*******************************************************************************
/* Protocol Message CriticalityDiagnostics_IE_List_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_CRITICALITYDIAGNOSTICS_IE_LIST_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_criticalitydiagnostics_ie_list_ext(
  LIBLTE_X2AP_MESSAGE_CRITICALITYDIAGNOSTICS_IE_LIST_EXT_STRUCT *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_criticalitydiagnostics_ie_list_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CRITICALITYDIAGNOSTICS_IE_LIST_EXT_STRUCT *ie);

/*******************************************************************************
/* ProtocolIE CriticalityDiagnostics SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_PROCEDURECODE_STRUCT                             procedureCode;
  bool                                                         procedureCode_present;
  LIBLTE_X2AP_TRIGGERINGMESSAGE_ENUM                           triggeringMessage;
  bool                                                         triggeringMessage_present;
  LIBLTE_X2AP_CRITICALITY_ENUM                                 procedureCriticality;
  bool                                                         procedureCriticality_present;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_IE_LIST_STRUCT            iEsCriticalityDiagnostics;
  bool                                                         iEsCriticalityDiagnostics_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_criticalitydiagnostics(
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_criticalitydiagnostics(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    *ie);

/*******************************************************************************
/* Protocol Message CriticalityDiagnostics_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_CRITICALITYDIAGNOSTICS_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_criticalitydiagnostics_ext(
  LIBLTE_X2AP_MESSAGE_CRITICALITYDIAGNOSTICS_EXT_STRUCT        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_criticalitydiagnostics_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CRITICALITYDIAGNOSTICS_EXT_STRUCT        *ie);

/*******************************************************************************
/* ProtocolIE SpecialSubframe_Info SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_SPECIALSUBFRAMEPATTERNS_ENUM_EXT                 specialSubframePatterns;
  LIBLTE_X2AP_CYCLICPREFIXDL_ENUM_EXT                          cyclicPrefixDL;
  LIBLTE_X2AP_CYCLICPREFIXUL_ENUM_EXT                          cyclicPrefixUL;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_SPECIALSUBFRAME_INFO_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_specialsubframe_info(
  LIBLTE_X2AP_SPECIALSUBFRAME_INFO_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_specialsubframe_info(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SPECIALSUBFRAME_INFO_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message SpecialSubframe_Info_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_SPECIALSUBFRAME_INFO_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_specialsubframe_info_ext(
  LIBLTE_X2AP_MESSAGE_SPECIALSUBFRAME_INFO_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_specialsubframe_info_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_SPECIALSUBFRAME_INFO_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE FDD_Info SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_EARFCN_STRUCT                              uL_EARFCN;
  LIBLTE_X2AP_EARFCN_STRUCT                              dL_EARFCN;
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_ENUM_EXT            uL_Transmission_Bandwidth;
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_ENUM_EXT            dL_Transmission_Bandwidth;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_FDD_INFO_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_fdd_info(
  LIBLTE_X2AP_FDD_INFO_STRUCT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_fdd_info(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_FDD_INFO_STRUCT                                *ie);

/*******************************************************************************
/* Protocol Message FDD_Info_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_EARFCNEXTENSION_STRUCT                        UL_EARFCNExtension;
  bool                                                      UL_EARFCNExtension_present;
  LIBLTE_X2AP_EARFCNEXTENSION_STRUCT                        DL_EARFCNExtension;
  bool                                                      DL_EARFCNExtension_present;
}LIBLTE_X2AP_MESSAGE_FDD_INFO_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_fdd_info_ext(
  LIBLTE_X2AP_MESSAGE_FDD_INFO_EXT_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_fdd_info_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_FDD_INFO_EXT_STRUCT                    *ie);

/*******************************************************************************
/* ProtocolIE TDD_Info SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_EARFCN_STRUCT                              eARFCN;
  LIBLTE_X2AP_TRANSMISSION_BANDWIDTH_ENUM_EXT              transmission_Bandwidth;
  LIBLTE_X2AP_SUBFRAMEASSIGNMENT_ENUM_EXT                 subframeAssignment;
  LIBLTE_X2AP_SPECIALSUBFRAME_INFO_STRUCT                specialSubframe_Info;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_TDD_INFO_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tdd_info(
  LIBLTE_X2AP_TDD_INFO_STRUCT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tdd_info(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TDD_INFO_STRUCT                                *ie);

/*******************************************************************************
/* Protocol Message TDD_Info_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ADDITIONALSPECIALSUBFRAME_INFO_STRUCT         AdditionalSpecialSubframe_Info;
  bool                                             AdditionalSpecialSubframe_present;
  LIBLTE_X2AP_EARFCNEXTENSION_STRUCT                        eARFCNExtension;
  bool                                                      eARFCNExtension_present;
}LIBLTE_X2AP_MESSAGE_TDD_INFO_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tdd_info_ext(
  LIBLTE_X2AP_MESSAGE_TDD_INFO_EXT_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tdd_info_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_TDD_INFO_EXT_STRUCT                    *ie);

/*******************************************************************************
/* ProtocolIE EUTRA_Mode_Info CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_EUTRA_MODE_INFO_FDD,
  LIBLTE_X2AP_EUTRA_MODE_INFO_TDD,
  LIBLTE_X2AP_EUTRA_MODE_INFO_N_ITEMS,
}LIBLTE_X2AP_EUTRA_MODE_INFO_CHOICE_ENUM;
static const char liblte_x2ap_eutra_mode_info_choice_text[LIBLTE_X2AP_EUTRA_MODE_INFO_N_ITEMS][50] = {
  "fDD",
  "tDD",
};

typedef union{
  LIBLTE_X2AP_FDD_INFO_STRUCT                                fDD;
  LIBLTE_X2AP_TDD_INFO_STRUCT                                tDD;
}LIBLTE_X2AP_EUTRA_MODE_INFO_CHOICE_UNION;

typedef struct{
  bool                            ext;
  LIBLTE_X2AP_EUTRA_MODE_INFO_CHOICE_UNION choice;
  LIBLTE_X2AP_EUTRA_MODE_INFO_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_EUTRA_MODE_INFO_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_eutra_mode_info(
  LIBLTE_X2AP_EUTRA_MODE_INFO_STRUCT                                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_eutra_mode_info(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_EUTRA_MODE_INFO_STRUCT                                    *ie);

/*******************************************************************************
/* ProtocolIE ENB_ID CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_ENB_ID_CHOICE_MACROENB_ID,
  LIBLTE_X2AP_ENB_ID_CHOICE_HOMEENB_ID,
  LIBLTE_X2AP_ENB_ID_CHOICE_N_ITEMS,
}LIBLTE_X2AP_ENB_ID_CHOICE_ENUM;
static const char liblte_x2ap_enb_id_choice_text[LIBLTE_X2AP_ENB_ID_CHOICE_N_ITEMS][50] = {
  "macroENB_ID",
  "homeENB_ID",
};

typedef union{
  LIBLTE_X2AP_MACROENB_ID_STRUCT                               macroENB_ID;
  LIBLTE_X2AP_HOMEENB_ID_STRUCT                                homeENB_ID;
}LIBLTE_X2AP_ENB_ID_CHOICE_UNION;

typedef struct{
  bool                            ext;
  LIBLTE_X2AP_ENB_ID_CHOICE_UNION choice;
  LIBLTE_X2AP_ENB_ID_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_ENB_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_enb_id(
  LIBLTE_X2AP_ENB_ID_STRUCT                                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_enb_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ENB_ID_STRUCT                                    *ie);

/*******************************************************************************
/* ProtocolIE EPLMNs DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:15
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_TBCD_STRING_STRUCT                               buffer[15]; 
}LIBLTE_X2AP_EPLMNS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_eplmns(
  LIBLTE_X2AP_EPLMNS_STRUCT                                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_eplmns(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_EPLMNS_STRUCT                                    *ie);

/*******************************************************************************
/* ProtocolIE GBR_QosInformation SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_BITRATE_STRUCT                                   e_RAB_MaximumBitrateDL;
  LIBLTE_X2AP_BITRATE_STRUCT                                   e_RAB_MaximumBitrateUL;
  LIBLTE_X2AP_BITRATE_STRUCT                                   e_RAB_GuaranteedBitrateDL;
  LIBLTE_X2AP_BITRATE_STRUCT                                   e_RAB_GuaranteedBitrateUL;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_GBR_QOSINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_gbr_qosinformation(
  LIBLTE_X2AP_GBR_QOSINFORMATION_STRUCT                        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_gbr_qosinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_GBR_QOSINFORMATION_STRUCT                        *ie);

/*******************************************************************************
/* Protocol Message GBR_QosInformation_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_GBR_QOSINFORMATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_gbr_qosinformation_ext(
  LIBLTE_X2AP_MESSAGE_GBR_QOSINFORMATION_EXT_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_gbr_qosinformation_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_GBR_QOSINFORMATION_EXT_STRUCT            *ie);

/*******************************************************************************
/* ProtocolIE E_RAB_Level_QoS_Parameters SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_QCI_STRUCT                                       qCI;
  LIBLTE_X2AP_ALLOCATIONANDRETENTIONPRIORITY_STRUCT            allocationRetentionPriority;
  LIBLTE_X2AP_GBR_QOSINFORMATION_STRUCT                        gbrQosInformation;
  bool                                                         gbrQosInformation_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_E_RAB_LEVEL_QOS_PARAMETERS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rab_level_qos_parameters(
  LIBLTE_X2AP_E_RAB_LEVEL_QOS_PARAMETERS_STRUCT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rab_level_qos_parameters(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_E_RAB_LEVEL_QOS_PARAMETERS_STRUCT                   *ie);

/*******************************************************************************
/* Protocol Message E_RAB_Level_QoS_Parameters_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_E_RAB_LEVEL_QOS_PARAMETERS_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rab_level_qos_parameters_ext(
  LIBLTE_X2AP_MESSAGE_E_RAB_LEVEL_QOS_PARAMETERS_EXT_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rab_level_qos_parameters_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_E_RAB_LEVEL_QOS_PARAMETERS_EXT_STRUCT            *ie);

/*******************************************************************************
/* ProtocolIE E_RAB_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_E_RAB_ID_STRUCT                                  e_RAB_ID;
  LIBLTE_X2AP_CAUSE_STRUCT                                     cause;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_E_RAB_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rab_item(
  LIBLTE_X2AP_E_RAB_ITEM_STRUCT                                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rab_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_E_RAB_ITEM_STRUCT                                 *ie);

/*******************************************************************************
/* Protocol Message E_RAB_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_E_RAB_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rab_item_ext(
  LIBLTE_X2AP_MESSAGE_E_RAB_ITEM_EXT_STRUCT                     *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rab_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_E_RAB_ITEM_EXT_STRUCT                     *ie);

/*******************************************************************************
/* Protocol Container List E_RAB_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_E_RAB_ITEM_STRUCT                                 buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_E_RAB_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rab_list(
  LIBLTE_X2AP_E_RAB_LIST_STRUCT                                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rab_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_E_RAB_LIST_STRUCT                                 *ie);

/*******************************************************************************
/* ProtocolIE ForbiddenTACs DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:4096
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_TAC_STRUCT                                       buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_FORBIDDENTACS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_forbiddentacs(
  LIBLTE_X2AP_FORBIDDENTACS_STRUCT                             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_forbiddentacs(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_FORBIDDENTACS_STRUCT                             *ie);

/*******************************************************************************
/* ProtocolIE ForbiddenTAs_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TBCD_STRING_STRUCT                               pLMN_Identity;
  LIBLTE_X2AP_FORBIDDENTACS_STRUCT                             forbiddenTACs;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_FORBIDDENTAS_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_forbiddentas_item(
  LIBLTE_X2AP_FORBIDDENTAS_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_forbiddentas_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_FORBIDDENTAS_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message ForbiddenTAs_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_FORBIDDENTAS_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_forbiddentas_item_ext(
  LIBLTE_X2AP_MESSAGE_FORBIDDENTAS_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_forbiddentas_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_FORBIDDENTAS_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* ProtocolIE ForbiddenTAs DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:16
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_FORBIDDENTAS_ITEM_STRUCT                         buffer[16]; 
}LIBLTE_X2AP_FORBIDDENTAS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_forbiddentas(
  LIBLTE_X2AP_FORBIDDENTAS_STRUCT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_forbiddentas(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_FORBIDDENTAS_STRUCT                              *ie);

/*******************************************************************************
/* ProtocolIE ForbiddenLACs DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:4096
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_LAC_STRUCT                                       buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_FORBIDDENLACS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_forbiddenlacs(
  LIBLTE_X2AP_FORBIDDENLACS_STRUCT                             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_forbiddenlacs(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_FORBIDDENLACS_STRUCT                             *ie);

/*******************************************************************************
/* ProtocolIE ForbiddenLAs_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TBCD_STRING_STRUCT                               pLMN_Identity;
  LIBLTE_X2AP_FORBIDDENLACS_STRUCT                             forbiddenLACs;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_FORBIDDENLAS_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_forbiddenlas_item(
  LIBLTE_X2AP_FORBIDDENLAS_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_forbiddenlas_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_FORBIDDENLAS_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message ForbiddenLAs_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_FORBIDDENLAS_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_forbiddenlas_item_ext(
  LIBLTE_X2AP_MESSAGE_FORBIDDENLAS_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_forbiddenlas_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_FORBIDDENLAS_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* ProtocolIE ForbiddenLAs DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:16
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_FORBIDDENLAS_ITEM_STRUCT                         buffer[16]; 
}LIBLTE_X2AP_FORBIDDENLAS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_forbiddenlas(
  LIBLTE_X2AP_FORBIDDENLAS_STRUCT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_forbiddenlas(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_FORBIDDENLAS_STRUCT                              *ie);

/*******************************************************************************
/* ProtocolIE GlobalENB_ID SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TBCD_STRING_STRUCT                   pLMN_Identity;
  LIBLTE_X2AP_ENB_ID_STRUCT                        eNB_ID;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_GLOBALENB_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_globalenb_id(
  LIBLTE_X2AP_GLOBALENB_ID_STRUCT                        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_globalenb_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_GLOBALENB_ID_STRUCT                        *ie);

/*******************************************************************************
/* Protocol Message GlobalENB_ID_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_GLOBALENB_ID_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_globalenb_id_ext(
  LIBLTE_X2AP_MESSAGE_GLOBALENB_ID_EXT_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_globalenb_id_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_GLOBALENB_ID_EXT_STRUCT            *ie);

/*******************************************************************************
/* ProtocolIE GTPtunnelEndpoint SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TRANSPORTLAYERADDRESS_STRUCT             transportLayerAddress;
  LIBLTE_X2AP_GTP_TEI_STRUCT                           gTP_TEID;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_GTPTUNNELENDPOINT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_gtptunnelendpoint(
  LIBLTE_X2AP_GTPTUNNELENDPOINT_STRUCT                        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_gtptunnelendpoint(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_GTPTUNNELENDPOINT_STRUCT                        *ie);

/*******************************************************************************
/* Protocol Message GTPtunnelEndpoint_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_GTPTUNNELENDPOINT_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_gtptunnelendpoint_ext(
  LIBLTE_X2AP_MESSAGE_GTPTUNNELENDPOINT_EXT_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_gtptunnelendpoint_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_GTPTUNNELENDPOINT_EXT_STRUCT            *ie);

/*******************************************************************************
/* ProtocolIE GU_Group_ID SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TBCD_STRING_STRUCT              pLMN_Identity;
  LIBLTE_X2AP_MME_GROUP_ID_STRUCT             mME_Group_ID;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_GU_GROUP_ID_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_gu_group_id(
  LIBLTE_X2AP_GU_GROUP_ID_STRUCT                        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_gu_group_id(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_GU_GROUP_ID_STRUCT                        *ie);

/*******************************************************************************
/* Protocol Message GU_Group_ID_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_GU_GROUP_ID_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_gu_group_id_ext(
  LIBLTE_X2AP_MESSAGE_GU_GROUP_ID_EXT_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_gu_group_id_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_GU_GROUP_ID_EXT_STRUCT            *ie);

/*******************************************************************************
/* ProtocolIE GUGroupIDList DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:16
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_GU_GROUP_ID_STRUCT                               buffer[16];
}LIBLTE_X2AP_GUGROUPIDLIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_gugroupidlist(
  LIBLTE_X2AP_GUGROUPIDLIST_STRUCT                          *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_gugroupidlist(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_GUGROUPIDLIST_STRUCT                          *ie);

/*******************************************************************************
/* ProtocolIE GUMMEI SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_GU_GROUP_ID_STRUCT                              gU_Group_ID;
  LIBLTE_X2AP_MME_CODE_STRUCT                                  mME_Code;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_GUMMEI_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_gummei(
  LIBLTE_X2AP_GUMMEI_STRUCT                                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_gummei(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_GUMMEI_STRUCT                                    *ie);

/*******************************************************************************
/* Protocol Message GUMMEI_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_GUMMEI_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_gummei_ext(
  LIBLTE_X2AP_MESSAGE_GUMMEI_EXT_STRUCT                        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_gummei_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_GUMMEI_EXT_STRUCT                        *ie);

/*******************************************************************************
/* ProtocolIE HandoverRestrictionList SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TBCD_STRING_STRUCT                               servingPLMN;
  LIBLTE_X2AP_EPLMNS_STRUCT                                    equivalentPLMNs;
  bool                                                         equivalentPLMNs_present;
  LIBLTE_X2AP_FORBIDDENTAS_STRUCT                              forbiddenTAs;
  bool                                                         forbiddenTAs_present;
  LIBLTE_X2AP_FORBIDDENLAS_STRUCT                              forbiddenLAs;
  bool                                                         forbiddenLAs_present;
  LIBLTE_X2AP_FORBIDDENINTERRATS_ENUM_EXT                      forbiddenInterRATs;
  bool                                                         forbiddenInterRATs_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_HANDOVERRESTRICTIONLIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_handoverrestrictionlist(
  LIBLTE_X2AP_HANDOVERRESTRICTIONLIST_STRUCT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handoverrestrictionlist(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_HANDOVERRESTRICTIONLIST_STRUCT                   *ie);

/*******************************************************************************
/* Protocol Message HandoverRestrictionList_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_HANDOVERRESTRICTIONLIST_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_handoverrestrictionlist_ext(
  LIBLTE_X2AP_MESSAGE_HANDOVERRESTRICTIONLIST_EXT_STRUCT       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handoverrestrictionlist_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_HANDOVERRESTRICTIONLIST_EXT_STRUCT       *ie);

/*******************************************************************************
/* ProtocolIE HWLoadIndicator SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_LOADINDICATOR_ENUM_EXT                             dLHWLoadIndicator;
  LIBLTE_X2AP_LOADINDICATOR_ENUM_EXT                             uLHWLoadIndicator;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_HWLOADINDICATOR_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_hwloadindicator(
  LIBLTE_X2AP_HWLOADINDICATOR_STRUCT                                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_hwloadindicator(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_HWLOADINDICATOR_STRUCT                                    *ie);

/*******************************************************************************
/* Protocol Message HWLoadIndicator_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_HWLOADINDICATOR_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_hwloadindicator_ext(
  LIBLTE_X2AP_MESSAGE_HWLOADINDICATOR_EXT_STRUCT                        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_hwloadindicator_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_HWLOADINDICATOR_EXT_STRUCT                        *ie);

/*******************************************************************************
/* ProtocolIE LastVisitedEUTRANCellInformation SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                global_Cell_ID;
  LIBLTE_X2AP_CELLTYPE_STRUCT                                  cellType;
  LIBLTE_X2AP_TIME_UE_STAYEDINCELL_STRUCT                      time_UE_StayedInCell;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_LASTVISITEDEUTRANCELLINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_lastvisitedeutrancellinformation(
  LIBLTE_X2AP_LASTVISITEDEUTRANCELLINFORMATION_STRUCT          *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_lastvisitedeutrancellinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_LASTVISITEDEUTRANCELLINFORMATION_STRUCT          *ie);

/*******************************************************************************
/* Protocol Message LastVisitedEUTRANCellInformation_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TIME_UE_STAYEDINCELL_ENHANCEDGRANULARITY_STRUCT  Time_UE_StayedInCell_EnhancedGranularity;
  bool                                                         Time_UE_StayedInCell_EnhancedGranularity_present;
  LIBLTE_X2AP_CAUSE_STRUCT                                     HO_Cause;
  bool                                                         HO_Cause_present;
}LIBLTE_X2AP_MESSAGE_LASTVISITEDEUTRANCELLINFORMATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_lastvisitedeutrancellinformation_ext(
  LIBLTE_X2AP_MESSAGE_LASTVISITEDEUTRANCELLINFORMATION_EXT_STRUCT *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_lastvisitedeutrancellinformation_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_LASTVISITEDEUTRANCELLINFORMATION_EXT_STRUCT *ie);

/*******************************************************************************
/* ProtocolIE LastVisitedGERANCellInformation CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_CHOICE_UNDEFINED,
  LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_CHOICE_N_ITEMS,
}LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_CHOICE_ENUM;
static const char liblte_x2ap_lastvisitedgerancellinformation_choice_text[LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_CHOICE_N_ITEMS][50] = {
  "undefined",
};

typedef union{
  //TODO: NULL                                                 undefined;
}LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_CHOICE_UNION;

typedef struct{
  bool                            ext;
  LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_CHOICE_UNION choice;
  LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_lastvisitedgerancellinformation(
  LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_STRUCT           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_lastvisitedgerancellinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_STRUCT           *ie);

/*******************************************************************************
/* ProtocolIE LastVisitedCell_Item CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_LASTVISITEDCELL_ITEM_CHOICE_E_UTRAN_CELL,
  LIBLTE_X2AP_LASTVISITEDCELL_ITEM_CHOICE_UTRAN_CELL,
  LIBLTE_X2AP_LASTVISITEDCELL_ITEM_CHOICE_GERAN_CELL,
  LIBLTE_X2AP_LASTVISITEDCELL_ITEM_CHOICE_N_ITEMS,
}LIBLTE_X2AP_LASTVISITEDCELL_ITEM_CHOICE_ENUM;
static const char liblte_x2ap_lastvisitedcell_item_choice_text[LIBLTE_X2AP_LASTVISITEDCELL_ITEM_CHOICE_N_ITEMS][50] = {
  "e_UTRAN_Cell",
  "uTRAN_Cell",
  "gERAN_Cell",
};

typedef union{
  LIBLTE_X2AP_LASTVISITEDEUTRANCELLINFORMATION_STRUCT          e_UTRAN_Cell;
  LIBLTE_X2AP_LASTVISITEDUTRANCELLINFORMATION_STRUCT           uTRAN_Cell;
  LIBLTE_X2AP_LASTVISITEDGERANCELLINFORMATION_STRUCT           gERAN_Cell;
}LIBLTE_X2AP_LASTVISITEDCELL_ITEM_CHOICE_UNION;

typedef struct{
  bool                            ext;
  LIBLTE_X2AP_LASTVISITEDCELL_ITEM_CHOICE_UNION choice;
  LIBLTE_X2AP_LASTVISITEDCELL_ITEM_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_LASTVISITEDCELL_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_lastvisitedcell_item(
  LIBLTE_X2AP_LASTVISITEDCELL_ITEM_STRUCT                      *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_lastvisitedcell_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_LASTVISITEDCELL_ITEM_STRUCT                      *ie);

/*******************************************************************************
/* ProtocolIE LocationReportingInformatin SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_EVENTTYPE_ENUM_EXT                         eventType;
  LIBLTE_X2AP_REPORTAREA_ENUM_EXT                          reportArea;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_LOCATIONREPORTINGINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_locationreportinginformation(
  LIBLTE_X2AP_LOCATIONREPORTINGINFORMATION_STRUCT          *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_locationreportinginformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_LOCATIONREPORTINGINFORMATION_STRUCT          *ie);

/*******************************************************************************
/* Protocol Message LocationReportingInformation_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_LOCATIONREPORTINGINFORMATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_locationreportinginformation_ext(
  LIBLTE_X2AP_MESSAGE_LOCATIONREPORTINGINFORMATION_EXT_STRUCT *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_locationreportinginformation_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_LOCATIONREPORTINGINFORMATION_EXT_STRUCT *ie);

/*******************************************************************************
/* ProtocolIE MeasurementThresholdA2 CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_CHOICE_THRESHOLD_RSRP,
  LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_CHOICE_THRESHOLD_RSRQ,
  LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_CHOICE_N_ITEMS,
}LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_CHOICE_ENUM;
static const char liblte_x2ap_measurementthresholda2_choice_text[LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_CHOICE_N_ITEMS][50] = {
  "threshold_RSRP",
  "threshold_RSRQ",
};

typedef union{
  LIBLTE_X2AP_THRESHOLD_RSRP_STRUCT                            threshold_RSRP;
  LIBLTE_X2AP_THRESHOLD_RSRQ_STRUCT                            threshold_RSRQ;
}LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_CHOICE_UNION;

typedef struct{
  bool                            ext;
  LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_CHOICE_UNION choice;
  LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurementthresholda2(
  LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurementthresholda2(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_STRUCT                    *ie);

/*******************************************************************************
/* ProtocolIE M1ThresholdEventA2 SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MEASUREMENTTHRESHOLDA2_STRUCT                    measurementThreshold;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_M1THRESHOLDEVENTA2_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m1thresholdeventa2(
  LIBLTE_X2AP_M1THRESHOLDEVENTA2_STRUCT                        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m1thresholdeventa2(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_M1THRESHOLDEVENTA2_STRUCT                        *ie);

/*******************************************************************************
/* Protocol Message M1ThresholdEventA2_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_M1THRESHOLDEVENTA2_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m1thresholdeventa2_ext(
  LIBLTE_X2AP_MESSAGE_M1THRESHOLDEVENTA2_EXT_STRUCT            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m1thresholdeventa2_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_M1THRESHOLDEVENTA2_EXT_STRUCT            *ie);

/*******************************************************************************
/* ProtocolIE M1PeriodicReporting SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_REPORTINTERVALMDT_ENUM                           reportInterval;
  LIBLTE_X2AP_REPORTAMOUNTMDT_ENUM                             reportAmount;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_M1PERIODICREPORTING_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m1periodicreporting(
  LIBLTE_X2AP_M1PERIODICREPORTING_STRUCT                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m1periodicreporting(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_M1PERIODICREPORTING_STRUCT                       *ie);

/*******************************************************************************
/* Protocol Message M1PeriodicReporting_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_M1PERIODICREPORTING_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m1periodicreporting_ext(
  LIBLTE_X2AP_MESSAGE_M1PERIODICREPORTING_EXT_STRUCT           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m1periodicreporting_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_M1PERIODICREPORTING_EXT_STRUCT           *ie);

/*******************************************************************************
/* ProtocolIE M3Configuration SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_M3PERIOD_ENUM_EXT                                m3period;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_M3CONFIGURATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m3configuration(
  LIBLTE_X2AP_M3CONFIGURATION_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m3configuration(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_M3CONFIGURATION_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message M3Configuration_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_M3CONFIGURATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m3configuration_ext(
  LIBLTE_X2AP_MESSAGE_M3CONFIGURATION_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m3configuration_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_M3CONFIGURATION_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE M4Configuration SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_M4PERIOD_ENUM_EXT                                m4period;
  LIBLTE_X2AP_LINKS_TO_LOG_ENUM_EXT                            m4_links_to_log;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_M4CONFIGURATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m4configuration(
  LIBLTE_X2AP_M4CONFIGURATION_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m4configuration(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_M4CONFIGURATION_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message M4Configuration_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_M4CONFIGURATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m4configuration_ext(
  LIBLTE_X2AP_MESSAGE_M4CONFIGURATION_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m4configuration_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_M4CONFIGURATION_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE M5Configuration SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_M5PERIOD_ENUM_EXT                                m5period;
  LIBLTE_X2AP_LINKS_TO_LOG_ENUM_EXT                            m5_links_to_log;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_M5CONFIGURATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m5configuration(
  LIBLTE_X2AP_M5CONFIGURATION_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m5configuration(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_M5CONFIGURATION_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message M5Configuration_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_M5CONFIGURATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_m5configuration_ext(
  LIBLTE_X2AP_MESSAGE_M5CONFIGURATION_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_m5configuration_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_M5CONFIGURATION_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE MDTPLMNList DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:16
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_TBCD_STRING_STRUCT                               buffer[16];
}LIBLTE_X2AP_MDTPLMNLIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mdtplmnlist(
  LIBLTE_X2AP_MDTPLMNLIST_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mdtplmnlist(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MDTPLMNLIST_STRUCT                               *ie);

/*******************************************************************************
/* ProtocolIE MBMS_Service_Area_Identity_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:8
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_MBMS_SERVICE_AREA_IDENTITY_STRUCT           buffer[8]; 
}LIBLTE_X2AP_MBMS_SERVICE_AREA_IDENTITY_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mbms_service_area_identity_list(
  LIBLTE_X2AP_MBMS_SERVICE_AREA_IDENTITY_LIST_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mbms_service_area_identity_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MBMS_SERVICE_AREA_IDENTITY_LIST_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE SubframeAllocation CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_SUBFRAMEALLOCATION_CHOICE_ONEFRAME,
  LIBLTE_X2AP_SUBFRAMEALLOCATION_CHOICE_FOURFRAMES,
  LIBLTE_X2AP_SUBFRAMEALLOCATION_CHOICE_N_ITEMS,
}LIBLTE_X2AP_SUBFRAMEALLOCATION_CHOICE_ENUM;
static const char liblte_x2ap_subframeallocation_choice_text[LIBLTE_X2AP_SUBFRAMEALLOCATION_CHOICE_N_ITEMS][50] = {
  "oneframe",
  "fourframes",
};

typedef union{
  LIBLTE_X2AP_ONEFRAME_STRUCT                          oneframe;
  LIBLTE_X2AP_FOURFRAMES_STRUCT                        fourframes;
}LIBLTE_X2AP_SUBFRAMEALLOCATION_CHOICE_UNION;

typedef struct{
  bool                            ext;
  LIBLTE_X2AP_SUBFRAMEALLOCATION_CHOICE_UNION choice;
  LIBLTE_X2AP_SUBFRAMEALLOCATION_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_SUBFRAMEALLOCATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_subframeallocation(
  LIBLTE_X2AP_SUBFRAMEALLOCATION_STRUCT                          *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_subframeallocation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SUBFRAMEALLOCATION_STRUCT                          *ie);

/*******************************************************************************
/* ProtocolIE MBSFN_Subframe_Info SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONPERIOD_ENUM_EXT              radioframeAllocationPeriod;
  LIBLTE_X2AP_RADIOFRAMEALLOCATIONOFFSET_STRUCT                radioframeAllocationOffset;
  LIBLTE_X2AP_SUBFRAMEALLOCATION_STRUCT                        subframeAllocation;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_MBSFN_SUBFRAME_INFO_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mbsfn_subframe_info(
  LIBLTE_X2AP_MBSFN_SUBFRAME_INFO_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mbsfn_subframe_info(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MBSFN_SUBFRAME_INFO_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message MBSFN_Subframe_Info_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_MBSFN_SUBFRAME_INFO_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mbsfn_subframe_info_ext(
  LIBLTE_X2AP_MESSAGE_MBSFN_SUBFRAME_INFO_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mbsfn_subframe_info_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_MBSFN_SUBFRAME_INFO_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE MBSFN_Subframe_Infolist DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:8
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_MBSFN_SUBFRAME_INFO_STRUCT           buffer[8]; 
}LIBLTE_X2AP_MBSFN_SUBFRAME_INFOLIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mbsfn_subframe_infolist(
  LIBLTE_X2AP_MBSFN_SUBFRAME_INFOLIST_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mbsfn_subframe_infolist(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MBSFN_SUBFRAME_INFOLIST_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE MobilityParametersModificationRange SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  int32_t                                 handoverTriggerChangeLowerLimit;
  int32_t                                 handoverTriggerChangeUpperLimit;
}LIBLTE_X2AP_MOBILITYPARAMETERSMODIFICATIONRANGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mobilityparametersmodificationrange_info(
  LIBLTE_X2AP_MOBILITYPARAMETERSMODIFICATIONRANGE_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mobilityparametersmodificationrange_info(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MOBILITYPARAMETERSMODIFICATIONRANGE_STRUCT                           *ie);

/*******************************************************************************
/* ProtocolIE MobilityParametersInformation SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  int32_t                                    handoverTriggerChange;
}LIBLTE_X2AP_MOBILITYPARAMETERSINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mobilityparametersinformation_info(
  LIBLTE_X2AP_MOBILITYPARAMETERSINFORMATION_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mobilityparametersinformation_info(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MOBILITYPARAMETERSINFORMATION_STRUCT                           *ie);

/*******************************************************************************
/* ProtocolIE BandInfo SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_FREQBANDINDICATOR_STRUCT                         freqBandIndicator;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_BANDINFO_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_bandinfo(
  LIBLTE_X2AP_BANDINFO_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_bandinfo(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_BANDINFO_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message BandInfo_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_BANDINFO_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_bandinfo_ext(
  LIBLTE_X2AP_MESSAGE_BANDINFO_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_bandinfo_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_BANDINFO_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE MultibandInfolist DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:16
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_BANDINFO_STRUCT           buffer[16]; 
}LIBLTE_X2AP_MULTIBANDINFOLIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_multibandinfolist(
  LIBLTE_X2AP_MULTIBANDINFOLIST_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_multibandinfolist(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MULTIBANDINFOLIST_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE Neighbour_Information_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      eCGI;
  LIBLTE_X2AP_PCI_STRUCT                                       pCI;
  LIBLTE_X2AP_EARFCN_STRUCT                                    eARFCN;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_NEIGHBOUR_INFORMATION_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_neighbour_information_item(
  LIBLTE_X2AP_NEIGHBOUR_INFORMATION_ITEM_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_heighbour_information_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_NEIGHBOUR_INFORMATION_ITEM_STRUCT                           *ie);

/*******************************************************************************
/* ProtocolIE Neighour_Information DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:51
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_NEIGHBOUR_INFORMATION_ITEM_STRUCT           buffer[51]; 
}LIBLTE_X2AP_NEIGHBOUR_INFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_neighbour_information(
  LIBLTE_X2AP_NEIGHBOUR_INFORMATION_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_neighbour_information(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_NEIGHBOUR_INFORMATION_STRUCT                            *ie);

/*******************************************************************************
/* Protocol Message Neighbour_Information_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TAC_STRUCT                                 NeighbourTAC;
  bool                                                   NeighbourTAC_present;
  LIBLTE_X2AP_EARFCNEXTENSION_STRUCT                     eARFCNExtension;
  bool                                                   eARFCNExtension_present;
}LIBLTE_X2AP_MESSAGE_NEIGHBOUR_INFORMATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_neighbour_information_ext(
  LIBLTE_X2AP_MESSAGE_NEIGHBOUR_INFORMATION_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_neighbour_information_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_NEIGHBOUR_INFORMATION_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE PRACH_Configuration SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  uint32_t                                                     rootSequenceIndex;
  uint32_t                                                     zeroCorrelationIndex;
  bool                                                         highSpeedFlag;
  uint32_t                                                     prach_FreqOffset;
  uint32_t                                                     prach_ConfigIndex;
  bool                                                         prach_ConfigIndex_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_PRACH_CONFIGURATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_prach_configuration(
  LIBLTE_X2AP_PRACH_CONFIGURATION_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_prach_configuration(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PRACH_CONFIGURATION_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message PRACH_Configuration_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_PRACH_CONFIGURATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_prach_configuration_ext(
  LIBLTE_X2AP_MESSAGE_PRACH_CONFIGURATION_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_prach_configuration_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_PRACH_CONFIGURATION_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE RadioResourceStatus SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_DL_GBR_PRB_USAGE_STRUCT                          dL_GBR_PRB_usage;
  LIBLTE_X2AP_UL_GBR_PRB_USAGE_STRUCT                          uL_GBR_PRB_usage;
  LIBLTE_X2AP_DL_NON_GBR_PRB_USAGE_STRUCT                      dL_non_GBR_PRB_usage;
  LIBLTE_X2AP_UL_NON_GBR_PRB_USAGE_STRUCT                      uL_non_GBR_PRB_usage;
  LIBLTE_X2AP_DL_TOTAL_PRB_USAGE_STRUCT                        dL_Total_PRB_usage;
  LIBLTE_X2AP_UL_TOTAL_PRB_USAGE_STRUCT                        uL_Total_PRB_usage;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_RADIORESOURCESTATUS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_radioresourcestatus(
  LIBLTE_X2AP_RADIORESOURCESTATUS_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_radioresourcestatus(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_RADIORESOURCESTATUS_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message RadioResourceStatus_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_RADIORESOURCESTATUS_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_radioresourcestatus_ext(
  LIBLTE_X2AP_MESSAGE_RADIORESOURCESTATUS_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_radioresourcestatus_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_RADIORESOURCESTATUS_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE RelativeNarrowbandTxPower SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  uint8_t rNTP_PerPRB[110];
  LIBLTE_X2AP_RNTP_THRESHOLD_ENUM_EXT                          rNTP_Threshold;
  enum numberOfCellSpecificAntennaPorts {one, two, four};
  uint32_t p_B;
  uint32_t pDCCH_InterferenceImpact;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_RELATIVENARROWBANDTXPOWER_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_relativenarrowbandtxpower(
  LIBLTE_X2AP_RELATIVENARROWBANDTXPOWER_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_relativenarrowbandtxpower(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_RELATIVENARROWBANDTXPOWER_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message RelativeNarrowbandTxPower_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_RELATIVENARROWBANDTXPOWER_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_relativenarrowbandtxpower_ext(
  LIBLTE_X2AP_MESSAGE_RELATIVENARROWBANDTXPOWER_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_relativenarrowbandtxpower_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_RELATIVENARROWBANDTXPOWER_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE S1TNLLoadIndicator SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_LOADINDICATOR_ENUM_EXT                           dLS1TNLLoadIndicator;
  LIBLTE_X2AP_LOADINDICATOR_ENUM_EXT                           uLS1TNLLoadIndicator;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_S1TNLLOADINDICATOR_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_s1tnlloadindicator(
  LIBLTE_X2AP_S1TNLLOADINDICATOR_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_s1tnlloadindicator(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_S1TNLLOADINDICATOR_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message S1TNLLoadIndicator_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_S1TNLLOADINDICATOR_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_s1tnlloadindicator_ext(
  LIBLTE_X2AP_MESSAGE_S1TNLLOADINDICATOR_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_s1tnlloadindicator_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_S1TNLLOADINDICATOR_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE ServedCell_Information SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_PCI_STRUCT                                       pCI;
  LIBLTE_X2AP_ECGI_STRUCT                                      cellId;
  LIBLTE_X2AP_TAC_STRUCT                                       tAC;
  LIBLTE_X2AP_BROADCASTPLMNS_ITEM_STRUCT                       broadcastPLMNS;
  LIBLTE_X2AP_EUTRA_MODE_INFO_STRUCT                           eUTRA_Mode_Info;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_SERVEDCELL_INFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedCell(
  LIBLTE_X2AP_SERVEDCELL_INFORMATION_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcell(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SERVEDCELL_INFORMATION_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message ServedCell_Information_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_NUMBER_OF_ANTENNAPORTS_ENUM_EXT                  number_of_antennaports;
  bool                                                         number_of_antennaports_present;
  LIBLTE_X2AP_PRACH_CONFIGURATION_STRUCT                       PRACH_Configuration;
  bool                                                         PRACH_Configuration_present;
  LIBLTE_X2AP_MBSFN_SUBFRAME_INFOLIST_STRUCT                   MBSFN_Subframe_Info;
  bool                                                         MBSFN_Subframe_Info_present;
  LIBLTE_X2AP_CSG_ID_STRUCT                                    CSG_Id;
  bool                                                         CSG_Id_presnet;
  LIBLTE_X2AP_MBMS_SERVICE_AREA_IDENTITY_LIST_STRUCT           MBMS_Service_Area_List;
  bool                                                         MBMS_Service_Area_List_present;
  LIBLTE_X2AP_MULTIBANDINFOLIST_STRUCT                         MultibandInfoList;
  bool                                                         MultibandInfolist_present;
}LIBLTE_X2AP_MESSAGE_SERVEDCELL_INFORMATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedcell_information_ext(
  LIBLTE_X2AP_MESSAGE_SERVEDCELL_INFORMATION_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcell_information_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_SERVEDCELL_INFORMATION_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE ServedCell SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_SERVEDCELL_INFORMATION_STRUCT                    servedCellInfo;
  LIBLTE_X2AP_NEIGHBOUR_INFORMATION_STRUCT                     neighbour_Info;
  bool                                                         neighbour_Info_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_SERVEDCELL_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedCell(
  LIBLTE_X2AP_SERVEDCELL_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcell(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SERVEDCELL_STRUCT                           *ie);

/*******************************************************************************
/* ProtocolIE ServedCells DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_SERVEDCELL_STRUCT                          buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_SERVEDCELLS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedcells(
  LIBLTE_X2AP_SERVEDCELLS_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcells(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SERVEDCELLS_STRUCT                            *ie);

/*******************************************************************************
/* Protocol Message ServedCell_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_SERVEDCELL_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedcell_ext(
  LIBLTE_X2AP_MESSAGE_SERVEDCELL_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcell_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_SERVEDCELL_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE TAListforMDT DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:8
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_TAC_STRUCT                                       buffer[8]; 
}LIBLTE_X2AP_TALISTFORMDT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_talistformdt(
  LIBLTE_X2AP_TALISTFORMDT_STRUCT                             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_talistformdt(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TALISTFORMDT_STRUCT                             *ie);

/*******************************************************************************
/* ProtocolIE TAI_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TBCD_STRING_STRUCT                               pLMNidentity;
  LIBLTE_X2AP_TAC_STRUCT                                       tAC;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_TAI_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tai_item(
  LIBLTE_X2AP_TAI_ITEM_STRUCT                                       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tai_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TAI_ITEM_STRUCT                                       *ie);

/*******************************************************************************
/* Protocol Message TAI_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_TAI_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tai_item_ext(
  LIBLTE_X2AP_MESSAGE_TAI_ITEM_EXT_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tai_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_TAI_ITEM_EXT_STRUCT                           *ie);

/*******************************************************************************
/* ProtocolIE TAIListforMDT DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:8
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_TAI_ITEM_STRUCT                                       buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_TAILISTFORMDT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tailistformdt(
  LIBLTE_X2AP_TAILISTFORMDT_STRUCT                             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tailistformdt(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TAILISTFORMDT_STRUCT                             *ie);

/*******************************************************************************
/* ProtocolIE TAIBasedMDT SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TAILISTFORMDT_STRUCT                             tAIListforMDT;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_TAIBASEDMDT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_taibasedmdt(
  LIBLTE_X2AP_TAIBASEDMDT_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_taibasedmdt(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TAIBASEDMDT_STRUCT                               *ie);

/*******************************************************************************
/* Protocol Message TAIBasedMDT_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_TAIBASEDMDT_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_taibasedmdt_ext(
  LIBLTE_X2AP_MESSAGE_TAIBASEDMDT_EXT_STRUCT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_taibasedmdt_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_TAIBASEDMDT_EXT_STRUCT                   *ie);

/*******************************************************************************
/* ProtocolIE TABasedMDT SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_TALISTFORMDT_STRUCT                              tAListforMDT;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_TABASEDMDT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tabasedmdt(
  LIBLTE_X2AP_TABASEDMDT_STRUCT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tabasedmdt(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TABASEDMDT_STRUCT                                *ie);

/*******************************************************************************
/* Protocol Message TABasedMDT_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_TABASEDMDT_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_tabasedmdt_ext(
  LIBLTE_X2AP_MESSAGE_TABASEDMDT_EXT_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_tabasedmdt_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_TABASEDMDT_EXT_STRUCT                    *ie);

/*******************************************************************************
/* ProtocolIE AreaScopeOfMDT CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_AREASCOPEOFMDT_CHOICE_CELLBASED,
  LIBLTE_X2AP_AREASCOPEOFMDT_CHOICE_TABASED,
  LIBLTE_X2AP_AREASCOPEOFMDT_CHOICE_PLMNWIDE,
  LIBLTE_X2AP_AREASCOPEOFMDT_CHOICE_TAIBASED,
  LIBLTE_X2AP_AREASCOPEOFMDT_CHOICE_N_ITEMS,
}LIBLTE_X2AP_AREASCOPEOFMDT_CHOICE_ENUM;
static const char liblte_x2ap_areascopeofmdt_choice_text[LIBLTE_X2AP_AREASCOPEOFMDT_CHOICE_N_ITEMS][50] = {
  "cellBased",
  "tABased",
  "pLMNWide",
  "tAIBased",
};

typedef union{
  LIBLTE_X2AP_CELLBASEDMDT_STRUCT                              cellBased;
  LIBLTE_X2AP_TABASEDMDT_STRUCT                                tABased;
  //TODO: NULL                                                 pLMNWide;
  LIBLTE_X2AP_TAIBASEDMDT_STRUCT                               tAIBased;
}LIBLTE_X2AP_AREASCOPEOFMDT_CHOICE_UNION;

typedef struct{
  bool                            ext;
  LIBLTE_X2AP_AREASCOPEOFMDT_CHOICE_UNION choice;
  LIBLTE_X2AP_AREASCOPEOFMDT_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_AREASCOPEOFMDT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_areascopeofmdt(
  LIBLTE_X2AP_AREASCOPEOFMDT_STRUCT                            *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_areascopeofmdt(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_AREASCOPEOFMDT_STRUCT                            *ie);

/*******************************************************************************
/* ProtocolIE MDT_Configuration SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MDT_ACTIVATION_ENUM_EXT                          mdt_Activation;
  LIBLTE_X2AP_AREASCOPEOFMDT_STRUCT                            areaScopeOfMDT;
  LIBLTE_X2AP_MEASUREMENTSTOACTIVATE_STRUCT                    measurementsToActivate;
  LIBLTE_X2AP_M1REPORTINGTRIGGER_ENUM_EXT                      m1reportingTrigger;
  LIBLTE_X2AP_M1THRESHOLDEVENTA2_STRUCT                        m1thresholdeventA2;
  bool                                                         m1thresholdeventA2_present;
  LIBLTE_X2AP_M1PERIODICREPORTING_STRUCT                       m1periodicReporting;
  bool                                                         m1periodicReporting_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_MDT_CONFIGURATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mdt_configuration(
  LIBLTE_X2AP_MDT_CONFIGURATION_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mdt_configuration(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MDT_CONFIGURATION_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message MDT_Configuration_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_M3CONFIGURATION_STRUCT                           M3Configuration;
  bool                                                         M3Configuration_present;
  LIBLTE_X2AP_M4CONFIGURATION_STRUCT                           M4Configuration;
  bool                                                         M4Configuration_present;
  LIBLTE_X2AP_M5CONFIGURATION_STRUCT                           M5Configuration;
  bool                                                         M5Configuration_present;
  LIBLTE_X2AP_MDT_LOCATION_INFO_STRUCT                         MDT_Location_Info;
  bool                                                         MDT_Location_Info_present;
  LIBLTE_X2AP_MDTPLMNLIST_STRUCT                               SignallingBasedMDTPLMNList;
  bool                                                         SignallingBasedMDTPLMNList_present;
}LIBLTE_X2AP_MESSAGE_MDT_CONFIGURATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mdt_configuration_ext(
  LIBLTE_X2AP_MESSAGE_MDT_CONFIGURATION_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mdt_configuration_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_MDT_CONFIGURATION_EXT_STRUCT             *ie);

/*******************************************************************************
/* ProtocolIE TraceActivation SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_EUTRANTRACEID_STRUCT                             eUTRANTraceID;
  LIBLTE_X2AP_INTERFACESTOTRACE_STRUCT                         interfacesToTrace;
  LIBLTE_X2AP_TRACEDEPTH_ENUM_EXT                              traceDepth;
  LIBLTE_X2AP_TRANSPORTLAYERADDRESS_STRUCT                     traceCollectionEntityIPAddress;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_TRACEACTIVATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_traceactivation(
  LIBLTE_X2AP_TRACEACTIVATION_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_traceactivation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_TRACEACTIVATION_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message TraceActivation_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MDT_CONFIGURATION_STRUCT                         MDTConfiguration;
  bool                                                         MDTConfiguration_present;
}LIBLTE_X2AP_MESSAGE_TRACEACTIVATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_traceactivation_ext(
  LIBLTE_X2AP_MESSAGE_TRACEACTIVATION_EXT_STRUCT               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_traceactivation_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_TRACEACTIVATION_EXT_STRUCT               *ie);

/*******************************************************************************
/* ProtocolIE UEAggregateMaximumBitrate SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_BITRATE_STRUCT                                   uEaggregateMaximumBitRateDownlink;
  LIBLTE_X2AP_BITRATE_STRUCT                                   uEaggregateMaximumBitRateUplink;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_UEAGGREGATEMAXIMUMBITRATE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ueaggregatemaximumbitrate(
  LIBLTE_X2AP_UEAGGREGATEMAXIMUMBITRATE_STRUCT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ueaggregatemaximumbitrate(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UEAGGREGATEMAXIMUMBITRATE_STRUCT                 *ie);

/*******************************************************************************
/* Protocol Message UEAggregate_MaximumBitrates_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_UEAGGREGATE_MAXIMUMBITRATES_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ueaggregate_maximumbitrates_ext(
  LIBLTE_X2AP_MESSAGE_UEAGGREGATE_MAXIMUMBITRATES_EXT_STRUCT   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ueaggregate_maximumbitrates_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_UEAGGREGATE_MAXIMUMBITRATES_EXT_STRUCT   *ie);

/*******************************************************************************
/* ProtocolIE UE_HistoryInformation DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:16
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_LASTVISITEDCELL_ITEM_STRUCT                      buffer[16]; 
}LIBLTE_X2AP_UE_HISTORYINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ue_historyinformation(
  LIBLTE_X2AP_UE_HISTORYINFORMATION_STRUCT                              *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ue_historyinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UE_HISTORYINFORMATION_STRUCT                              *ie);

/*******************************************************************************
/* ProtocolIE UESecurityCapabilities SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ENCRYPTIONALGORITHMS_STRUCT                      encryptionAlgorithms;
  LIBLTE_X2AP_INTEGRITYPROTECTIONALGORITHMS_STRUCT             integrityProtectionAlgorithms;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_UESECURITYCAPABILITIES_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_uesecuritycapabilities(
  LIBLTE_X2AP_UESECURITYCAPABILITIES_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_uesecuritycapabilities(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UESECURITYCAPABILITIES_STRUCT                    *ie);

/*******************************************************************************
/* Protocol Message UESecurityCapabilities_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_UESECURITYCAPABILITIES_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_uesecuritycapabilities_ext(
  LIBLTE_X2AP_MESSAGE_UESECURITYCAPABILITIES_EXT_STRUCT        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_uesecuritycapabilities_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_UESECURITYCAPABILITIES_EXT_STRUCT        *ie);

/*******************************************************************************
/* ProtocolIE UL_InterferenceOverloadIndication DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:110
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_ITEM_ENUM_EXT     buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ul_interferenceoverloadindication(
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ul_interferenceoverloadindication(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_STRUCT                    *ie);

/*******************************************************************************
/* ProtocolIE UL_HighInterferenceIndicationInfo_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      target_Cell_ID;
  LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATION_STRUCT             ul_interferenceindication;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATIONINFO_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ul_highinterferenceindicationinfo_item(
  LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATIONINFO_ITEM_STRUCT                           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ul_highinterferenceindicationinfo_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATIONINFO_ITEM_STRUCT                           *ie);

/*******************************************************************************
/* Protocol Message UL_HighInterferenceIndicationInfo_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_UL_HIGHINTERFERENCEINDICATIONINFO_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ul_highinterferenceindicationinfo_item_ext(
  LIBLTE_X2AP_MESSAGE_UL_HIGHINTERFERENCEINDICATIONINFO_ITEM_EXT_STRUCT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ul_highinterferenceindicationinfo_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_UL_HIGHINTERFERENCEINDICATIONINFO_ITEM_EXT_STRUCT                   *ie);

/*******************************************************************************
/* ProtocolIE UL_HighInterferenceIndicationInfo DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATIONINFO_ITEM_STRUCT     buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATIONINFO_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ul_highinterferenceindicationinfo(
  LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATIONINFO_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ul_highinterferenceindicationinfo(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATIONINFO_STRUCT                    *ie);

/* x2ap-ie.asn FINISH */

/* x2ap-pdu-contents.asn */

/*-- **************************************************************
 *--
 *-- HANDOVER REQUEST
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* ProtocolIE E_RABs_ToBeSetup_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_E_RAB_ID_STRUCT                                  e_RAB_ID;
  LIBLTE_X2AP_E_RAB_LEVEL_QOS_PARAMETERS_STRUCT                e_RAB_Level_Qos_Parameters;
  LIBLTE_X2AP_DL_FORWARDING_ENUM_EXT                           dL_Forwarding;
  LIBLTE_X2AP_GTPTUNNELENDPOINT_STRUCT                         uL_GTPtunnelEndpoint;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_E_RABS_TOBESETUP_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_tobesetup_item(
  LIBLTE_X2AP_E_RABS_TOBESETUP_ITEM_STRUCT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_tobesetup_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_E_RABS_TOBESETUP_ITEM_STRUCT                   *ie);

/*******************************************************************************
/* Protocol Message E_RABs_ToBeSetup_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_E_RABS_TOBESETUP_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_tobesetup_item_ext(
  LIBLTE_X2AP_MESSAGE_E_RABS_TOBESETUP_ITEM_EXT_STRUCT       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_tobesetup_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_E_RABS_TOBESETUP_ITEM_EXT_STRUCT       *ie);

/*******************************************************************************
/* Protocol Message E_RABs_ToBeSetup_Item STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_E_RABS_TOBESETUP_ITEM_STRUCT                     E_RABs_ToBeSetup_Item;
}LIBLTE_X2AP_MESSAGE_E_RABS_TOBESETUP_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_tobesetup_item(
  LIBLTE_X2AP_MESSAGE_E_RABS_TOBESETUP_ITEM_STRUCT           *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rab_tobesetup_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_E_RABS_TOBESETUP_ITEM_STRUCT           *ie);

/*******************************************************************************
/* Protocol Container List E_RABs_ToBeSetup_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_E_RABS_TOBESETUP_ITEM_STRUCT                      buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_E_RABS_TOBESETUP_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_tobesetup_list(
  LIBLTE_X2AP_E_RABS_TOBESETUP_LIST_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_tobesetup_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_E_RABS_TOBESETUP_LIST_STRUCT                         *ie);

/*******************************************************************************
/* ProtocolIE UE_ContextInformation SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_UE_S1AP_ID_STRUCT                                mME_UE_S1AP_ID;
  LIBLTE_X2AP_UESECURITYCAPABILITIES_STRUCT                    uESecurityCapabilities;
  LIBLTE_X2AP_AS_SECURITYINFORMATION_STRUCT                    aS_SecurityInformation;
  LIBLTE_X2AP_UEAGGREGATEMAXIMUMBITRATE_STRUCT                 uEaggregateMaximumBitRate;
  LIBLTE_X2AP_SUBSCRIBERPROFILEIDFORRFP_STRUCT                 subscribeProfileIDforRFP;
  bool                                                         subscribeProfileIDforRFP_present;
  LIBLTE_X2AP_E_RABS_TOBESETUP_LIST_STRUCT                     E_RABS_ToBeSetup_List;
  LIBLTE_X2AP_RRC_CONTEXT_STRUCT                               rRC_Context;
  LIBLTE_X2AP_HANDOVERRESTRICTIONLIST_STRUCT                   handoverRestrictionList;
  bool                                                         handoverRestrictionList_present;
  LIBLTE_X2AP_LOCATIONREPORTINGINFORMATION_STRUCT              locationReportingInformation;
  bool                                                         locationReportingInformation_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_UE_CONTEXTINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ue_contextinformation(
  LIBLTE_X2AP_UE_CONTEXTINFORMATION_STRUCT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ue_contextinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_UE_CONTEXTINFORMATION_STRUCT                   *ie);

/*******************************************************************************
/* Protocol Message UE_ContextInformation_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MANAGEMENTBASEDMDTALLOWED_ENUM_EXT               ManagementBasedMDTallowed;
  bool                                                         ManagementBasedMDTallowed_present;
  LIBLTE_X2AP_MDTPLMNLIST_STRUCT                               ManagementBasedMDTPLMNList;
  bool                                                         ManagementBasedMDTPLMNList_present;
}LIBLTE_X2AP_MESSAGE_UE_CONTEXTINFORMATION_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_ue_contextinformation_ext(
  LIBLTE_X2AP_MESSAGE_UE_CONTEXTINFORMATION_EXT_STRUCT       *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_ue_contextinformation_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_UE_CONTEXTINFORMATION_EXT_STRUCT       *ie);

/*******************************************************************************
/* ProtocolIE MobilityInformation STATIC BIT STRING
********************************************************************************/
#define LIBLTE_X2AP_MOBILITYINFORMATION_BIT_STRING_LEN 32
typedef struct{
  uint8_t  buffer[32];
}LIBLTE_X2AP_MOBILITYINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mobilityinformation(
  LIBLTE_X2AP_MOBILITYINFORMATION_STRUCT                               *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mobilityinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MOBILITYINFORMATION_STRUCT                               *ie);

/*******************************************************************************
/* Protocol Message HandoverRequest STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                Old_eNB_UE_X2AP_ID;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
  LIBLTE_X2AP_ECGI_STRUCT                                      TargetCell_ID;
  LIBLTE_X2AP_GUMMEI_STRUCT                                    GUMMEI_ID;
  LIBLTE_X2AP_UE_CONTEXTINFORMATION_STRUCT                     UE_ContextInformation;
  LIBLTE_X2AP_UE_HISTORYINFORMATION_STRUCT                     UE_HistoryInformation;
  LIBLTE_X2AP_TRACEACTIVATION_STRUCT                           TraceActivation;
  bool                                                         TraceActivation_present;
  LIBLTE_X2AP_SRVCCOPERATIONPOSSIBLE_ENUM_EXT                  SRVCCOperationPossible;
  bool                                                         SRVCCOperationPossible_present;
  LIBLTE_X2AP_CSGMEMBERSHIPSTATUS_ENUM                         CSGMembershipStatus;
  bool                                                         CSGMembershipStatus_present;
  LIBLTE_X2AP_MOBILITYINFORMATION_STRUCT                       MobilityInformation;
  bool                                                         MobilityInformation_present;
}LIBLTE_X2AP_MESSAGE_HANDOVERREQUEST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_handoverrequest(
  LIBLTE_X2AP_MESSAGE_HANDOVERREQUEST_STRUCT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handoverrequest(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_HANDOVERREQUEST_STRUCT                   *ie);

/*-- **************************************************************
 *--
 *-- HANDOVER REQUEST ACKNOWLEDGE
 *--
 *-- **************************************************************
*/

/*******************************************************************************
/* ProtocolIE E_RABs_Admitted_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_E_RAB_ID_STRUCT                                  e_RAB_ID;
  LIBLTE_X2AP_GTPTUNNELENDPOINT_STRUCT                         uL_GTP_TunnelEndpoint;
  bool                                                         uL_GTP_TunnelEndpoint_present;
  LIBLTE_X2AP_GTPTUNNELENDPOINT_STRUCT                         dL_GTP_TunnelEndpoint;
  bool                                                         dL_GTP_TunnelEndpoint_presnet;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_E_RABS_ADMITTED_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_admitted_item(
  LIBLTE_X2AP_E_RABS_ADMITTED_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_admitted_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_E_RABS_ADMITTED_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message E_RABs_Admitted_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_E_RABS_ADMITTED_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_admitted_item_ext(
  LIBLTE_X2AP_MESSAGE_E_RABS_ADMITTED_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_admitted_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_E_RABS_ADMITTED_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Message E_RABs_Admitted_Item STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_E_RABS_ADMITTED_ITEM_STRUCT                      E_RABs_Admitted_Item;
}LIBLTE_X2AP_MESSAGE_E_RABS_ADMITTED_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_admitted_item(
  LIBLTE_X2AP_MESSAGE_E_RABS_ADMITTED_ITEM_STRUCT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_admitted_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_E_RABS_ADMITTED_ITEM_STRUCT                 *ie);

/*******************************************************************************
/* Protocol Container List E_RABs_Admitted_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_E_RABS_ADMITTED_ITEM_STRUCT                      buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_E_RABS_ADMITTED_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_admitted_list(
  LIBLTE_X2AP_E_RABS_ADMITTED_LIST_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_admitted_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_E_RABS_ADMITTED_LIST_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message HandoverRequestAcknowledge STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                Old_eNB_UE_X2AP_ID;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                New_eNB_UE_X2AP_ID;
  LIBLTE_X2AP_E_RABS_ADMITTED_LIST_STRUCT                      E_RABs_Admitted_List;
  LIBLTE_X2AP_E_RAB_LIST_STRUCT                                E_RABs_NotAdmitted_List;
  bool                                                         E_RABs_NotAdmitted_List_present;
  LIBLTE_X2AP_TARGETENBTOSOURCE_ENBTRANSPARENTCONTAINER_STRUCT TargeteNBtoSource_eNBTransparentContainer;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_HANDOVERREQUESTACKNOWLEDGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_handoverrequestacknowledge(
  LIBLTE_X2AP_MESSAGE_HANDOVERREQUESTACKNOWLEDGE_STRUCT        *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handoverrequestacknowledge(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_HANDOVERREQUESTACKNOWLEDGE_STRUCT        *ie);

/*-- **************************************************************
 *--
 *-- HANDOVER PREPARATION FAILURE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message HandoverPreparationFailure STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                Old_eNB_UE_X2AP_ID;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_HANDOVERPREPARATIONFAILURE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_handoverpreparationfailure(
  LIBLTE_X2AP_MESSAGE_HANDOVERPREPARATIONFAILURE_STRUCT                   *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handoverpreparationfailure(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_HANDOVERPREPARATIONFAILURE_STRUCT                   *ie);

/*-- **************************************************************
 *--
 *-- Handover Report
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message HandoverReport STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_HANDOVERREPORTTYPE_ENUM_EXT                      HandoverReportType;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
  LIBLTE_X2AP_ECGI_STRUCT                                      SourceCellECGI;
  LIBLTE_X2AP_ECGI_STRUCT                                      FailureCellECGI;
  LIBLTE_X2AP_ECGI_STRUCT                                      re_establishmentCellECGI;
  bool                                                         re_establishmentCellECGI_present;
  LIBLTE_X2AP_TARGETCELLINUTRAN_STRUCT                         TargetCellInUTRAN;
  bool                                                         TargetCellInUTRAN_present;
  LIBLTE_X2AP_CRNTI_STRUCT                                     SouceCellCRNTI;
  bool                                                         SourceCellCRNTI_present;
  LIBLTE_X2AP_MOBILITYINFORMATION_STRUCT                       MobilityInformation;
  bool                                                         MobilityInformation_present;
  LIBLTE_X2AP_UE_RLF_REPORT_CONTAINER_STRUCT                   UE_RLF_Report_Container;
  bool                                                         UE_RLF_Report_Container_present;
}LIBLTE_X2AP_MESSAGE_HANDOVERREPORT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_handoverreport(
  LIBLTE_X2AP_MESSAGE_HANDOVERREPORT_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handoverreport(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_HANDOVERREPORT_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- SN Status Transfer
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* ProtocolIE E_RABs_SubjectToStatusTransfer_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_E_RAB_ID_STRUCT                                  e_RAB_ID;
  LIBLTE_X2AP_RECEIVESTATUSOFULPDCPSDUS_STRUCT                 receiveStatusofULPDCPSDUs;
  bool                                                         receiveStatusofULPECPSDUs_present;
  LIBLTE_X2AP_COUNTVALUE_STRUCT                                uL_COUNTvalue;
  LIBLTE_X2AP_COUNTVALUE_STRUCT                                dL_COUNTvalue;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_subjecttostatustransfer_item(
  LIBLTE_X2AP_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_subjecttostatustransfer_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message E_RABs_SubjectToStatusTransfer_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_RECEIVESTATUSOFULPDCPSDUSEXTENDED_STRUCT         ReceiveStatusOfULPDCPSDUsExtended;
  bool                                                         ReceiveStatusOfULPDCPSDUsExtended_present;
  LIBLTE_X2AP_COUNTVALUEEXTENDED_STRUCT                        ULCOUNTValueExtended;
  bool                                                         ULCOUNTValueExtended_present;
  LIBLTE_X2AP_COUNTVALUEEXTENDED_STRUCT                        DLCOUNTValueExtended;
  bool                                                         DLCOUNTValueExtended_present;
}LIBLTE_X2AP_MESSAGE_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_subjecttostatustransfer_item_ext(
  LIBLTE_X2AP_MESSAGE_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_subjecttostatustransfer_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Message E_RABs_SubjectToStatusTransfer_Item STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_STRUCT       E_RABs_SubjectToStatusTransfer_Item;
}LIBLTE_X2AP_MESSAGE_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_subjecttostatustransfer_item(
  LIBLTE_X2AP_MESSAGE_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_STRUCT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_subjecttostatustransfer_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_STRUCT                 *ie);

/*******************************************************************************
/* Protocol Container List E_RABs_SubjectToStatusTransfer_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_E_RABS_SUBJECTTOSTATUSTRANSFER_ITEM_STRUCT                      buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_E_RABS_SUBJECTTOSTATUSTRANSFER_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_e_rabs_subjecttostatustransfer_list(
  LIBLTE_X2AP_E_RABS_SUBJECTTOSTATUSTRANSFER_LIST_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_e_rabs_subjecttostatustransfer_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_E_RABS_SUBJECTTOSTATUSTRANSFER_LIST_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message SNStatusTransfer STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                Old_eNB_UE_X2AP_ID;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                New_eNB_UE_X2AP_ID;
  LIBLTE_X2AP_E_RABS_SUBJECTTOSTATUSTRANSFER_LIST_STRUCT       E_RABs_SubjectToStatusTransfer_List;
}LIBLTE_X2AP_MESSAGE_SNSTATUSTRANSFER_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_snstatustransfer(
  LIBLTE_X2AP_MESSAGE_SNSTATUSTRANSFER_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_snstatustransfer(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_SNSTATUSTRANSFER_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- UE Context Release 
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message UEContextRelease STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                Old_eNB_UE_X2AP_ID;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                New_eNB_UE_X2AP_ID;
}LIBLTE_X2AP_MESSAGE_UECONTEXTRELEASE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_uecontextrelease(
  LIBLTE_X2AP_MESSAGE_UECONTEXTRELEASE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_uecontextrelease(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_UECONTEXTRELEASE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- HANDOVER CANCEL
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message HandoverCancel STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                Old_eNB_UE_X2AP_ID;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                New_eNB_UE_X2AP_ID;
  bool                                                         New_eNB_UE_X2AP_ID_present;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
}LIBLTE_X2AP_MESSAGE_HANDOVERCANCEL_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_handovercancel(
  LIBLTE_X2AP_MESSAGE_HANDOVERCANCEL_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handovercancel(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_HANDOVERCANCEL_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- ERROR INDICATION
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message ErrorIndication STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                Old_eNB_UE_X2AP_ID;
  bool                                                         Old_eNB_UE_X2AP_ID_present;
  LIBLTE_X2AP_UE_X2AP_ID_STRUCT                                New_eNB_UE_X2AP_ID;
  bool                                                         New_eNB_UE_X2AP_ID_present;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
  bool                                                         Cause_present;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         criticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_ERRORINDICATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_errorindication(
  LIBLTE_X2AP_MESSAGE_ERRORINDICATION_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_errorindication(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ERRORINDICATION_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- RESET REQUEST
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message ResetRequest STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
}LIBLTE_X2AP_MESSAGE_RESETREQUEST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_resetrequest(
  LIBLTE_X2AP_MESSAGE_RESETREQUEST_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_resetrequest(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_RESETREQUEST_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- RESET RESPONSE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message ResetResponse STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_RESETRESPONSE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_resetresponse(
  LIBLTE_X2AP_MESSAGE_RESETRESPONSE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_resetresponse(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_RESETRESPONSE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- X2 SETUP REQUEST
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message X2SetupRequest STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_GLOBALENB_ID_STRUCT                              GlobalENB_ID;
  LIBLTE_X2AP_SERVEDCELLS_STRUCT                               ServedCells;
  LIBLTE_X2AP_GUGROUPIDLIST_STRUCT                             GUGroupIDList;
  bool                                                         GUGroupIDList_present;
}LIBLTE_X2AP_MESSAGE_X2SETUPREQUEST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_x2setuprequest(
  LIBLTE_X2AP_MESSAGE_X2SETUPREQUEST_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_x2setuprequest(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_X2SETUPREQUEST_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- X2 SETUP RESPONSE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message X2SetupResponse STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_GLOBALENB_ID_STRUCT                              GlobalENB_ID;
  LIBLTE_X2AP_SERVEDCELLS_STRUCT                               ServedCells;
  LIBLTE_X2AP_GUGROUPIDLIST_STRUCT                             GUGroupIDList;
  bool                                                         GUGroupIDList_present;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_X2SETUPRESPONSE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_x2setupresponse(
  LIBLTE_X2AP_MESSAGE_X2SETUPRESPONSE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_x2setupresponse(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_X2SETUPRESPONSE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- X2 SETUP FAILURE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message X2SetupFailure STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
  LIBLTE_X2AP_TIMETOWAIT_ENUM_EXT                              TimeToWait;
  bool                                                         TimeToWait_present;
  LIBLTE_X2AP_GUGROUPIDLIST_STRUCT                             GUGroupIDList;
  bool                                                         GUGroupIDList_present;
}LIBLTE_X2AP_MESSAGE_X2SETUPFAILURE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_x2setupfailure(
  LIBLTE_X2AP_MESSAGE_X2SETUPFAILURE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_x2setupfailure(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_X2SETUPFAILURE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- LOAD INFORMATION
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* ProtocolIE CellInformation_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      cell_ID;
  LIBLTE_X2AP_UL_INTERFERENCEOVERLOADINDICATION_STRUCT         ul_InterferenceOverloadIndication;
  bool                                                         ul_InterferenceOverloadIndication_present;
  LIBLTE_X2AP_UL_HIGHINTERFERENCEINDICATIONINFO_STRUCT         ul_HighInterferenceIndicationInfo;
  bool                                                         ul_HighInterferenceIndicationInfo_present;
  LIBLTE_X2AP_RELATIVENARROWBANDTXPOWER_STRUCT                 relativeNarrowbandTxPower;
  bool                                                         relativeNarrowbandTxPower_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_CELLINFORMATION_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellinformation_item(
  LIBLTE_X2AP_CELLINFORMATION_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellinformation_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELLINFORMATION_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message CellInformation_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ABSINFORMATION_STRUCT                            ABSInformation;
  bool                                                         ABSInformation_present;
  LIBLTE_X2AP_INVOKEINDICATION_ENUM_EXT                        InvokedIndication;
  bool                                                         InvokedIndication_present;
}LIBLTE_X2AP_MESSAGE_CELLINFORMATION_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellinformation_item_ext(
  LIBLTE_X2AP_MESSAGE_CELLINFORMATION_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellinformation_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLINFORMATION_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Message CellInformation_Item STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CELLINFORMATION_ITEM_STRUCT                      CellInformation_Item;
}LIBLTE_X2AP_MESSAGE_CELLINFORMATION_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellinformation_item(
  LIBLTE_X2AP_MESSAGE_CELLINFORMATION_ITEM_STRUCT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellinformation_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLINFORMATION_ITEM_STRUCT                 *ie);

/*******************************************************************************
/* Protocol Container List CellInformation_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_CELLINFORMATION_ITEM_STRUCT       buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_CELLINFORMATION_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellinformation_list(
  LIBLTE_X2AP_CELLINFORMATION_LIST_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellinformation_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELLINFORMATION_LIST_STRUCT                      *ie);

/*******************************************************************************
/* Protocol Message LoadInformation STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CELLINFORMATION_LIST_STRUCT                      CellInformation;
}LIBLTE_X2AP_MESSAGE_LOADINFORMATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_loadinformation(
  LIBLTE_X2AP_MESSAGE_LOADINFORMATION_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_loadinformation(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_LOADINFORMATION_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- ENB CONFIGURATION UPDATE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* ProtocolIE ServedCellsToModify_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      cell_ID;
  LIBLTE_X2AP_SERVEDCELL_INFORMATION_STRUCT                    servedCellInfo;
  LIBLTE_X2AP_NEIGHBOUR_INFORMATION_STRUCT                     neighbour_info;
  bool                                                         neighbour_info_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_SERVEDCELLSTOMODIFY_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedcellstomodify_item(
  LIBLTE_X2AP_SERVEDCELLSTOMODIFY_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcellstomodify_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SERVEDCELLSTOMODIFY_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message ServedCellsToModify_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_DEACTIVATIONINDICATION_ENUM_EXT                  DeactivationIndication;
  bool                                                         DeactivationIndication_present;
}LIBLTE_X2AP_MESSAGE_SERVEDCELLSTOMODIFY_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedcellstomodify_item_ext(
  LIBLTE_X2AP_MESSAGE_SERVEDCELLSTOMODIFY_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcellstomodify_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_SERVEDCELLSTOMODIFY_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Container List ServedCellsToModify DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_SERVEDCELLSTOMODIFY_ITEM_STRUCT       buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_SERVEDCELLSTOMODIFY_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedcellstomodify(
  LIBLTE_X2AP_SERVEDCELLSTOMODIFY_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcellstomodify(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SERVEDCELLSTOMODIFY_STRUCT                      *ie);

/*******************************************************************************
/* Protocol Container List Old_ECGIs DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_ECGI_STRUCT                                      buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_OLD_ECGIS_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_old_ecgis(
  LIBLTE_X2AP_OLD_ECGIS_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_old_ecgis(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_OLD_ECGIS_STRUCT                      *ie);

/*******************************************************************************
/* Protocol Message ENBConfigurationUpdate STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_SERVEDCELLS_STRUCT                               ServedCellsToAdd;
  bool                                                         ServedCellsToAdd_present;
  LIBLTE_X2AP_SERVEDCELLSTOMODIFY_STRUCT                       ServedCellsToModify;
  bool                                                         ServedCellsToModify_present;
  LIBLTE_X2AP_OLD_ECGIS_STRUCT                                 ServedCellsToDelete;
  bool                                                         ServedCellsToDelete_present;
  LIBLTE_X2AP_GUGROUPIDLIST_STRUCT                             GUGroupIDToAddList;
  bool                                                         GUGroupdIDToAddList_present;
  LIBLTE_X2AP_GUGROUPIDLIST_STRUCT                             GUGroupIDToDeleteList;
  bool                                                         GUGroupIDToDeleteList_present;
}LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_enbconfigurationupdate(
  LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_enbconfigurationupdate(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- ENB CONFIGURATION UPDATE ACKNOWLEDGE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message ENBConfigurationUpdateAcknowledge STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATEACKNOWLEDGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_enbconfigurationupdateacknowledge(
  LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATEACKNOWLEDGE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_enbconfigurationupdateacknowledge(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATEACKNOWLEDGE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- ENB CONFIGURATION UPDATE FAILURE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message ENBConfigurationUpdateFailure STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
  LIBLTE_X2AP_TIMETOWAIT_ENUM_EXT                              TimeToWait;
  bool                                                         TimeToWait_present;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATEFAILURE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_enbconfigurationupdatefailure(
  LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATEFAILURE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_enbconfigurationupdatefailure(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATEFAILURE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- RESOURCE STATUS REQUEST
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* ProtocolIE CellToReport_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      cell_ID;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_CELLTOREPORT_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_celltoreport_item(
  LIBLTE_X2AP_CELLTOREPORT_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_celltoreport_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELLTOREPORT_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message CellToReport_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_CELLTOREPORT_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_celltoreport_item_ext(
  LIBLTE_X2AP_MESSAGE_CELLTOREPORT_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_celltoreport_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLTOREPORT_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Message CellToReport_Item STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CELLTOREPORT_ITEM_STRUCT                         CellToReport_Item;
}LIBLTE_X2AP_MESSAGE_CELLTOREPORT_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_celltoreport_item(
  LIBLTE_X2AP_MESSAGE_CELLTOREPORT_ITEM_STRUCT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_celltoreport_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLTOREPORT_ITEM_STRUCT                 *ie);

/*******************************************************************************
/* Protocol Container List CellToReport_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_CELLTOREPORT_ITEM_STRUCT       buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_CELLTOREPORT_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_celltoreport_list(
  LIBLTE_X2AP_CELLTOREPORT_LIST_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_celltoreport_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELLTOREPORT_LIST_STRUCT                      *ie);

/*******************************************************************************
/* ProtocolIE ReportingPeriodicity ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_REPORTINGPERIODICITY_ONE_THOUSAND_MS,
  LIBLTE_X2AP_REPORTINGPERIODICITY_TWO_THOUSAND_MS,
  LIBLTE_X2AP_REPORTINGPERIODICITY_FIVE_THOUSAND_MS,
  LIBLTE_X2AP_REPORTINGPERIODICITY_TEN_THOUSAND_MS,
  LIBLTE_X2AP_REPORTINGPERIODICITY_N_ITEMS,
}LIBLTE_X2AP_REPORTINGPERIODICITY_ENUM;
static const char liblte_x2ap_reportingperiodicity_text[LIBLTE_X2AP_REPORTINGPERIODICITY_N_ITEMS][80] = {
  "one-thousand-ms",
  "two-thousand-ms",
  "five-thousand-ms",
  "ten-thousand-ms",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_REPORTINGPERIODICITY_ENUM e;
}LIBLTE_X2AP_REPORTINGPERIODICITY_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_reportingperiodicity(
  LIBLTE_X2AP_REPORTINGPERIODICITY_ENUM_EXT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_reportingperiodicity(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_REPORTINGPERIODICITY_ENUM_EXT                                *ie);

/*******************************************************************************
/* ProtocolIE ReartialSuccessIndicator ENUMERATED
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_PARTIALSUCCESSINDICATOR_PARTIAL_SUCCESS_ALLOWED,
  LIBLTE_X2AP_PARTIALSUCCESSINDICATOR_N_ITEMS,
}LIBLTE_X2AP_PARTIALSUCCESSINDICATOR_ENUM;
static const char liblte_x2ap_partialsuccessindicator_text[LIBLTE_X2AP_PARTIALSUCCESSINDICATOR_N_ITEMS][80] = {
  "partial-success-allowed",
};

typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_PARTIALSUCCESSINDICATOR_ENUM e;
}LIBLTE_X2AP_PARTIALSUCCESSINDICATOR_ENUM_EXT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_partialsuccessindicator(
  LIBLTE_X2AP_PARTIALSUCCESSINDICATOR_ENUM_EXT                                *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_partialsuccessindicator(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_PARTIALSUCCESSINDICATOR_ENUM_EXT                                *ie);

/*******************************************************************************
/* Protocol Message ResourceStatusRequest STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MEASUREMENT_ID_STRUCT                            ENB1_Measurement_ID;
  LIBLTE_X2AP_MEASUREMENT_ID_STRUCT                            ENB2_Measurement_ID;
  bool                                                         ENB2_Measurement_ID_present;
  LIBLTE_X2AP_REGISTRATION_REQUEST_ENUM_EXT                    Registration_Request;
  LIBLTE_X2AP_REPORTCHARACTERISTICS_STRUCT                     ReportCharacteristics;
  bool                                                         ReportCharacteristics_present;
  LIBLTE_X2AP_CELLTOREPORT_LIST_STRUCT                         CellToReport;
  LIBLTE_X2AP_REPORTINGPERIODICITY_ENUM_EXT                    ReportingPeriodicity;
  bool                                                         ReportingPeriodicity_present;
  LIBLTE_X2AP_PARTIALSUCCESSINDICATOR_ENUM_EXT                 PartialSuccessIndicator;
  bool                                                         PartialSuccessIndicator_present;
}LIBLTE_X2AP_MESSAGE_RESOURCESTATUSREQUEST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_resourcestatusrequest(
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSREQUEST_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_resourcestatusrequest(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSREQUEST_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- RESOURCE STATUS RESPONSE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* ProtocolIE MeasurementFailureCause_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_REPORTCHARACTERISTICS_STRUCT                     measurementFailedReportCharacteristics;
  LIBLTE_X2AP_CAUSE_STRUCT                                     cause;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_MEASUREMENTFAILURECAUSE_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurementfailurecause_item(
  LIBLTE_X2AP_MEASUREMENTFAILURECAUSE_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurementfailurecause_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MEASUREMENTFAILURECAUSE_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message MeasurementFailureCause_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_MEASUREMENTFAILURECAUSE_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurementfailurecause_item_ext(
  LIBLTE_X2AP_MESSAGE_MEASUREMENTFAILURECAUSE_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurementfailurecause_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_MEASUREMENTFAILURECAUSE_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Message MeasurementFailureCause_Item STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MEASUREMENTFAILURECAUSE_ITEM_STRUCT              MeasurementFailureCause_Item;
}LIBLTE_X2AP_MESSAGE_MEASUREMENTFAILURECAUSE_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurementfailurecause_item(
  LIBLTE_X2AP_MESSAGE_MEASUREMENTFAILURECAUSE_ITEM_STRUCT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurementfailurecause_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_MEASUREMENTFAILURECAUSE_ITEM_STRUCT                 *ie);

/*******************************************************************************
/* Protocol Container List MeasurementFailureCause_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_MEASUREMENTFAILURECAUSE_ITEM_STRUCT              buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_MEASUREMENTFAILURECAUSE_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurementfailurecause_list(
  LIBLTE_X2AP_MEASUREMENTFAILURECAUSE_LIST_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurementfailurecause_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MEASUREMENTFAILURECAUSE_LIST_STRUCT                      *ie);

/*******************************************************************************
/* ProtocolIE MeasurementInitiationResult_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      cell_ID;
  LIBLTE_X2AP_MEASUREMENTFAILURECAUSE_LIST_STRUCT              measurementFailureCause_List;
  bool                                                         measurementFailureCause_List_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_MEASUREMENTINITIATIONRESULT_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurementinitiationresult_item(
  LIBLTE_X2AP_MEASUREMENTINITIATIONRESULT_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurementinitiationresult_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MEASUREMENTINITIATIONRESULT_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message MeasurementInitiationresult_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_MEASUREMENTINITIATIONRESULT_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurementinitiationresult_item_ext(
  LIBLTE_X2AP_MESSAGE_MEASUREMENTINITIATIONRESULT_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurementinitiationresult_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_MEASUREMENTINITIATIONRESULT_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Message MeasurementInitiationResult_Item STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MEASUREMENTINITIATIONRESULT_ITEM_STRUCT          MeasurementInitiationResult_Item;
}LIBLTE_X2AP_MESSAGE_MEASUREMENTINITIATIONRESULT_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurementinitiationresult_item(
  LIBLTE_X2AP_MESSAGE_MEASUREMENTINITIATIONRESULT_ITEM_STRUCT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurementinitiationresult_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_MEASUREMENTINITIATIONRESULT_ITEM_STRUCT                 *ie);

/*******************************************************************************
/* Protocol Container List MeasurementInitiationResult_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_MEASUREMENTINITIATIONRESULT_ITEM_STRUCT          buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_MEASUREMENTINITIATIONRESULT_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_measurementinitiationresult_list(
  LIBLTE_X2AP_MEASUREMENTINITIATIONRESULT_LIST_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_measurementinitiationresult_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MEASUREMENTINITIATIONRESULT_LIST_STRUCT                      *ie);

/*******************************************************************************
/* Protocol Message ResourceStatusResponse STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MEASUREMENT_ID_STRUCT                            ENB1_Measurement_ID;
  LIBLTE_X2AP_MEASUREMENT_ID_STRUCT                            ENB2_Measurement_ID;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
  LIBLTE_X2AP_MEASUREMENTINITIATIONRESULT_LIST_STRUCT          MeasurementInitiationResult_List;
  bool                                                         MeasurementInitiationresult_List_present;
}LIBLTE_X2AP_MESSAGE_RESOURCESTATUSRESPONSE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_resourcestatusresponse(
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSRESPONSE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_resourcestatusresponse(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSRESPONSE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- RESOURCE STATUS FAILURE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* ProtocolIE CompleteFailureCauseInformation_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      cell_ID;
  LIBLTE_X2AP_MEASUREMENTFAILURECAUSE_LIST_STRUCT              measurementFailureCause_List;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_COMPLETEFAILURECAUSEINFORMATION_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_completefailurecauseinformation_item(
  LIBLTE_X2AP_COMPLETEFAILURECAUSEINFORMATION_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_completefailurecauseinformation_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_COMPLETEFAILURECAUSEINFORMATION_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message CompleteFailureCauseInformation_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_COMPLETEFAILURECAUSEINFORMATION_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_completefailurecauseinformation_item_ext(
  LIBLTE_X2AP_MESSAGE_COMPLETEFAILURECAUSEINFORMATION_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_completefailurecauseinformation_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_COMPLETEFAILURECAUSEINFORMATION_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Message CompleteFailureCauseInformation_Item STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_COMPLETEFAILURECAUSEINFORMATION_ITEM_STRUCT      CompleteFailureCauseInformation_Item;
}LIBLTE_X2AP_MESSAGE_COMPLETEFAILURECAUSEINFORMATION_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_completefailurecauseinformation_item(
  LIBLTE_X2AP_MESSAGE_COMPLETEFAILURECAUSEINFORMATION_ITEM_STRUCT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_completefailurecauseinformation_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_COMPLETEFAILURECAUSEINFORMATION_ITEM_STRUCT                 *ie);

/*******************************************************************************
/* Protocol Container List CompleteFailureCauseInformation_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_COMPLETEFAILURECAUSEINFORMATION_ITEM_STRUCT      buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_COMPLETEFAILURECAUSEINFORMATION_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_completefailurecauseinformation_list(
  LIBLTE_X2AP_COMPLETEFAILURECAUSEINFORMATION_LIST_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_completefailurecauseinformation_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_COMPLETEFAILURECAUSEINFORMATION_LIST_STRUCT                      *ie);

/*******************************************************************************
/* Protocol Message ResourceStatusFailure STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MEASUREMENT_ID_STRUCT                            ENB1_Measurement_ID;
  LIBLTE_X2AP_MEASUREMENT_ID_STRUCT                            ENB2_Measurement_ID;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
  LIBLTE_X2AP_COMPLETEFAILURECAUSEINFORMATION_LIST_STRUCT      CompleteFailureCauseInformation_List;
  bool                                                         CompleteFailureCauseInformation_List_present;
}LIBLTE_X2AP_MESSAGE_RESOURCESTATUSFAILURE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_resourcestatusfailure(
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSFAILURE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_resourcestatusfailure(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSFAILURE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- RESOURCE STATUS UPDATE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* ProtocolIE CellMeasurementResult_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      cell_ID;
  LIBLTE_X2AP_HWLOADINDICATOR_STRUCT                           hWLoadIndicator;
  bool                                                         hWLoadIndicator_present;
  LIBLTE_X2AP_S1TNLLOADINDICATOR_STRUCT                        s1TNLLoadIndicator;
  bool                                                         s1TNLLoadIndicator_present;
  LIBLTE_X2AP_RADIORESOURCESTATUS_STRUCT                       radioResourceStatus;
  bool                                                         radioResourceStatus_present;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_CELLMEASUREMENTRESULT_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellmeasurementresult_item(
  LIBLTE_X2AP_CELLMEASUREMENTRESULT_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellmeasurementresult_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELLMEASUREMENTRESULT_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message CellMeasurementResult_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_COMPOSITEAVAILABLECAPACITYGROUP_STRUCT           CompositeAvailableCapacityGroup;
  bool                                                         CompositeAvailableCapacityGroup_present;
  LIBLTE_X2AP_ABS_STATUS_STRUCT                                ABS_Status;
  bool                                                         ABS_Status_present;
}LIBLTE_X2AP_MESSAGE_CELLMEASUREMENTRESULT_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellmeasurementresult_item_ext(
  LIBLTE_X2AP_MESSAGE_CELLMEASUREMENTRESULT_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellmeasurementresult_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLMEASUREMENTRESULT_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Message CellMeasurementResult_Item STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CELLMEASUREMENTRESULT_ITEM_STRUCT                CellMeasurementResult_Item;
}LIBLTE_X2AP_MESSAGE_CELLMEASUREMENTRESULT_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellmeasurementresult_item(
  LIBLTE_X2AP_MESSAGE_CELLMEASUREMENTRESULT_ITEM_STRUCT                 *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellmeasurementresult_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLMEASUREMENTRESULT_ITEM_STRUCT                 *ie);

/*******************************************************************************
/* Protocol Container List CellMeasurementResult_List DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_CELLMEASUREMENTRESULT_ITEM_STRUCT                buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_CELLMEASUREMENTRESULT_LIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellmeasurementresult_list(
  LIBLTE_X2AP_CELLMEASUREMENTRESULT_LIST_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellmeasurementresult_list(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_CELLMEASUREMENTRESULT_LIST_STRUCT                      *ie);

/*******************************************************************************
/* Protocol Message ResourceStatusUpdate STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_MEASUREMENT_ID_STRUCT                            ENB1_Measurement_ID;
  LIBLTE_X2AP_MEASUREMENT_ID_STRUCT                            ENB2_Measurement_ID;
  LIBLTE_X2AP_CELLMEASUREMENTRESULT_LIST_STRUCT              CellMeasurementResult;
}LIBLTE_X2AP_MESSAGE_RESOURCESTATUSUPDATE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_resourcestatusupdate(
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSUPDATE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_resourcestatusupdate(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSUPDATE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- PRIVATE MESSAGE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message PrivateMessage STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_PRIVATEMESSAGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_privatemessage(
  LIBLTE_X2AP_MESSAGE_PRIVATEMESSAGE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_privatemessage(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_PRIVATEMESSAGE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- MOBILITY CHANGE REQUEST
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message MobilityChangeRequest STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      ENB1_Cell_ID;
  LIBLTE_X2AP_ECGI_STRUCT                                      ENB2_Cell_ID;
  LIBLTE_X2AP_MOBILITYPARAMETERSINFORMATION_STRUCT             ENB1_Mobility_Parameters;
  bool                                                         ENB1_Mobility_Parameters_present;
  LIBLTE_X2AP_MOBILITYPARAMETERSINFORMATION_STRUCT             ENB2_Proposed_Mobility_Parameters;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
}LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEREQUEST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mobilitychangerequest(
  LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEREQUEST_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mobilitychangerequest(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEREQUEST_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- MOBILITY CHANGE ACKNOWLEDGE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message MobilityChangeAcknowledge STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      ENB1_Cell_ID;
  LIBLTE_X2AP_ECGI_STRUCT                                      ENB2_Cell_ID;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEACKNOWLEDGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mobilitychangeacknowledge(
  LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEACKNOWLEDGE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mobilitychangeacknowledge(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEACKNOWLEDGE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- MOBILITY CHANGE FAILURE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message MobilityChangeFailure STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      ENB1_Cell_ID;
  LIBLTE_X2AP_ECGI_STRUCT                                      ENB2_Cell_ID;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
  LIBLTE_X2AP_MOBILITYPARAMETERSMODIFICATIONRANGE_STRUCT       ENB2_Mobility_Parameters_Modification_Range;
  bool                                                         ENB2_Mobility_Parameters_Modification_Range_present;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEFAILURE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_mobilitychangefailure(
  LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEFAILURE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_mobilitychangefailure(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEFAILURE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- RADIO LINK FAILURE INDICATION
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message RLFIndication STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_PCI_STRUCT                                       FailureCellPCI;
  LIBLTE_X2AP_ECGI_STRUCT                                      Re_establishmentCellECGI;
  LIBLTE_X2AP_CRNTI_STRUCT                                     FailureCellCRNTI;
  LIBLTE_X2AP_SHORTMAC_I_STRUCT                                ShortMAC_I;
  bool                                                         ShortMAC_I_present;
  LIBLTE_X2AP_UE_RLF_REPORT_CONTAINER_STRUCT                   UE_RLF_Report_Container;
  bool                                                         UE_RLF_Report_Container_present;
  LIBLTE_X2AP_RRCCONNSETUPINDICATOR_ENUM_EXT                   RRCConnSetupIndicator;
  bool                                                         RRCConnSetupIndicator_present;
  LIBLTE_X2AP_RRCCONNREESTABINDICATOR_ENUM_EXT                 RRCConnReestabIndicator;
  bool                                                         RRCConnReestabIndicator_present;
}LIBLTE_X2AP_MESSAGE_RLFINDICATION_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_rlfindication(
  LIBLTE_X2AP_MESSAGE_RLFINDICATION_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_rlfindication(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_RLFINDICATION_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- CELL ACTIVATION REQUEST
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* ProtocolIE ServedCellsToActivate_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      ecgi;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_SERVEDCELLSTOACTIVATE_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedcellstoactivate_item(
  LIBLTE_X2AP_SERVEDCELLSTOACTIVATE_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcellstoactivate_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SERVEDCELLSTOACTIVATE_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message ServedCellsToActivate_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_SERVEDCELLSTOACTIVATE_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedcellstoactivate_item_ext(
  LIBLTE_X2AP_MESSAGE_SERVEDCELLSTOACTIVATE_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcellstoactivate_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_SERVEDCELLSTOACTIVATE_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Container List ServedCellsToActivate DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_SERVEDCELLSTOACTIVATE_ITEM_STRUCT                buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_SERVEDCELLSTOACTIVATE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_servedcellstoactivate(
  LIBLTE_X2AP_SERVEDCELLSTOACTIVATE_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_servedcellstoactivate(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_SERVEDCELLSTOACTIVATE_STRUCT                      *ie);

/*******************************************************************************
/* Protocol Message CellActivationRequest STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_SERVEDCELLSTOACTIVATE_STRUCT                     ServedCellsToActivate;
}LIBLTE_X2AP_MESSAGE_CELLACTIVATIONREQUEST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellactivationrequest(
  LIBLTE_X2AP_MESSAGE_CELLACTIVATIONREQUEST_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellactivationrequest(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLACTIVATIONREQUEST_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- CELL ACTIVATION RESPONSE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* ProtocolIE ActivatedCellList_Item SEQUENCE
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ECGI_STRUCT                                      ecgi;
  LIBLTE_X2AP_PROTOCOLEXTENSIONCONTAINER_STRUCT                iE_Extensions;
  bool                                                         iE_Extensions_present;
}LIBLTE_X2AP_ACTIVATEDCELLLIST_ITEM_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_activatedcelllist_item(
  LIBLTE_X2AP_ACTIVATEDCELLLIST_ITEM_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_activatedcelllist_item(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ACTIVATEDCELLLIST_ITEM_STRUCT                         *ie);

/*******************************************************************************
/* Protocol Message ActivatedCellList_Item_Ext STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
}LIBLTE_X2AP_MESSAGE_ACTIVATEDCELLLIST_ITEM_EXT_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_activatedcelllist_item_ext(
  LIBLTE_X2AP_MESSAGE_ACTIVATEDCELLLIST_ITEM_EXT_STRUCT             *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_activatedcelllist_item_ext(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_ACTIVATEDCELLLIST_ITEM_EXT_STRUCT             *ie);

/*******************************************************************************
/* Protocol Container List ActivatedCellList DYNAMIC SEQUENCE OF
********************************************************************************/
// lb:1, ub:256
typedef struct{
  uint32_t                                                     len;
  LIBLTE_X2AP_ACTIVATEDCELLLIST_ITEM_STRUCT                    buffer[32]; //WARNING: Artificial limit to reduce memory footprint
}LIBLTE_X2AP_ACTIVATEDCELLLIST_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_activatedcelllist(
  LIBLTE_X2AP_ACTIVATEDCELLLIST_STRUCT                         *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_activatedcelllist(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_ACTIVATEDCELLLIST_STRUCT                      *ie);

/*******************************************************************************
/* Protocol Message CellActivationResponse STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_ACTIVATEDCELLLIST_STRUCT                         ActivatedCellList;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_CELLACTIVATIONRESPONSE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellactivationresponse(
  LIBLTE_X2AP_MESSAGE_CELLACTIVATIONRESPONSE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellactivationresponse(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLACTIVATIONRESPONSE_STRUCT                    *ie);

/*-- **************************************************************
 *--
 *-- CELL ACTIVATION FAILURE
 *--
 *-- **************************************************************
 */

/*******************************************************************************
/* Protocol Message CellActivationFailure STRUCT
********************************************************************************/
typedef struct{
  bool                                                         ext;
  LIBLTE_X2AP_CAUSE_STRUCT                                     Cause;
  LIBLTE_X2AP_CRITICALITYDIAGNOSTICS_STRUCT                    CriticalityDiagnostics;
  bool                                                         CriticalityDiagnostics_present;
}LIBLTE_X2AP_MESSAGE_CELLACTIVATIONFAILURE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_cellactivationfailure(
  LIBLTE_X2AP_MESSAGE_CELLACTIVATIONFAILURE_STRUCT                    *ie,
  uint8_t                                                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_cellactivationfailure(
  uint8_t                                                     **ptr,
  LIBLTE_X2AP_MESSAGE_CELLACTIVATIONFAILURE_STRUCT                    *ie);

/* X2AP-PDU-Contents.asn FINISH */

/* X2AP-PDU-Descriptions.asn */

/*******************************************************************************
/* ProtocolIE-Field
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolie_header(
  uint32_t                      len,
  uint32_t                      ie_id,
  LIBLTE_X2AP_CRITICALITY_ENUM  crit,
  uint8_t                     **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolie_header(
  uint8_t                     **ptr,
  uint32_t                     *ie_id,
  LIBLTE_X2AP_CRITICALITY_ENUM *crit,
  uint32_t                     *len);

/*******************************************************************************
/* Procedure code criticality lookups
********************************************************************************/
static const LIBLTE_X2AP_CRITICALITY_ENUM liblte_x2ap_procedure_criticality[16] = {
  LIBLTE_X2AP_CRITICALITY_REJECT,
  LIBLTE_X2AP_CRITICALITY_IGNORE,
  LIBLTE_X2AP_CRITICALITY_IGNORE,
  LIBLTE_X2AP_CRITICALITY_IGNORE,
  LIBLTE_X2AP_CRITICALITY_IGNORE,
  LIBLTE_X2AP_CRITICALITY_IGNORE,
  LIBLTE_X2AP_CRITICALITY_REJECT,
  LIBLTE_X2AP_CRITICALITY_REJECT,
  LIBLTE_X2AP_CRITICALITY_IGNORE,
  LIBLTE_X2AP_CRITICALITY_REJECT,
  LIBLTE_X2AP_CRITICALITY_REJECT,
  LIBLTE_X2AP_CRITICALITY_IGNORE,
  LIBLTE_X2AP_CRITICALITY_IGNORE,
  LIBLTE_X2AP_CRITICALITY_IGNORE,
  LIBLTE_X2AP_CRITICALITY_REJECT,
  LIBLTE_X2AP_CRITICALITY_REJECT,
};

/*******************************************************************************
/* InitiatingMessage CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_HANDOVERREQUEST,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_SNSTATUSTRANSFER,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_UECONTEXTRELEASE,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_HANDOVERCANCEL,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_HANDOVERREPORT,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_ERRORINDICATION,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_RESETREQUEST,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_X2SETUPREQUEST,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_LOADINFORMATION,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_ENBCONFIGURATIONUPDATE,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_RESOURCESTATUSREQUEST,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_RESOURCESTATUSUPDATE,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_RLFINDICATION,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_PRIVATEMESSAGE,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_MOBILITYCHANGEREQUEST,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_CELLACTIVATIONREQUEST,
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_N_ITEMS,
}LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_ENUM;
static const char liblte_x2ap_initiatingmessage_choice_text[LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_N_ITEMS][50] = {
  "HandoverRequest",
  "SNStatusTransfer",
  "UEContextRelease",
  "HandoverCancel",
  "HandoverReport",
  "ErrorIndication",
  "ResetRequest",
  "X2SetupRequest",
  "LoadInformation",
  "ENBConfigurationUpdate",
  "ResourceStatusRequest",
  "ResourceStatusUpdate",
  "RLFIndication",
  "PrivateMessage",
  "MobilityChangeRequest",
  "CellActivationRequest",
};

typedef union{
  LIBLTE_X2AP_MESSAGE_HANDOVERREQUEST_STRUCT           HandoverRequest;
  LIBLTE_X2AP_MESSAGE_SNSTATUSTRANSFER_STRUCT          SNStatusTransfer;
  LIBLTE_X2AP_MESSAGE_UECONTEXTRELEASE_STRUCT          UEContextRelease;
  LIBLTE_X2AP_MESSAGE_HANDOVERCANCEL_STRUCT            HandoverCancel;
  LIBLTE_X2AP_MESSAGE_HANDOVERREPORT_STRUCT            HandoverReport;
  LIBLTE_X2AP_MESSAGE_ERRORINDICATION_STRUCT           ErrorIndication;
  LIBLTE_X2AP_MESSAGE_RESETREQUEST_STRUCT              ResetRequest;
  LIBLTE_X2AP_MESSAGE_X2SETUPREQUEST_STRUCT            X2SetupRequest;
  LIBLTE_X2AP_MESSAGE_LOADINFORMATION_STRUCT           LoadInformation;
  LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATE_STRUCT    ENBConfigurationUpdate;
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSREQUEST_STRUCT     ResourceStatusRequest;
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSUPDATE_STRUCT      ResourceStatusUpdate;
  LIBLTE_X2AP_MESSAGE_RLFINDICATION_STRUCT             RLFIndication;
  LIBLTE_X2AP_MESSAGE_PRIVATEMESSAGE_STRUCT            PrivateMessage;
  LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEREQUEST_STRUCT     MobilityChangeRequest;
  LIBLTE_X2AP_MESSAGE_CELLACTIVATIONREQUEST_STRUCT     CellActivationRequest;
}LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_UNION;

typedef struct{
  uint8_t                                               procedureCode;
  LIBLTE_X2AP_CRITICALITY_ENUM                          criticality;
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_UNION            choice;
  LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_ENUM             choice_type;
}LIBLTE_X2AP_INITIATINGMESSAGE_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_initiatingmessage(
  LIBLTE_X2AP_INITIATINGMESSAGE_STRUCT *msg,
  uint8_t                             **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_initiatingmessage(
  uint8_t                             **ptr,
  LIBLTE_X2AP_INITIATINGMESSAGE_STRUCT *msg);

/*******************************************************************************
/* UnsuccessfulOutcome CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_HANDOVERPREPARATIONFAILURE,
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_X2SETUPFAILURE,
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_ENBCONFIGURATIONUPDATEFAILURE,
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_RESOURCESTATUSFAILURE,
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_MOBILITYCHANGEFAILURE,
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_CELLACTIVATIONFAILURE,
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_N_ITEMS,
}LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_ENUM;
static const char liblte_x2ap_unsuccessfuloutcome_choice_text[LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_N_ITEMS][50] = {
  "HandoverPreparationFailure",
  "X2SetupFailure",
  "ENBConfigurationUpdateFailure",
  "ResourceStatusFailure",
  "MobilityChangeFailure",
  "CellActivationFailure",
};

typedef union{
  LIBLTE_X2AP_MESSAGE_HANDOVERPREPARATIONFAILURE_STRUCT    HandoverPreparationFailure;
  LIBLTE_X2AP_MESSAGE_X2SETUPFAILURE_STRUCT                X2SetupFailure;
  LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATEFAILURE_STRUCT ENBConfigurationUpdateFailure;
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSFAILURE_STRUCT         ResourceStatusFailure;
  LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEFAILURE_STRUCT         MobilityChangeFailure;
  LIBLTE_X2AP_MESSAGE_CELLACTIVATIONFAILURE_STRUCT         CellActivationFailure;
}LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_UNION;

typedef struct{
  uint8_t                                                 procedureCode;
  LIBLTE_X2AP_CRITICALITY_ENUM                            criticality;
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_UNION            choice;
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_ENUM             choice_type;
}LIBLTE_X2AP_UNSUCCESSFULOUTCOME_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_unsuccessfuloutcome(
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_STRUCT *msg,
  uint8_t                               **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_unsuccessfuloutcome(
  uint8_t                               **ptr,
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_STRUCT *msg);

/*******************************************************************************
/* SuccessfulOutcome CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_HANDOVERREQUESTACKNOWLEDGE,
  LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_RESETRESPONSE,
  LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_X2SETUPRESPONSE,
  LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_ENBCONFIGURATIONUPDATEACKNOWLEDGE,
  LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_RESOURCESTATUSRESPONSE,
  LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_MOBILITYCHANGEACKNOWLEDGE,
  LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_CELLACTIVATIONRESPONSE,
  LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_N_ITEMS,
}LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_ENUM;
static const char liblte_x2ap_successfuloutcome_choice_text[LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_N_ITEMS][50] = {
  "HandoverRequestAcknowledge",
  "ResetResponse",
  "X2SetupResponse",
  "ENBConfigurationUpdateAcknowledge",
  "ResourceStatusResponse",
  "MobilityChangeAcknowledge",
  "CellActivationResponse"
};

typedef union{
  LIBLTE_X2AP_MESSAGE_HANDOVERREQUESTACKNOWLEDGE_STRUCT         HandoverRequestAcknowledge;
  LIBLTE_X2AP_MESSAGE_RESETRESPONSE_STRUCT                      ResetResponse;
  LIBLTE_X2AP_MESSAGE_X2SETUPRESPONSE_STRUCT                    X2SetupResponse;
  LIBLTE_X2AP_MESSAGE_ENBCONFIGURATIONUPDATEACKNOWLEDGE_STRUCT  ENBConfigurationUpdateAcknowledge;
  LIBLTE_X2AP_MESSAGE_RESOURCESTATUSRESPONSE_STRUCT             ResourceStatusResponse;
  LIBLTE_X2AP_MESSAGE_MOBILITYCHANGEACKNOWLEDGE_STRUCT          MobilityChangeAcknowledge;
  LIBLTE_X2AP_MESSAGE_CELLACTIVATIONRESPONSE_STRUCT             CellActivationResponse;
}LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_UNION;

typedef struct{
  uint8_t                                               procedureCode;
  LIBLTE_X2AP_CRITICALITY_ENUM                          criticality;
  LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_UNION            choice;
  LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_ENUM             choice_type;
}LIBLTE_X2AP_SUCCESSFULOUTCOME_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_successfuloutcome(
  LIBLTE_X2AP_SUCCESSFULOUTCOME_STRUCT *msg,
  uint8_t                             **ptr);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_successfuloutcome(
  uint8_t                             **ptr,
  LIBLTE_X2AP_SUCCESSFULOUTCOME_STRUCT *msg);

/*******************************************************************************
/* X2AP_PDU CHOICE
********************************************************************************/
typedef enum{
  LIBLTE_X2AP_X2AP_PDU_CHOICE_INITIATINGMESSAGE,
  LIBLTE_X2AP_X2AP_PDU_CHOICE_SUCCESSFULOUTCOME,
  LIBLTE_X2AP_X2AP_PDU_CHOICE_UNSUCCESSFULOUTCOME,
  LIBLTE_X2AP_X2AP_PDU_CHOICE_N_ITEMS,
}LIBLTE_X2AP_X2AP_PDU_CHOICE_ENUM;
static const char liblte_x2ap_x2ap_pdu_choice_text[LIBLTE_X2AP_X2AP_PDU_CHOICE_N_ITEMS][50] = {
  "initiatingMessage",
  "successfulOutcome",
  "unsuccessfulOutcome",
};

typedef union{
  LIBLTE_X2AP_INITIATINGMESSAGE_STRUCT                         initiatingMessage;
  LIBLTE_X2AP_SUCCESSFULOUTCOME_STRUCT                         successfulOutcome;
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_STRUCT                       unsuccessfulOutcome;
}LIBLTE_X2AP_X2AP_PDU_CHOICE_UNION;

typedef struct{
  bool ext;
  LIBLTE_X2AP_X2AP_PDU_CHOICE_UNION choice;
  LIBLTE_X2AP_X2AP_PDU_CHOICE_ENUM  choice_type;
}LIBLTE_X2AP_X2AP_PDU_STRUCT;

LIBLTE_ERROR_ENUM liblte_x2ap_pack_x2ap_pdu(
  LIBLTE_X2AP_X2AP_PDU_STRUCT *x2ap_pdu,
  LIBLTE_BYTE_MSG_STRUCT      *msg);
LIBLTE_ERROR_ENUM liblte_x2ap_unpack_x2ap_pdu(
  LIBLTE_BYTE_MSG_STRUCT      *msg,
  LIBLTE_X2AP_X2AP_PDU_STRUCT *x2ap_pdu);
  
#endif //LIBLTE_X2AP_h