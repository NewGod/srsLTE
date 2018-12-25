#include "srslte/asn1/liblte_x2ap.h"
# include <stdio.h>
# include <stdarg.h>
# include <math.h>

/*******************************************************************************
                              LOGGING
*******************************************************************************/

static log_handler_t log_handler;
static void *callback_ctx = NULL;

void liblte_x2ap_log_register_handler(void *ctx, log_handler_t handler) {
  log_handler  = handler;
  callback_ctx = ctx;
}

static void liblte_x2ap_log_print(const char *format, ...) {
  va_list   args;
  va_start(args, format);
  if (log_handler) {
    char *args_msg = NULL;
    if(vasprintf(&args_msg, format, args) > 0) {
      log_handler(callback_ctx, args_msg);
    }
    if (args_msg) {
      free(args_msg);
    }
  } else {
    vprintf(format, args);
  }
  va_end(args);
}

/*******************************************************************************
/* ProtocolIE-Field
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_protocolie_header(
  uint32_t                      len,
  uint32_t                      ie_id,
  LIBLTE_X2AP_CRITICALITY_ENUM  crit,
  uint8_t                     **ptr)
{
  liblte_value_2_bits(ie_id, ptr, 16); // ProtocolIE-ID
  liblte_value_2_bits(crit,  ptr, 2);  // Criticality
  liblte_align_up_zero(ptr, 8);
  if(len < 128) {                      // Length
    liblte_value_2_bits(0,   ptr, 1);
    liblte_value_2_bits(len, ptr, 7);
  } else if(len < 16383) {
    liblte_value_2_bits(1,   ptr, 1);
    liblte_value_2_bits(0,   ptr, 1);
    liblte_value_2_bits(len, ptr, 14);
  } else {
    // FIXME: Unlikely to have more than 16K of octets
  }

  return LIBLTE_SUCCESS;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_protocolie_header(
  uint8_t                     **ptr,
  uint32_t                     *ie_id,
  LIBLTE_X2AP_CRITICALITY_ENUM *crit,
  uint32_t                     *len)
{
  *ie_id = liblte_bits_2_value(ptr, 16);                               // ProtocolIE-ID
  *crit  = (LIBLTE_X2AP_CRITICALITY_ENUM)liblte_bits_2_value(ptr, 2);  // Criticality
  liblte_align_up(ptr, 8);
  if(0 == liblte_bits_2_value(ptr, 1)) {                               // Length
    *len = liblte_bits_2_value(ptr, 7);
  } else {
    if(0 == liblte_bits_2_value(ptr, 1)) {
      *len = liblte_bits_2_value(ptr, 14);
    } else {
      // FIXME: Unlikely to have more than 16K of octets
    }
  }

  return LIBLTE_SUCCESS;
}

/*******************************************************************************
/* Protocol Message UEContextRelease STRUCT
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_uecontextrelease(
  LIBLTE_X2AP_MESSAGE_UECONTEXTRELEASE_STRUCT                        *msg,
  uint8_t                                                           **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {

    // Extension
    liblte_value_2_bits(msg->ext?1:0, ptr, 1);
    liblte_align_up_zero(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("UEContextRelease-IEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_ENCODE_FAIL;
    }

    // No. of ProtocolIE
    uint32_t n_ie = 2;
    liblte_value_2_bits(n_ie, ptr, 16);

    // Temp container for IEs
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t              *tmp_ptr;

    // ProtocolIE - Old_eNB_UE_X2AP_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ue_x2ap_id(&msg->Old_eNB_UE_X2AP_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_UE_X2AP_ID,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - New_eNB_UE_X2AP_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ue_x2ap_id(&msg->New_eNB_UE_X2AP_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_UE_X2AP_ID,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_uecontextrelease(
  uint8_t                                                           **ptr,
  LIBLTE_X2AP_MESSAGE_UECONTEXTRELEASE_STRUCT                 *msg)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {
    LIBLTE_X2AP_CRITICALITY_ENUM crit;
    uint32_t ie_id;
    uint32_t len;
    uint32_t n_ie;
    uint32_t i;

    // Set booleans

    // Extension
    msg->ext  = liblte_bits_2_value(ptr, 1);
    liblte_align_up(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("UEContextRelease-IEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_DECODE_FAIL;
    }

    // No. of ProtocolIE-Container
    n_ie = liblte_bits_2_value(ptr, 16);
    bool old = false;

    // Unpack ProtocolIE Fields
    for(i=0;i<n_ie;i++) {
      if(liblte_x2ap_unpack_protocolie_header(ptr, &ie_id, &crit, &len) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
      if(LIBLTE_X2AP_IE_ID_UE_X2AP_ID == ie_id && !old) {
        /*if(liblte_x2ap_unpack_ue_x2ap_id(ptr, &msg->Old_eNB_UE_X2AP_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        old = true;
      } else      if(LIBLTE_X2AP_IE_ID_UE_X2AP_ID == ie_id) {
        /*if(liblte_x2ap_unpack_ue_x2ap_id(ptr, &msg->New_eNB_UE_X2AP_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } 
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* Protocol Message SNStatusTransfer STRUCT
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_snstatustransfer(
  LIBLTE_X2AP_MESSAGE_SNSTATUSTRANSFER_STRUCT                       *msg,
  uint8_t                                                           **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {

    // Extension
    liblte_value_2_bits(msg->ext?1:0, ptr, 1);
    liblte_align_up_zero(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("SNStatusTransferIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_ENCODE_FAIL;
    }

    // No. of ProtocolIE
    uint32_t n_ie = 3;
    liblte_value_2_bits(n_ie, ptr, 16);

    // Temp container for IEs
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t              *tmp_ptr;

    // ProtocolIE - Old_eNB_UE_X2AP_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ue_x2ap_id(&msg->Old_eNB_UE_X2AP_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_UE_X2AP_ID,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - New_eNB_UE_X2AP_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ue_x2ap_id(&msg->New_eNB_UE_X2AP_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_UE_X2AP_ID,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - E_RABs_SubjectToStatusTransfer_List
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_e_rabs_subjecttostatustransfer_list(&msg->E_RABs_SubjectToStatusTransfer_List, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_E_RABS_SUBJECTTOSTATUSTRANSFER_LIST,
                                          LIBLTE_X2AP_CRITICALITY_IGNORE,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_snstatustransfer(
  uint8_t                                                           **ptr,
  LIBLTE_X2AP_MESSAGE_SNSTATUSTRANSFER_STRUCT                       *msg)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {
    LIBLTE_X2AP_CRITICALITY_ENUM crit;
    uint32_t ie_id;
    uint32_t len;
    uint32_t n_ie;
    uint32_t i;

    // Set booleans

    // Extension
    msg->ext  = liblte_bits_2_value(ptr, 1);
    liblte_align_up(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("SNStatusTransferIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_DECODE_FAIL;
    }

    // No. of ProtocolIE-Container
    n_ie = liblte_bits_2_value(ptr, 16);
    bool old = false;

    // Unpack ProtocolIE Fields
    for(i=0;i<n_ie;i++) {
      if(liblte_x2ap_unpack_protocolie_header(ptr, &ie_id, &crit, &len) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
      if(LIBLTE_X2AP_IE_ID_UE_X2AP_ID == ie_id && !old) {
        /*if(liblte_x2ap_unpack_ue_x2ap_id(ptr, &msg->Old_eNB_UE_X2AP_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        old = true;
      } else      if(LIBLTE_X2AP_IE_ID_UE_X2AP_ID == ie_id && old) {
        /*if(liblte_x2ap_unpack_ue_x2ap_id(ptr, &msg->New_eNB_UE_X2AP_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_E_RABS_SUBJECTTOSTATUSTRANSFER_LIST == ie_id) {
        /*if(liblte_x2ap_unpack_e_rabs_subjecttostatustransfer_list(ptr, &msg->E_RABs_SubjectToStatusTransfer_List) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } 
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* Protocol Message HandoverRequest STRUCT
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_handoverrequest(
  LIBLTE_X2AP_MESSAGE_HANDOVERREQUEST_STRUCT                         *msg,
  uint8_t                                                           **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {

    // Extension
    liblte_value_2_bits(msg->ext?1:0, ptr, 1);
    liblte_align_up_zero(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("HandoverRequestIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_ENCODE_FAIL;
    }

    // No. of ProtocolIE
    uint32_t n_ie = 10;
    if(!msg->TraceActivation_present)
      n_ie--;
    if(!msg->SRVCCOperationPossible_present)
      n_ie--;
    if(!msg->CSGMembershipStatus_present)
      n_ie--;
    if(!msg->MobilityInformation_present)
      n_ie--;
    liblte_value_2_bits(n_ie, ptr, 16);

    // Temp container for IEs
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t              *tmp_ptr;

    // ProtocolIE - Old_eNB_UE_X2AP_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ue_x2ap_id(&msg->Old_eNB_UE_X2AP_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_UE_X2AP_ID,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - Cause
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_cause(&msg->Cause, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_CAUSE,
                                          LIBLTE_X2AP_CRITICALITY_IGNORE,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - TargetCell_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ecgi(&msg->TargetCell_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_TARGETCELL_ID,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - GUMMEI_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_gummei(&msg->GUMMEI_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_GUMMEI_ID,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - UE_ContextInformation
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ue_contextinformation(&msg->UE_ContextInformation, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_UE_CONTEXTINFORMATION,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - UE_HistoryInformation
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ue_historyinformation(&msg->UE_HistoryInformation, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_UE_HISTORYINFORMATION,
                                          LIBLTE_X2AP_CRITICALITY_IGNORE,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - TraceActivation
    if(msg->TraceActivation_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_traceactivation(&msg->TraceActivation, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_TRACEACTIVATION,
                                            LIBLTE_X2AP_CRITICALITY_IGNORE,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    // ProtocolIE - SRVCCOperationPossible
    if(msg->SRVCCOperationPossible_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_srvccoperationpossible(&msg->SRVCCOperationPossible, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_SRVCCOPERATIONPOSSIBLE,
                                            LIBLTE_X2AP_CRITICALITY_IGNORE,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    // ProtocolIE - CSGMembershipStatus
    if(msg->CSGMembershipStatus_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_csgmembershipstatus(&msg->CSGMembershipStatus, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_CSGMEMBERSHIPSTATUS,
                                            LIBLTE_X2AP_CRITICALITY_REJECT,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    // ProtocolIE - MobilityInformation
    if(msg->MobilityInformation_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_mobilityinformation(&msg->MobilityInformation, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_MOBILITYINFORMATION,
                                            LIBLTE_X2AP_CRITICALITY_IGNORE,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handoverrequest(
  uint8_t                                                           **ptr,
  LIBLTE_X2AP_MESSAGE_HANDOVERREQUEST_STRUCT                         *msg)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {
    LIBLTE_X2AP_CRITICALITY_ENUM crit;
    uint32_t ie_id;
    uint32_t len;
    uint32_t n_ie;
    uint32_t i;

    // Set booleans
    msg->TraceActivation_present = false;
    msg->SRVCCOperationPossible_present = false;
    msg->CSGMembershipStatus_present = false;
    msg->MobilityInformation_present = false;

    // Extension
    msg->ext  = liblte_bits_2_value(ptr, 1);
    liblte_align_up(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("HandoverRequestIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_DECODE_FAIL;
    }

    // No. of ProtocolIE-Container
    n_ie = liblte_bits_2_value(ptr, 16);

    // Unpack ProtocolIE Fields
    for(i=0;i<n_ie;i++) {
      if(liblte_x2ap_unpack_protocolie_header(ptr, &ie_id, &crit, &len) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
      if(LIBLTE_X2AP_IE_ID_UE_X2AP_ID == ie_id) {
        /*if(liblte_x2ap_unpack_ue_x2ap_id(ptr, &msg->Old_eNB_UE_X2AP_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_CAUSE == ie_id) {
        /*if(liblte_x2ap_unpack_cause(ptr, &msg->Cause) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_TARGETCELL_ID == ie_id) {
        /*if(liblte_x2ap_unpack_ecgi(ptr, &msg->TargetCell_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_GUMMEI_ID == ie_id) {
        /*if(liblte_x2ap_unpack_gummei(ptr, &msg->GUMMEI_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_UE_CONTEXTINFORMATION == ie_id) {
        /*if(liblte_x2ap_unpack_ue_contextinformation(ptr, &msg->UE_ContextInformation) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_UE_HISTORYINFORMATION == ie_id) {
        /*if(liblte_x2ap_unpack_ue_historyinformation(ptr, &msg->UE_HistoryInformation) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_TRACEACTIVATION == ie_id) {
        /*if(liblte_x2ap_unpack_traceactivation(ptr, &msg->TraceActivation) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->TraceActivation_present = true;
      } else      if(LIBLTE_X2AP_IE_ID_SRVCCOPERATIONPOSSIBLE == ie_id) {
        /*if(liblte_x2ap_unpack_srvccoperationpossible(ptr, &msg->SRVCCOperationPossible) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->SRVCCOperationPossible_present = true;
      } else      if(LIBLTE_X2AP_IE_ID_CSGMEMBERSHIPSTATUS == ie_id) {
        /*if(liblte_x2ap_unpack_csgmembershipstatus(ptr, &msg->CSGMembershipStatus) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->CSGMembershipStatus_present = true;
      } else      if(LIBLTE_X2AP_IE_ID_MOBILITYINFORMATION == ie_id) {
        /*if(liblte_x2ap_unpack_mobilityinformation(ptr, &msg->MobilityInformation) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->MobilityInformation_present = true;
      } 
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* Protocol Message X2SetupRequest STRUCT
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_x2setuprequest(
  LIBLTE_X2AP_MESSAGE_X2SETUPREQUEST_STRUCT                          *msg,
  uint8_t                                                           **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {

    // Extension
    liblte_value_2_bits(msg->ext?1:0, ptr, 1);
    liblte_align_up_zero(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("X2SetupRequestIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_ENCODE_FAIL;
    }

    // No. of ProtocolIE
    uint32_t n_ie = 3;
    if(!msg->GUGroupIDList_present)
      n_ie--;
    liblte_value_2_bits(n_ie, ptr, 16);

    // Temp container for IEs
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t              *tmp_ptr;

    // ProtocolIE - GlobalENB_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_globalenb_id(&msg->GlobalENB_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_GLOBALENB_ID,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - ServedCells
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_servedcells(&msg->ServedCells, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_SERVEDCELLS,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - GUGroupIDList
    if(msg->GUGroupIDList_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_gugroupidlist(&msg->GUGroupIDList, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_GUGROUPIDLIST,
                                            LIBLTE_X2AP_CRITICALITY_REJECT,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_x2setuprequest(
  uint8_t                                                           **ptr,
  LIBLTE_X2AP_MESSAGE_X2SETUPREQUEST_STRUCT                          *msg)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {
    LIBLTE_X2AP_CRITICALITY_ENUM crit;
    uint32_t ie_id;
    uint32_t len;
    uint32_t n_ie;
    uint32_t i;

    // Set booleans
    msg->GUGroupIDList_present = false;

    // Extension
    msg->ext  = liblte_bits_2_value(ptr, 1);
    liblte_align_up(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("X2SetupRequestIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_DECODE_FAIL;
    }

    // No. of ProtocolIE-Container
    n_ie = liblte_bits_2_value(ptr, 16);

    // Unpack ProtocolIE Fields
    for(i=0;i<n_ie;i++) {
      if(liblte_x2ap_unpack_protocolie_header(ptr, &ie_id, &crit, &len) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
      if(LIBLTE_X2AP_IE_ID_GLOBALENB_ID == ie_id) {
        /*if(liblte_x2ap_unpack_globalenb_id(ptr, &msg->GlobalENB_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_SERVEDCELLS == ie_id) {
        /*if(liblte_x2ap_unpack_servedcells(ptr, &msg->ServedCells) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_GUGROUPIDLIST == ie_id) {
        /*if(liblte_x2ap_unpack_gugroupidlist(ptr, &msg->GUGroupIDList) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->GUGroupIDList_present = true;
      } 
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* Protocol Message HandoverRequestAcknowledge STRUCT
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_handoverrequestacknowledge(
  LIBLTE_X2AP_MESSAGE_HANDOVERREQUESTACKNOWLEDGE_STRUCT              *msg,
  uint8_t                                                           **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {

    // Extension
    liblte_value_2_bits(msg->ext?1:0, ptr, 1);
    liblte_align_up_zero(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("HandoverRequestAcknowledgeIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_ENCODE_FAIL;
    }

    // No. of ProtocolIE
    uint32_t n_ie = 6;
    if(!msg->E_RABs_NotAdmitted_List_present)
      n_ie--;
    if(!msg->CriticalityDiagnostics_present)
      n_ie--;
    liblte_value_2_bits(n_ie, ptr, 16);

    // Temp container for IEs
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t              *tmp_ptr;

    // ProtocolIE - Old_eNB_UE_X2AP_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ue_x2ap_id(&msg->Old_eNB_UE_X2AP_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_UE_X2AP_ID,
                                          LIBLTE_X2AP_CRITICALITY_IGNORE,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - New_eNB_UE_X2AP_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ue_x2ap_id(&msg->New_eNB_UE_X2AP_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_UE_X2AP_ID,
                                          LIBLTE_X2AP_CRITICALITY_IGNORE,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - E_RABs_Admitted_List
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_e_rabs_admitted_list(&msg->E_RABs_Admitted_List, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_E_RABS_ADMITTED_LIST,
                                          LIBLTE_X2AP_CRITICALITY_IGNORE,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - E_RABs_NotAdmitted_List
    if(msg->E_RABs_NotAdmitted_List_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_e_rab_list(&msg->E_RABs_NotAdmitted_List, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_E_RABS_NOTADMITTED_LIST,
                                            LIBLTE_X2AP_CRITICALITY_IGNORE,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    // ProtocolIE - TargeteNBtoSource_eNBTransparentContainer
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_targetenbtosource_enbtransparentcontainer(&msg->TargeteNBtoSource_eNBTransparentContainer, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_TARGETENBTOSOURCE_ENBTRANSPARENTCONTAINER,
                                          LIBLTE_X2AP_CRITICALITY_IGNORE,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - CriticalityDiagnostics
    if(msg->CriticalityDiagnostics_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_criticalitydiagnostics(&msg->CriticalityDiagnostics, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_CRITICALITYDIAGNOSTICS,
                                            LIBLTE_X2AP_CRITICALITY_IGNORE,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handoverrequestacknowledge(
  uint8_t                                                           **ptr,
  LIBLTE_X2AP_MESSAGE_HANDOVERREQUESTACKNOWLEDGE_STRUCT              *msg)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {
    LIBLTE_X2AP_CRITICALITY_ENUM crit;
    uint32_t ie_id;
    uint32_t len;
    uint32_t n_ie;
    uint32_t i;

    // Set booleans
    msg->E_RABs_NotAdmitted_List_present = false;
    msg->CriticalityDiagnostics_present = false;

    // Extension
    msg->ext  = liblte_bits_2_value(ptr, 1);
    liblte_align_up(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("HandoverRequestAcknowledgeIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_DECODE_FAIL;
    }

    // No. of ProtocolIE-Container
    n_ie = liblte_bits_2_value(ptr, 16);

    // Unpack ProtocolIE Fields
    bool old = false;

    for(i=0;i<n_ie;i++) {
      if(liblte_x2ap_unpack_protocolie_header(ptr, &ie_id, &crit, &len) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
      if(LIBLTE_X2AP_IE_ID_UE_X2AP_ID == ie_id && !old) {
        /*if(liblte_x2ap_unpack_ue_x2ap_id(ptr, &msg->Old_eNB_UE_X2AP_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        old = true;
      } else      if(LIBLTE_X2AP_IE_ID_UE_X2AP_ID == ie_id && old) {
        /*if(liblte_x2ap_unpack_ue_x2ap_id(ptr, &msg->New_eNB_UE_X2AP_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_E_RABS_ADMITTED_LIST == ie_id) {
        /*if(liblte_x2ap_unpack_e_rabs_admitted_list(ptr, &msg->E_RABs_Admitted_List) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_E_RABS_NOTADMITTED_LIST == ie_id) {
        /*if(liblte_x2ap_unpack_e_rab_list(ptr, &msg->E_RABs_NotAdmitted_List) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->E_RABs_NotAdmitted_List_present = true;
      } else      if(LIBLTE_X2AP_IE_ID_TARGETENBTOSOURCE_ENBTRANSPARENTCONTAINER == ie_id) {
        /*if(liblte_x2ap_unpack_targetenbtosource_enbtransparentcontainer(ptr, &msg->TargeteNBtoSource_eNBTransparentContainer) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_CRITICALITYDIAGNOSTICS == ie_id) {
        /*if(liblte_x2ap_unpack_criticalitydiagnostics(ptr, &msg->CriticalityDiagnostics) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->CriticalityDiagnostics_present = true;
      } 
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* Protocol Message X2SetupResponse STRUCT
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_x2setupresponse(
  LIBLTE_X2AP_MESSAGE_X2SETUPRESPONSE_STRUCT                         *msg,
  uint8_t                                                           **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {

    // Extension
    liblte_value_2_bits(msg->ext?1:0, ptr, 1);
    liblte_align_up_zero(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("X2SetupResponseIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_ENCODE_FAIL;
    }

    // No. of ProtocolIE
    uint32_t n_ie = 4;
    if(!msg->GUGroupIDList_present)
      n_ie--;
    if(!msg->CriticalityDiagnostics_present)
      n_ie--;
    liblte_value_2_bits(n_ie, ptr, 16);

    // Temp container for IEs
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t              *tmp_ptr;

	// ProtocolIE - GlobalENB_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_globalenb_id(&msg->GlobalENB_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_GLOBALENB_ID,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

	// ProtocolIE - ServedCells
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_servedcells(&msg->ServedCells, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_SERVEDCELLS,
                                          LIBLTE_X2AP_CRITICALITY_REJECT,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - GUGroupIDList
    if(msg->GUGroupIDList_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_gugroupidlist(&msg->GUGroupIDList, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_GUGROUPIDLIST,
                                            LIBLTE_X2AP_CRITICALITY_REJECT,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    // ProtocolIE - CriticalityDiagnostics
    if(msg->CriticalityDiagnostics_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_criticalitydiagnostics(&msg->CriticalityDiagnostics, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_CRITICALITYDIAGNOSTICS,
                                            LIBLTE_X2AP_CRITICALITY_IGNORE,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_x2setupresponse(
  uint8_t                                                           **ptr,
  LIBLTE_X2AP_MESSAGE_X2SETUPRESPONSE_STRUCT                         *msg)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {
    LIBLTE_X2AP_CRITICALITY_ENUM crit;
    uint32_t ie_id;
    uint32_t len;
    uint32_t n_ie;
    uint32_t i;

    // Set booleans
    msg->GUGroupIDList_present = false;
    msg->CriticalityDiagnostics_present = false;

    // Extension
    msg->ext  = liblte_bits_2_value(ptr, 1);
    liblte_align_up(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("X2SetupResponseIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_DECODE_FAIL;
    }

    // No. of ProtocolIE-Container
    n_ie = liblte_bits_2_value(ptr, 16);

    // Unpack ProtocolIE Fields
    for(i=0;i<n_ie;i++) {
      if(liblte_x2ap_unpack_protocolie_header(ptr, &ie_id, &crit, &len) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
      if(LIBLTE_X2AP_IE_ID_GLOBALENB_ID == ie_id) {
        /*if(liblte_x2ap_unpack_globalenb_id(ptr, &msg->GlobalENB_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_SERVEDCELLS == ie_id) {
        /*if(liblte_x2ap_unpack_servedcells(ptr, &msg->ServedCells) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_GUGROUPIDLIST == ie_id) {
        /*if(liblte_x2ap_unpack_gugroupidlist(ptr, &msg->GUGroupIDList) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->GUGroupIDList_present = true;
      } else      if(LIBLTE_X2AP_IE_ID_CRITICALITYDIAGNOSTICS == ie_id) {
        /*if(liblte_x2ap_unpack_criticalitydiagnostics(ptr, &msg->CriticalityDiagnostics) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->CriticalityDiagnostics_present = true;
      } 
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* Protocol Message HandoverPreparationFailure STRUCT
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_handoverpreparationfailure(
  LIBLTE_X2AP_MESSAGE_HANDOVERPREPARATIONFAILURE_STRUCT              *msg,
  uint8_t                                                           **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {

    // Extension
    liblte_value_2_bits(msg->ext?1:0, ptr, 1);
    liblte_align_up_zero(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("HandoverPreparationFailureIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_ENCODE_FAIL;
    }

    // No. of ProtocolIE
    uint32_t n_ie = 3;
    if(!msg->CriticalityDiagnostics_present)
      n_ie--;
    liblte_value_2_bits(n_ie, ptr, 16);

    // Temp container for IEs
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t              *tmp_ptr;

    // ProtocolIE - Old_eNB_UE_X2AP_ID
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_ue_x2ap_id(&msg->Old_eNB_UE_X2AP_ID, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_UE_X2AP_ID,
                                          LIBLTE_X2AP_CRITICALITY_IGNORE,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - Cause
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_cause(&msg->Cause, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_CAUSE,
                                          LIBLTE_X2AP_CRITICALITY_IGNORE,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - CriticalityDiagnostics
    if(msg->CriticalityDiagnostics_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_criticalitydiagnostics(&msg->CriticalityDiagnostics, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_CRITICALITYDIAGNOSTICS,
                                            LIBLTE_X2AP_CRITICALITY_IGNORE,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_handoverpreparationfailure(
  uint8_t                                                           **ptr,
  LIBLTE_X2AP_MESSAGE_HANDOVERPREPARATIONFAILURE_STRUCT              *msg)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {
    LIBLTE_X2AP_CRITICALITY_ENUM crit;
    uint32_t ie_id;
    uint32_t len;
    uint32_t n_ie;
    uint32_t i;

    // Set booleans
    msg->CriticalityDiagnostics_present = false;

    // Extension
    msg->ext  = liblte_bits_2_value(ptr, 1);
    liblte_align_up(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("HandoverPreparationFailureIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_DECODE_FAIL;
    }

    // No. of ProtocolIE-Container
    n_ie = liblte_bits_2_value(ptr, 16);

    // Unpack ProtocolIE Fields
    for(i=0;i<n_ie;i++) {
      if(liblte_x2ap_unpack_protocolie_header(ptr, &ie_id, &crit, &len) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
      if(LIBLTE_X2AP_IE_ID_UE_X2AP_ID == ie_id) {
        /*if(liblte_x2ap_unpack_ue_x2ap_id(ptr, &msg->Old_eNB_UE_X2AP_ID) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_CAUSE == ie_id) {
        /*if(liblte_x2ap_unpack_cause(ptr, &msg->Cause) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_CRITICALITYDIAGNOSTICS == ie_id) {
        /*if(liblte_x2ap_unpack_criticalitydiagnostics(ptr, &msg->CriticalityDiagnostics) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->CriticalityDiagnostics_present = true;
      } 
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* Protocol Message X2SetupFailure STRUCT
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_x2setupfailure(
  LIBLTE_X2AP_MESSAGE_X2SETUPFAILURE_STRUCT                          *msg,
  uint8_t                                                           **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {

    // Extension
    liblte_value_2_bits(msg->ext?1:0, ptr, 1);
    liblte_align_up_zero(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("X2SetupFailureIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_ENCODE_FAIL;
    }

    // No. of ProtocolIE
    uint32_t n_ie = 3;
    if(!msg->TimeToWait_present)
      n_ie--;
    if(!msg->GUGroupIDList_present)
      n_ie--;
    liblte_value_2_bits(n_ie, ptr, 16);

    // Temp container for IEs
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t              *tmp_ptr;

    // ProtocolIE - Cause
    tmp_msg.reset();
    tmp_ptr = tmp_msg.msg;
    /*if(liblte_x2ap_pack_cause(&msg->Cause, &tmp_ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }*/
    liblte_align_up_zero(&tmp_ptr, 8);
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
    if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                          LIBLTE_X2AP_IE_ID_CAUSE,
                                          LIBLTE_X2AP_CRITICALITY_IGNORE,
                                          ptr) != LIBLTE_SUCCESS) {
      return LIBLTE_ERROR_ENCODE_FAIL;
    }
    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    // ProtocolIE - TimeToWait
    if(msg->TimeToWait_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_timetowait(&msg->TimeToWait, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_TIMETOWAIT,
                                            LIBLTE_X2AP_CRITICALITY_IGNORE,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    // ProtocolIE - GUGroupIDList
    if(msg->GUGroupIDList_present) {
      tmp_msg.reset();
      tmp_ptr = tmp_msg.msg;
      /*if(liblte_x2ap_pack_gugroupidlist(&msg->GUGroupIDList, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }*/
      liblte_align_up_zero(&tmp_ptr, 8);
      tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;
      if(liblte_x2ap_pack_protocolie_header(tmp_msg.N_bits / 8,
                                            LIBLTE_X2AP_IE_ID_GUGROUPIDLIST,
                                            LIBLTE_X2AP_CRITICALITY_IGNORE,
                                            ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
      memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
      *ptr += tmp_msg.N_bits;
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_x2setupfailure(
  uint8_t                                                           **ptr,
  LIBLTE_X2AP_MESSAGE_X2SETUPFAILURE_STRUCT                          *msg)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg != NULL &&
     ptr != NULL)
  {
    LIBLTE_X2AP_CRITICALITY_ENUM crit;
    uint32_t ie_id;
    uint32_t len;
    uint32_t n_ie;
    uint32_t i;

    // Set booleans
    msg->TimeToWait_present = false;
    msg->GUGroupIDList_present = false;

    // Extension
    msg->ext  = liblte_bits_2_value(ptr, 1);
    liblte_align_up(ptr, 8);
    if(msg->ext) {
      liblte_x2ap_log_print("X2SetupFailureIEs error: X2AP ASN extensions not currently supported\n");
      return LIBLTE_ERROR_DECODE_FAIL;
    }

    // No. of ProtocolIE-Container
    n_ie = liblte_bits_2_value(ptr, 16);

    // Unpack ProtocolIE Fields
    for(i=0;i<n_ie;i++) {
      if(liblte_x2ap_unpack_protocolie_header(ptr, &ie_id, &crit, &len) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
      if(LIBLTE_X2AP_IE_ID_CAUSE == ie_id) {
        /*if(liblte_x2ap_unpack_cause(ptr, &msg->Cause) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
      } else      if(LIBLTE_X2AP_IE_ID_TIMETOWAIT == ie_id) {
        /*if(liblte_x2ap_unpack_timetowait(ptr, &msg->TimeToWait) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->TimeToWait_present = true;
      } else      if(LIBLTE_X2AP_IE_ID_GUGROUPIDLIST == ie_id) {
        /*if(liblte_x2ap_unpack_gugroupidlist(ptr, &msg->GUGroupIDList) != LIBLTE_SUCCESS) {
          return LIBLTE_ERROR_DECODE_FAIL;
        }*/
        liblte_align_up(ptr, 8);
        msg->GUGroupIDList_present = true;
      } 
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* InitiatingMessage CHOICE
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_initiatingmessage(
  LIBLTE_X2AP_INITIATINGMESSAGE_STRUCT *msg,
  uint8_t                             **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg                                      != NULL &&
     ptr                                      != NULL)
  { 
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t *tmp_ptr = tmp_msg.msg;

    // Message
    if(msg->choice_type == LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_X2SETUPREQUEST) {
      if(liblte_x2ap_pack_x2setuprequest(&msg->choice.X2SetupRequest, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
    } else    if(msg->choice_type == LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_HANDOVERREQUEST) {
      if(liblte_x2ap_pack_handoverrequest(&msg->choice.HandoverRequest, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
    } else    if(msg->choice_type == LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_SNSTATUSTRANSFER) {
      if(liblte_x2ap_pack_snstatustransfer(&msg->choice.SNStatusTransfer, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
    } else    if(msg->choice_type == LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_UECONTEXTRELEASE) {
      if(liblte_x2ap_pack_uecontextrelease(&msg->choice.UEContextRelease, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
    } 
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;

    // Procedure code
    liblte_value_2_bits(msg->procedureCode, ptr, 8);

    // Criticality
    LIBLTE_X2AP_CRITICALITY_ENUM crit = liblte_x2ap_procedure_criticality[msg->procedureCode];
    liblte_value_2_bits(crit, ptr, 2);
    liblte_align_up_zero(ptr, 8);

    // Length
    uint32_t len = (tmp_msg.N_bits + 7) / 8;
    if(len < 128) {
      liblte_value_2_bits(0,   ptr, 1);
      liblte_value_2_bits(len, ptr, 7);
    } else if(len < 16383) {
      liblte_value_2_bits(1,   ptr, 1);
      liblte_value_2_bits(0,   ptr, 1);
      liblte_value_2_bits(len, ptr, 14);
    } else {
      // FIXME: Unlikely to have more than 16K of octets
    }

    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_initiatingmessage(
  uint8_t                             **ptr,
  LIBLTE_X2AP_INITIATINGMESSAGE_STRUCT *msg)
{  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg                                      != NULL &&
     ptr                                      != NULL)
  {
    // Procedure code
    msg->procedureCode  = liblte_bits_2_value(ptr, 8);

    // Criticality
    msg->criticality    = (LIBLTE_X2AP_CRITICALITY_ENUM)liblte_bits_2_value(ptr, 2);
    liblte_align_up(ptr, 8);

    // Length
    uint32_t len = 0;
    if(0 == liblte_bits_2_value(ptr, 1)) {
      len = liblte_bits_2_value(ptr, 7);
    } else {
      if(0 == liblte_bits_2_value(ptr, 1)) {
        len = liblte_bits_2_value(ptr, 14);
      } else {
        // FIXME: Unlikely to have more than 16K of octets
      }
    }

    // Message
    if(msg->procedureCode == LIBLTE_X2AP_PROC_ID_X2SETUP) {
      msg->choice_type = LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_X2SETUPREQUEST;
      if(liblte_x2ap_unpack_x2setuprequest(ptr, &msg->choice.X2SetupRequest) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    } else    if(msg->procedureCode == LIBLTE_X2AP_PROC_ID_HANDOVERPREPARATION) {
      msg->choice_type = LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_HANDOVERREQUEST;
      if(liblte_x2ap_unpack_handoverrequest(ptr, &msg->choice.HandoverRequest) != LIBLTE_SUCCESS) 
      {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    } else    if(msg->procedureCode == LIBLTE_X2AP_PROC_ID_SNSTATUSTRANSFER) {
      msg->choice_type = LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_SNSTATUSTRANSFER;
      if(liblte_x2ap_unpack_snstatustransfer(ptr, &msg->choice.SNStatusTransfer) != LIBLTE_SUCCESS) 
      {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    } else    if(msg->procedureCode == LIBLTE_X2AP_PROC_ID_UECONTEXTRELEASE) {
      msg->choice_type = LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_UECONTEXTRELEASE;
      if(liblte_x2ap_unpack_uecontextrelease(ptr, &msg->choice.UEContextRelease) != LIBLTE_SUCCESS) 
      {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    } 
    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* SuccessfulOutcome CHOICE
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_successfuloutcome(
  LIBLTE_X2AP_SUCCESSFULOUTCOME_STRUCT *msg,
  uint8_t                             **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg                                      != NULL &&
     ptr                                      != NULL)
  { 
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t *tmp_ptr = tmp_msg.msg;

    // Message
    if(msg->choice_type == LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_HANDOVERREQUESTACKNOWLEDGE) {
      if(liblte_x2ap_pack_handoverrequestacknowledge(&msg->choice.HandoverRequestAcknowledge, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
    } else    if(msg->choice_type == LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_X2SETUPRESPONSE) {
      if(liblte_x2ap_pack_x2setupresponse(&msg->choice.X2SetupResponse, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
    } 
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;

    // Procedure code
    liblte_value_2_bits(msg->procedureCode, ptr, 8);

    // Criticality
    LIBLTE_X2AP_CRITICALITY_ENUM crit = liblte_x2ap_procedure_criticality[msg->procedureCode];
    liblte_value_2_bits(crit, ptr, 2);
    liblte_align_up_zero(ptr, 8);

    // Length
    uint32_t len = (tmp_msg.N_bits + 7) / 8;
    if(len < 128) {
      liblte_value_2_bits(0,   ptr, 1);
      liblte_value_2_bits(len, ptr, 7);
    } else if(len < 16383) {
      liblte_value_2_bits(1,   ptr, 1);
      liblte_value_2_bits(0,   ptr, 1);
      liblte_value_2_bits(len, ptr, 14);
    } else {
      // FIXME: Unlikely to have more than 16K of octets
    }

    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_successfuloutcome(
  uint8_t                             **ptr,
  LIBLTE_X2AP_SUCCESSFULOUTCOME_STRUCT *msg)
{  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg                                      != NULL &&
     ptr                                      != NULL)
  {
    // Procedure code
    msg->procedureCode  = liblte_bits_2_value(ptr, 8);

    // Criticality
    msg->criticality    = (LIBLTE_X2AP_CRITICALITY_ENUM)liblte_bits_2_value(ptr, 2);
    liblte_align_up(ptr, 8);

    // Length
    uint32_t len = 0;
    if(0 == liblte_bits_2_value(ptr, 1)) {
      len = liblte_bits_2_value(ptr, 7);
    } else {
      if(0 == liblte_bits_2_value(ptr, 1)) {
        len = liblte_bits_2_value(ptr, 14);
      } else {
        // FIXME: Unlikely to have more than 16K of octets
      }
    }

    // Message
    if(msg->procedureCode == LIBLTE_X2AP_PROC_ID_HANDOVERPREPARATION) {
      msg->choice_type = LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_HANDOVERREQUESTACKNOWLEDGE;
      if(liblte_x2ap_unpack_handoverrequestacknowledge(ptr, &msg->choice.HandoverRequestAcknowledge) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    } else    if(msg->procedureCode == LIBLTE_X2AP_PROC_ID_X2SETUP) {
      msg->choice_type = LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_X2SETUPRESPONSE;
      if(liblte_x2ap_unpack_x2setupresponse(ptr, &msg->choice.X2SetupResponse) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    }
    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* UnsuccessfulOutcome CHOICE
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_unsuccessfuloutcome(
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_STRUCT *msg,
  uint8_t                               **ptr)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg                                      != NULL &&
     ptr                                      != NULL)
  { 
    LIBLTE_BIT_MSG_STRUCT tmp_msg;
    uint8_t *tmp_ptr = tmp_msg.msg;

    // Message
    if(msg->choice_type == LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_HANDOVERPREPARATIONFAILURE) {
      if(liblte_x2ap_pack_handoverpreparationfailure(&msg->choice.HandoverPreparationFailure, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
    } else    if(msg->choice_type == LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_X2SETUPFAILURE) {
      if(liblte_x2ap_pack_x2setupfailure(&msg->choice.X2SetupFailure, &tmp_ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
    } 
    tmp_msg.N_bits = tmp_ptr - tmp_msg.msg;

    // Procedure code
    liblte_value_2_bits(msg->procedureCode, ptr, 8);

    // Criticality
    LIBLTE_X2AP_CRITICALITY_ENUM crit = liblte_x2ap_procedure_criticality[msg->procedureCode];
    liblte_value_2_bits(crit, ptr, 2);
    liblte_align_up_zero(ptr, 8);

    // Length
    uint32_t len = (tmp_msg.N_bits + 7) / 8;
    if(len < 128) {
      liblte_value_2_bits(0,   ptr, 1);
      liblte_value_2_bits(len, ptr, 7);
    } else if(len < 16383) {
      liblte_value_2_bits(1,   ptr, 1);
      liblte_value_2_bits(0,   ptr, 1);
      liblte_value_2_bits(len, ptr, 14);
    } else {
      // FIXME: Unlikely to have more than 16K of octets
    }

    memcpy(*ptr, tmp_msg.msg, tmp_msg.N_bits);
    *ptr += tmp_msg.N_bits;

    err = LIBLTE_SUCCESS;
  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_unsuccessfuloutcome(
  uint8_t                               **ptr,
  LIBLTE_X2AP_UNSUCCESSFULOUTCOME_STRUCT *msg)
{  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

  if(msg                                      != NULL &&
     ptr                                      != NULL)
  {
    // Procedure code
    msg->procedureCode  = liblte_bits_2_value(ptr, 8);

    // Criticality
    msg->criticality    = (LIBLTE_X2AP_CRITICALITY_ENUM)liblte_bits_2_value(ptr, 2);
    liblte_align_up(ptr, 8);

    // Length
    uint32_t len = 0;
    if(0 == liblte_bits_2_value(ptr, 1)) {
      len = liblte_bits_2_value(ptr, 7);
    } else {
      if(0 == liblte_bits_2_value(ptr, 1)) {
        len = liblte_bits_2_value(ptr, 14);
      } else {
        // FIXME: Unlikely to have more than 16K of octets
      }
    }

    // Message
    if(msg->procedureCode == LIBLTE_X2AP_PROC_ID_HANDOVERPREPARATION) {
      msg->choice_type = LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_HANDOVERPREPARATIONFAILURE;
      if(liblte_x2ap_unpack_handoverpreparationfailure(ptr, &msg->choice.HandoverPreparationFailure) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    } else    if(msg->procedureCode == LIBLTE_X2AP_PROC_ID_X2SETUP) {
      msg->choice_type = LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_X2SETUPFAILURE;
      if(liblte_x2ap_unpack_x2setupfailure(ptr, &msg->choice.X2SetupFailure) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    } 
    err = LIBLTE_SUCCESS;
  }
  return err;
}

/*******************************************************************************
/* X2AP_PDU CHOICE
********************************************************************************/
LIBLTE_ERROR_ENUM liblte_x2ap_pack_x2ap_pdu(
  LIBLTE_X2AP_X2AP_PDU_STRUCT *x2ap_pdu,
  LIBLTE_BYTE_MSG_STRUCT      *msg)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;
  LIBLTE_BIT_MSG_STRUCT bit_msg;

  if(x2ap_pdu                                 != NULL &&
     msg                                      != NULL)
  {
    uint8_t *p    = bit_msg.msg;
    uint8_t **ptr = &p;

    // Extension
    liblte_value_2_bits(x2ap_pdu->ext?1:0, ptr, 1);

    // Message choice
    liblte_value_2_bits(x2ap_pdu->choice_type, ptr, 2);
    liblte_align_up_zero(ptr, 8);

    // Message
    if(LIBLTE_X2AP_X2AP_PDU_CHOICE_INITIATINGMESSAGE == x2ap_pdu->choice_type) {
      if(liblte_x2ap_pack_initiatingmessage(&x2ap_pdu->choice.initiatingMessage, ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
    } else if(LIBLTE_X2AP_X2AP_PDU_CHOICE_SUCCESSFULOUTCOME == x2ap_pdu->choice_type) {
      if(liblte_x2ap_pack_successfuloutcome(&x2ap_pdu->choice.successfulOutcome, ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
     }else if(LIBLTE_X2AP_X2AP_PDU_CHOICE_UNSUCCESSFULOUTCOME == x2ap_pdu->choice_type) {
      if(liblte_x2ap_pack_unsuccessfuloutcome(&x2ap_pdu->choice.unsuccessfulOutcome, ptr) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_ENCODE_FAIL;
      }
    }

    liblte_align_up_zero(ptr, 8);
    bit_msg.N_bits += (*ptr - bit_msg.msg);

    liblte_pack(&bit_msg, msg);
    err = LIBLTE_SUCCESS;

  }
  return err;
}

LIBLTE_ERROR_ENUM liblte_x2ap_unpack_x2ap_pdu(
  LIBLTE_BYTE_MSG_STRUCT      *msg,
  LIBLTE_X2AP_X2AP_PDU_STRUCT *x2ap_pdu)
{
  LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;
  LIBLTE_BIT_MSG_STRUCT bit_msg;

  if(x2ap_pdu                                 != NULL &&
     msg                                      != NULL)
  {
    liblte_unpack(msg, &bit_msg);

    uint8_t *p    = bit_msg.msg;
    uint8_t **ptr = &p;

    // Extension
    x2ap_pdu->ext = liblte_bits_2_value(ptr, 1);

    // Message choice
    x2ap_pdu->choice_type = (LIBLTE_X2AP_X2AP_PDU_CHOICE_ENUM)liblte_bits_2_value(ptr, 2);
    liblte_align_up(ptr, 8);

    // Message
    if(LIBLTE_X2AP_X2AP_PDU_CHOICE_INITIATINGMESSAGE == x2ap_pdu->choice_type) {
      if(liblte_x2ap_unpack_initiatingmessage(ptr, &x2ap_pdu->choice.initiatingMessage) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    }else if(LIBLTE_X2AP_X2AP_PDU_CHOICE_SUCCESSFULOUTCOME == x2ap_pdu->choice_type) {
      if(liblte_x2ap_unpack_successfuloutcome(ptr, &x2ap_pdu->choice.successfulOutcome) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    }else if(LIBLTE_X2AP_X2AP_PDU_CHOICE_UNSUCCESSFULOUTCOME == x2ap_pdu->choice_type) {
      if(liblte_x2ap_unpack_unsuccessfuloutcome(ptr, &x2ap_pdu->choice.unsuccessfulOutcome) != LIBLTE_SUCCESS) {
        return LIBLTE_ERROR_DECODE_FAIL;
      }
    }

    err = LIBLTE_SUCCESS;
  }
  return err;
}