#include "upper/x2ap.h"
#include "upper/common_enb.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/sctp.h>

namespace srsenb
{

bool x2ap::init(x2ap_args_t args_, rrc_interface_x2ap *rrc_, s1ap_interface_x2ap *s1ap_, srslte::log *x2ap_log_)
{
    rrc = rrc_;
    s1ap = s1ap_;
    args = args_;
    x2ap_log = x2ap_log_;

    pool                = srslte::byte_buffer_pool::get_instance();
    running             = false;
    neighbour_connected = false;
    //build_tai_cgi();

    start(X2AP_THREAD_PRIO);

    return true;
}

void x2ap::stop()
{
    if(running) 
    {
        running = false;
        thread_cancel();
        wait_thread_finish();
    }

    if(close(socket_fd) == -1) 
    {
        x2ap_log->error("Failed to close SCTP socket\n");
    }
    return;
}

void x2ap::get_metrics(x2ap_metrics_t &m)
{
    if(!running)
    {
        m.status = X2AP_ERROR;
        return;
    }
    if(neighbour_connected)
        m.status = X2AP_READY;
    else
        m.status = X2AP_ATTACHING;
    return;
}

void x2ap::run_thread()
{
	srslte::byte_buffer_t *pdu = pool_allocate;

    uint32_t sz = SRSLTE_MAX_BUFFER_SIZE_BYTES - SRSLTE_BUFFER_HEADER_OFFSET;
    running = true;

    // Connect to neighbour ENB
    while(running && !connect_neighbour())
    {
        x2ap_log->error("Failed to connect to neighbour ENB - retrying in 10 seconds\n");
        x2ap_log->console("Failed to connect to neighbour ENB - retrying in 10 seconds\n");
        sleep(10);
    }
    if(!setup_x2ap())
    {
        x2ap_log->error("X2 setup failed\n");
        x2ap_log->console("X2 setup failed\n");
        running = false;
        return;
    }

    while(running)
    {
        pdu->reset();
        pdu->N_bytes = recv(socket_fd, pdu->msg, sz, 0);

        if(pdu->N_bytes <= 0)
        {
            neighbour_connected = false;
            do
            {
                x2ap_log->error("Failed to connect to neighbour ENB - retrying in 10 seconds\n");
                x2ap_log->console("Failed to connect to neighbour ENB - retrying in 10 seconds\n");
                sleep(10);
            }while(running && !connect_neighbour());

            if(!setup_x2ap())
            {
                x2ap_log->error("X2 setup failed\n");
                x2ap_log->console("X2 setup failed\n");
                running = false;
                return;
            }
        }

        x2ap_log->info_hex(pdu->msg, pdu->N_bytes, "Received X2AP PDU");
        handle_x2ap_rx_pdu(pdu);
    }
}

bool x2ap::connect_neighbour()
{
    socket_fd = 0, conn_fd = 0;
    if(args.active_status == 1) // active connect
    {
        x2ap_log->info("Connecting to neighbour ENB actively: %s:%d\n", args.neighbour_addr.c_str(), X2AP_PORT);
        if((socket_fd = socket(ADDR_FAMILY, SOCK_TYPE, PROTO)) == -1)
        {
            x2ap_log->error("Failed to create X2AP socket\n");
            return false;
        }

        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = ADDR_FAMILY;
        local_addr.sin_port = 0; // Any local port will do
        if(inet_pton(AF_INET, args.gtp_bind_addr.c_str(), &(local_addr.sin_addr)) != 1)
        {
            x2ap_log->error("Error converting IP address (%s) to sockaddr_in structure\n", args.gtp_bind_addr.c_str());
            return false;
        }
        bind(socket_fd, (struct sockaddr*)&local_addr, sizeof(local_addr));

        memset(&neighbour_enb_addr, 0, sizeof(struct sockaddr_in));
        neighbour_enb_addr.sin_family = ADDR_FAMILY;
        neighbour_enb_addr.sin_port = htons(X2AP_PORT);
        if(inet_pton(AF_INET, args.neighbour_addr.c_str(), &(neighbour_enb_addr.sin_addr)) != 1)
        {
            x2ap_log->error("Error converting IP address (%s) to sockaddr_in structure\n", args.neighbour_addr.c_str());
            return false;
        }
        if(connect(socket_fd, (struct sockaddr*)&neighbour_enb_addr, sizeof(neighbour_enb_addr)) == -1)
        {
            x2ap_log->error("Failed to establish connection to neighbour ENB\n");
            return false;
        }
        x2ap_log->info("SCTP socket established with neighbour ENB\n");
        return true;
    }
    else if(args.active_status == 0) // passive connect
    {
        x2ap_log->info("Passively waiting for neighbour ENB connection\n");
        if((socket_fd = socket(ADDR_FAMILY, SOCK_TYPE, PROTO)) == -1)
        {
            x2ap_log->error("Failed to create X2AP socket\n");
            return false;
        }

        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = ADDR_FAMILY;
        local_addr.sin_port = htons(X2AP_PORT);
        if(inet_pton(AF_INET, args.gtp_bind_addr.c_str(), &(local_addr.sin_addr)) != 1)
        {
            x2ap_log->error("Error converting IP address (%s) to sockaddr_in structure\n", args.neighbour_addr.c_str());
            return false;
        }
        bind(socket_fd, (struct sockaddr*)&local_addr, sizeof(local_addr));

        if(listen(socket_fd,SOMAXCONN) != 0)
        {
            close(socket_fd);
            x2ap_log->error("Error in SCTP listen\n");
            return false;
        }

        conn_fd = accept(socket_fd, (struct sockaddr*)NULL, (socklen_t*)NULL);
        if(conn_fd == -1)
        {
            x2ap_log->error("Error in SCTP accept\n");
            close(conn_fd);
            close(socket_fd);
            return false;
        }

        x2ap_log->info("SCTP socket established with neighbour ENB\n");
        return true;
    }
    else
    {
        x2ap_log->error("Wrong active_status argument\n");
        return false;
    }
}

bool x2ap::setup_x2ap()
{
    uint32_t tmp32;
    uint16_t tmp16;
    srslte::byte_buffer_t msg;
    LIBLTE_X2AP_X2AP_PDU_STRUCT pdu;
    bzero(&pdu, sizeof(LIBLTE_X2AP_X2AP_PDU_STRUCT));

    pdu.choice_type = LIBLTE_X2AP_X2AP_PDU_CHOICE_INITIATINGMESSAGE;

    LIBLTE_X2AP_INITIATINGMESSAGE_STRUCT *init = &pdu.choice.initiatingMessage;
    init->procedureCode = LIBLTE_X2AP_PROC_ID_X2SETUP;
    init->choice_type = LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_X2SETUPREQUEST;

    LIBLTE_X2AP_MESSAGE_X2SETUPREQUEST_STRUCT *x2setup = &init->choice.X2SetupRequest;
    x2setup->ext = false;

    x2setup->GlobalENB_ID.ext = false;
    x2setup->GlobalENB_ID.iE_Extensions_present = false;
    uint32_t plmn;
    x2ap_mccmnc_to_plmn(args.mcc, args.mnc, &plmn);
    tmp32 = htonl(plmn);
    x2setup->GlobalENB_ID.pLMN_Identity.buffer[0] = ((uint8_t*)&tmp32)[1];
    x2setup->GlobalENB_ID.pLMN_Identity.buffer[1] = ((uint8_t*)&tmp32)[2];
    x2setup->GlobalENB_ID.pLMN_Identity.buffer[2] = ((uint8_t*)&tmp32)[3];

    x2setup->GlobalENB_ID.eNB_ID.ext = false;
    x2setup->GlobalENB_ID.eNB_ID.choice_type = LIBLTE_X2AP_ENB_ID_CHOICE_MACROENB_ID;
    tmp32 = htonl(args.enb_id);
    uint8_t  enb_id_bits[4*8];
    liblte_unpack((uint8_t*)&tmp32, 4, enb_id_bits);
    memcpy(x2setup->GlobalENB_ID.eNB_ID.choice.macroENB_ID.buffer, &enb_id_bits[32-LIBLTE_X2AP_MACROENB_ID_BIT_STRING_LEN], LIBLTE_X2AP_MACROENB_ID_BIT_STRING_LEN);
    
    x2setup->GUGroupIDList_present = false;

    x2setup->ServedCells.len = 1;
    x2setup->ServedCells.buffer[0].ext = false;
    x2setup->ServedCells.buffer[0].iE_Extensions_present = false;
    x2setup->ServedCells.buffer[0].neighbour_Info_present = false;

    x2setup->ServedCells.buffer[0].servedCellInfo.ext = false;
    x2setup->ServedCells.buffer[0].servedCellInfo.iE_Extensions_present = false;
    tmp32 = htonl(args.pci);
    x2setup->ServedCells.buffer[0].servedCellInfo.pCI.PCI = tmp32; // to be verifiled 
    x2setup->ServedCells.buffer[0].servedCellInfo.cellId.ext = false;
    x2setup->ServedCells.buffer[0].servedCellInfo.cellId.iE_Extensions_present = false;
    tmp32 = htonl(plmn);
    x2setup->ServedCells.buffer[0].servedCellInfo.cellId.pLMN_Identity.buffer[0] = ((uint8_t*)&tmp32)[1];
    x2setup->ServedCells.buffer[0].servedCellInfo.cellId.pLMN_Identity.buffer[1] = ((uint8_t*)&tmp32)[2];
    x2setup->ServedCells.buffer[0].servedCellInfo.cellId.pLMN_Identity.buffer[2] = ((uint8_t*)&tmp32)[3];
    uint8_t cell_id_bits[1*8];
    liblte_unpack(&args.cell_id, 1, cell_id_bits);
    memcpy(x2setup->ServedCells.buffer[0].servedCellInfo.cellId.eUTRANcellIdentifier.buffer, &enb_id_bits[32-LIBLTE_X2AP_MACROENB_ID_BIT_STRING_LEN], LIBLTE_X2AP_MACROENB_ID_BIT_STRING_LEN);
    memcpy(&x2setup->ServedCells.buffer[0].servedCellInfo.cellId.eUTRANcellIdentifier.buffer[LIBLTE_X2AP_MACROENB_ID_BIT_STRING_LEN], cell_id_bits, 8); 
    tmp16 = htons(args.tac);
    memcpy(x2setup->ServedCells.buffer[0].servedCellInfo.tAC.buffer, (uint8_t*)&tmp16, 2);
    x2setup->ServedCells.buffer[0].servedCellInfo.broadcastPLMNS.len = 1;
    tmp32 = htonl(plmn);
    x2setup->ServedCells.buffer[0].servedCellInfo.broadcastPLMNS.buffer[0].buffer[0] = ((uint8_t*)&tmp32)[1];
    x2setup->ServedCells.buffer[0].servedCellInfo.broadcastPLMNS.buffer[0].buffer[1] = ((uint8_t*)&tmp32)[2];
    x2setup->ServedCells.buffer[0].servedCellInfo.broadcastPLMNS.buffer[0].buffer[2] = ((uint8_t*)&tmp32)[3];
    //TODO: x2setup->ServedCells.buffer[0].servedCellInfo.eUTRA_Mode_Info 
    
    liblte_x2ap_pack_x2ap_pdu(&pdu, (LIBLTE_BYTE_MSG_STRUCT*)&msg);
    x2ap_log->info_hex(msg.msg, msg.N_bytes, "Sending x2SetupRequest");

    ssize_t n_sent = sctp_sendmsg(socket_fd, msg.msg, msg.N_bytes,
                                (struct sockaddr*)&neighbour_enb_addr, sizeof(struct sockaddr_in),
                                htonl(PPID), 0, NONUE_STREAM_ID, 0, 0);
    if(n_sent == -1) 
    {
        x2ap_log->error("Failed to send x2SetupRequest\n");
        return false;
    }
    return true;
}

bool x2ap::handle_x2ap_rx_pdu(srslte::byte_buffer_t *pdu)
{
    LIBLTE_X2AP_X2AP_PDU_STRUCT rx_pdu;

    if(liblte_x2ap_unpack_x2ap_pdu((LIBLTE_BYTE_MSG_STRUCT*)pdu, &rx_pdu) != LIBLTE_SUCCESS)
    {
        x2ap_log->error("Failed to unpack received PDU\n");
        return false;
    }

    switch(rx_pdu.choice_type)
    {
        case LIBLTE_X2AP_X2AP_PDU_CHOICE_INITIATINGMESSAGE:
            return handle_initiatingmessage(&rx_pdu.choice.initiatingMessage);
            break;
        case LIBLTE_X2AP_X2AP_PDU_CHOICE_SUCCESSFULOUTCOME:
            return handle_successfuloutcome(&rx_pdu.choice.successfulOutcome);
            break;
        case LIBLTE_X2AP_X2AP_PDU_CHOICE_UNSUCCESSFULOUTCOME:
            return handle_unsuccessfuloutcome(&rx_pdu.choice.unsuccessfulOutcome);
            break;
        default:
            x2ap_log->error("Unhandled PDU type %d\n", rx_pdu.choice_type);
            return false;
    }
    return true;
}

bool x2ap::handle_initiatingmessage(LIBLTE_X2AP_INITIATINGMESSAGE_STRUCT *msg)
{
    switch(msg->choice_type)
    {
        case LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_X2SETUPREQUEST:
            return handle_x2setuprequest(&msg->choice.X2SetupRequest);
            break;
        case LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_HANDOVERREQUEST:
            return handle_handoverrequest(&msg->choice.HandoverRequest);
            break;
        case LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_SNSTATUSTRANSFER:
            return handle_snstatustransfer(&msg->choice.SNStatusTransfer);
            break;
        case LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_UECONTEXTRELEASE:
            return handle_uecontextrelease(&msg->choice.UEContextRelease);
            break;
        default:
            x2ap_log->error("Unhandled intiating message: %s\n", liblte_x2ap_initiatingmessage_choice_text[msg->choice_type]);
            return false;
    }
    return true;
}

bool x2ap::handle_successfuloutcome(LIBLTE_X2AP_SUCCESSFULOUTCOME_STRUCT *msg)
{
    switch(msg->choice_type)
    {
        case LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_HANDOVERREQUESTACKNOWLEDGE:
            return handle_handoverrequestacknowledge(&msg->choice.HandoverRequestAcknowledge);
            break;
        case LIBLTE_X2AP_SUCCESSFULOUTCOME_CHOICE_X2SETUPRESPONSE:
            return handle_x2setupresponse(&msg->choice.X2SetupResponse);
            break;
        default:
            x2ap_log->error("Unhandled successful outcome message: %s\n", liblte_x2ap_successfuloutcome_choice_text[msg->choice_type]);
            return false;
    }
    return true;
}

bool x2ap::handle_unsuccessfuloutcome(LIBLTE_X2AP_UNSUCCESSFULOUTCOME_STRUCT *msg)
{
    switch(msg->choice_type)
    {
        case LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_HANDOVERPREPARATIONFAILURE:
            return handle_handoverpreparationfailure(&msg->choice.HandoverPreparationFailure);
            break;
        case LIBLTE_X2AP_UNSUCCESSFULOUTCOME_CHOICE_X2SETUPFAILURE:
            return handle_x2setupfailure(&msg->choice.X2SetupFailure);
            break;
        default:
            x2ap_log->error("Unhandled unsuccessful outcome message: %s\n", liblte_x2ap_unsuccessfuloutcome_choice_text[msg->choice_type]);
            return false;
    }
    return true;
}

bool x2ap::handle_x2setuprequest(LIBLTE_X2AP_MESSAGE_X2SETUPREQUEST_STRUCT *msg)
{
    x2ap_log->info("Received X2setuprequest\n");
    //TODO: send_x2setupresponse();
    return true;
}

bool x2ap::handle_handoverrequest(LIBLTE_X2AP_MESSAGE_HANDOVERREQUEST_STRUCT *msg)
{
    x2ap_log->info("Received Handoverrequest\n");
    //TODO: send_handoverrequestacknowledge();
    return true;
}

bool x2ap::handle_snstatustransfer(LIBLTE_X2AP_MESSAGE_SNSTATUSTRANSFER_STRUCT *msg)
{
    x2ap_log->info("Received Snstatustransfer\n");
    //TODO: send_uecontextrelease();
    return true;
}

bool x2ap::handle_uecontextrelease(LIBLTE_X2AP_MESSAGE_UECONTEXTRELEASE_STRUCT *msg)
{
    x2ap_log->info("Received Uecontextrelease\n");
    //TODO: release_ue_cxt();
    return true;
}

bool x2ap::handle_handoverrequestacknowledge(LIBLTE_X2AP_MESSAGE_HANDOVERREQUESTACKNOWLEDGE_STRUCT *msg)
{
    x2ap_log->info("Received Handoverrequestacknowledge\n");
    //TODO: send_snstatustransfer();
    return true;
}

bool x2ap::handle_x2setupresponse(LIBLTE_X2AP_MESSAGE_X2SETUPRESPONSE_STRUCT *msg)
{
    x2ap_log->info("Received X2SetupResponse\n");
    x2setupresponse = *msg;
    neighbour_connected = true;
    return true;
}

bool x2ap::handle_handoverpreparationfailure(LIBLTE_X2AP_MESSAGE_HANDOVERPREPARATIONFAILURE_STRUCT *msg)
{
    std::string cause = get_cause(&msg->Cause);
    x2ap_log->error("X2 Handover Preparation Failure. Cause: %s\n", cause.c_str());
    x2ap_log->console("X2 Handover Preparation Failure. Cause: %s\n", cause.c_str());
    return true;
}

bool x2ap::handle_x2setupfailure(LIBLTE_X2AP_MESSAGE_X2SETUPFAILURE_STRUCT *msg)
{
    std::string cause = get_cause(&msg->Cause);
    x2ap_log->error("X2 Setup Failure. Cause: %s\n", cause.c_str());
    x2ap_log->console("X2 Setup Failure. Cause: %s\n", cause.c_str());
    return true;
}

bool x2ap::send_handoverrequest()
{
    if(!neighbour_connected)
        return false;

    srslte::byte_buffer_t msg;

    LIBLTE_X2AP_X2AP_PDU_STRUCT pdu;
    bzero(&pdu, sizeof(LIBLTE_X2AP_X2AP_PDU_STRUCT));
    pdu.ext = false;
    pdu.choice_type = LIBLTE_X2AP_X2AP_PDU_CHOICE_INITIATINGMESSAGE;

    LIBLTE_X2AP_INITIATINGMESSAGE_STRUCT *init = &pdu.choice.initiatingMessage;
    init->procedureCode = LIBLTE_X2AP_PROC_ID_HANDOVERPREPARATION;
    init->choice_type = LIBLTE_X2AP_INITIATINGMESSAGE_CHOICE_HANDOVERREQUEST;

    LIBLTE_X2AP_MESSAGE_HANDOVERREQUEST_STRUCT *req = &(init->choice.HandoverRequest);
    req->ext = false;
    req->TraceActivation_present = false;
    req->SRVCCOperationPossible_present = false;
    req->CSGMembershipStatus_present = false;
    req->MobilityInformation_present = false;
    return true;
}

//RRC Interface
void x2ap::handover_start(uint16_t rnti)
{
    if(send_handoverrequest())
        x2ap_log->info("Sent X2 handover request\n");
    else
        x2ap_log->error("Failed to send X2 handover request\n");
}

std::string x2ap::get_cause(LIBLTE_X2AP_CAUSE_STRUCT *c)
{
  std::string cause = liblte_x2ap_cause_choice_text[c->choice_type];
  cause += " - ";
  switch(c->choice_type) {
  case LIBLTE_X2AP_CAUSE_CHOICE_RADIONETWORK:
    cause += liblte_x2ap_causeradionetwork_text[c->choice.radioNetwork.e];
    break;
  case LIBLTE_X2AP_CAUSE_CHOICE_TRANSPORT:
    cause += liblte_x2ap_causetransport_text[c->choice.transport.e];
    break;
  case LIBLTE_X2AP_CAUSE_CHOICE_PROTOCOL:
    cause += liblte_x2ap_causeprotocol_text[c->choice.protocol.e];
    break;
  case LIBLTE_X2AP_CAUSE_CHOICE_MISC:
    cause += liblte_x2ap_causemisc_text[c->choice.misc.e];
    break;
  default:
    cause += "unkown";
    break;
  }
  return cause;
}

}