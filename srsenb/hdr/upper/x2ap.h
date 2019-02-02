#ifndef X2AP_H
#define X2AP_H
#include <map>
#include "srslte/common/threads.h"
#include "srslte/common/log.h"
#include "srslte/common/buffer_pool.h"
#include "srslte/interfaces/enb_interfaces.h"

#include "srslte/asn1/liblte_x2ap.h"
#include "x2ap_metrics.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/sctp.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <unistd.h>

namespace srsenb
{

typedef struct 
{
	uint32_t      enb_id;     // 20-bit id (lsb bits)
	uint32_t      pci;        // 32-bit pci
	uint8_t       cell_id;    // 8-bit cell id 
	uint16_t      tac;        // 16-bit tac
	uint16_t      mcc;        // BCD-coded with 0xF filler
	uint16_t      mnc;        // BCD-coded with 0xF filler
	uint8_t       active_status; // 0: passive connect 1: active connect
	std::string   neighbour_addr;
	std::string   gtp_bind_addr;
	uint16_t      local_port;
	uint16_t      neigh_port;
}x2ap_args_t;

typedef struct {
  uint32_t  rnti;
  uint32_t  eNB_UE_X2AP_ID;
  uint32_t  MME_UE_X2AP_ID;
  bool      release_requested;
  uint16_t  stream_id;
}x2_ue_ctxt_t;

class x2ap : public x2ap_interface_rrc , public x2ap_interface_s1ap, public thread
{
public:
	bool init(x2ap_args_t args_, rrc_interface_x2ap *rrc_, s1ap_interface_x2ap *s1ap_, srslte::log *x2ap_log_);
	void stop();
	void get_metrics(x2ap_metrics_t &m);

	void run_thread();


	//RRC Interface
	void handover_start(uint16_t rnti);

private:
	static const int X2AP_THREAD_PRIO = 65;
	static const int X2AP_PORT        = 12901;
	static const int ADDR_FAMILY      = AF_INET;
	static const int SOCK_TYPE        = SOCK_STREAM;
	static const int PROTO            = IPPROTO_SCTP;
	static const int PPID             = 20;
	static const int NONUE_STREAM_ID  = 0;

	rrc_interface_x2ap         *rrc;
	s1ap_interface_x2ap        *s1ap;
	x2ap_args_t                args;
	srslte::log                *x2ap_log;
	srslte::byte_buffer_pool   *pool;

	bool neighbour_connected; // whether connected to a neighbour ENB
	bool running;
	int socket_fd, conn_fd;
	struct sockaddr_in neighbour_enb_addr; // Neighbour ENB address
	uint32_t  next_eNB_UE_X2AP_ID;    // Next ENB-side UE identifier
    uint16_t  next_ue_stream_id;      // Next UE SCTP stream identifier

    // Protocol IEs sent with every UL S1AP message
    LIBLTE_S1AP_TAI_STRUCT        tai;
    LIBLTE_S1AP_EUTRAN_CGI_STRUCT eutran_cgi;
    std::map<uint16_t, x2_ue_ctxt_t> ue_ctxt_map;
    std::map<uint32_t, uint16_t>  enbid_to_rnti_map;

	LIBLTE_X2AP_MESSAGE_X2SETUPRESPONSE_STRUCT x2setupresponse;

	//void build_tai_cgi();
	bool connect_neighbour();
	bool setup_x2ap();

	// Handle X2AP signaling messages
	bool handle_x2ap_rx_pdu(srslte::byte_buffer_t *pdu);
	bool handle_initiatingmessage(LIBLTE_X2AP_INITIATINGMESSAGE_STRUCT *msg);
	bool handle_successfuloutcome(LIBLTE_X2AP_SUCCESSFULOUTCOME_STRUCT *msg);
	bool handle_unsuccessfuloutcome(LIBLTE_X2AP_UNSUCCESSFULOUTCOME_STRUCT *msg);

	// X2AP initiatingmessages
	bool handle_x2setuprequest(LIBLTE_X2AP_MESSAGE_X2SETUPREQUEST_STRUCT *msg);
	bool handle_handoverrequest(LIBLTE_X2AP_MESSAGE_HANDOVERREQUEST_STRUCT *msg);
	bool handle_snstatustransfer(LIBLTE_X2AP_MESSAGE_SNSTATUSTRANSFER_STRUCT *msg);
	bool handle_uecontextrelease(LIBLTE_X2AP_MESSAGE_UECONTEXTRELEASE_STRUCT *msg);

	// X2AP successfuloutcomes
	bool handle_handoverrequestacknowledge(LIBLTE_X2AP_MESSAGE_HANDOVERREQUESTACKNOWLEDGE_STRUCT *msg);
	bool handle_x2setupresponse(LIBLTE_X2AP_MESSAGE_X2SETUPRESPONSE_STRUCT *msg);

	// X2AP unsuccessfuloutcomes
	bool handle_handoverpreparationfailure(LIBLTE_X2AP_MESSAGE_HANDOVERPREPARATIONFAILURE_STRUCT *msg);
	bool handle_x2setupfailure(LIBLTE_X2AP_MESSAGE_X2SETUPFAILURE_STRUCT *msg);

	// X2AP send messages
	bool send_x2setuprequest();
	bool send_x2setupresponse(LIBLTE_X2AP_MESSAGE_X2SETUPREQUEST_STRUCT *msg1);
	bool send_handoverrequest();
	bool send_handoverrequestacknowledge(LIBLTE_X2AP_MESSAGE_HANDOVERREQUEST_STRUCT *msg1);
	bool send_snstatustransfer(LIBLTE_X2AP_MESSAGE_HANDOVERREQUESTACKNOWLEDGE_STRUCT *msg1);
	bool send_uecontextrelease(LIBLTE_X2AP_MESSAGE_SNSTATUSTRANSFER_STRUCT *msg1);
	bool release_ue_cxt(LIBLTE_X2AP_MESSAGE_UECONTEXTRELEASE_STRUCT *msg1);

	std::string get_cause(LIBLTE_X2AP_CAUSE_STRUCT *c);
};

} // namespace srsenb
#endif
