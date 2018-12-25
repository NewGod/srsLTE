#ifndef X2AP_H
#define X2AP_H

#include "srslte/common/threads.h"
#include "srslte/common/log.h"
#include "srslte/common/buffer_pool.h"
#include "srslte/interfaces/enb_interfaces.h"

#include "srslte/asn1/liblte_x2ap.h"
#include "upper/x2ap_metrics.h"

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
}x2ap_args_t;

class x2ap
	: public x2ap_interface_rrc
	, public x2ap_interface_s1ap
	, public thread
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

	LIBLTE_X2AP_MESSAGE_X2SETUPRESPONSE_STRUCT x2setupresponse;

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
	bool send_handoverrequest();

	std::string get_cause(LIBLTE_X2AP_CAUSE_STRUCT *c);
};

} // namespace srsenb
#endif