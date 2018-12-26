#ifndef ENB_X2AP_METRICS_H
#define ENB_X2AP_METRICS_H


namespace srsenb {

typedef enum{
  X2AP_ATTACHING = 0, // Attempting to create X2 connection
  X2AP_READY,         // X2 connected
  X2AP_ERROR          // Failure
}X2AP_STATUS_ENUM;

struct x2ap_metrics_t
{
  X2AP_STATUS_ENUM status;
};

} // namespace srsenb

#endif // ENB_X2AP_METRICS_H