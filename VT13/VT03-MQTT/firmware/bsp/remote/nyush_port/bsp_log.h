/* Count upstream warning/error calls without blocking UART callbacks on logging. */
#ifndef NYUSH_LOG_PORT_H
#define NYUSH_LOG_PORT_H
void BspRc_UpstreamWarning(void);
void BspRc_UpstreamLogError(void);
#define LOGWARNING(...) BspRc_UpstreamWarning()
#define LOGERROR(...) BspRc_UpstreamLogError()
#endif
