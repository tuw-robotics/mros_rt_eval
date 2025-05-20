#ifndef MROS_UART_TRANSPORT
#define MROS_UART_TRANSPORT

#include <uxr/client/transport.h>

bool serial_open(struct uxrCustomTransport *transport);
bool serial_close(struct uxrCustomTransport *transport);
size_t serial_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t serial_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);


#endif // MROS_UART_TRANSPORT