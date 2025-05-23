#ifndef USART_H
#define USART_H

typedef void (CpltCallback_t)(void *);

static inline void usartRegisterHandlers(void *port, CpltCallback_t *rx, CpltCallback_t *tx, CpltCallback_t *err) {
    (void)port; (void)rx; (void)tx; (void)err;
}
static inline void usartUnregisterHandlers(void *port) { (void)port; }

#endif
