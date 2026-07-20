/*
 * nomodem.c -- LD_PRELOAD shim to run a serial IPCC tool over a pty.
 *
 * sp-emu exposes the host-SP UART (host_sp_comms / IPCC) as a pty when started
 * with SP_EMU_HOST_PTY=1. Serial tools such as `faux-ipcc` open that pty and,
 * via serial2, assert the DTR modem-control line on open (ioctl TIOCMBIS). A pty
 * has no modem-control lines, so the ioctl fails with ENOTTY ("Not a
 * typewriter") and the tool aborts.
 *
 * Baud and modem-control signals are meaningless over the emulator's byte pipe,
 * so this shim intercepts the modem-control ioctls (TIOCMGET/TIOCMBIS/TIOCMBIC/
 * TIOCMSET) and reports success with nothing asserted, passing everything else
 * through. That lets the unmodified tool talk to sp-emu's pty.
 *
 * Build:  cc -shared -fPIC -o nomodem.so nomodem.c -ldl
 * Use:    LD_PRELOAD=./nomodem.so faux-ipcc --port <sp-emu pty> \
 *             --read-timeout-ms 5000 status
 *
 * This is a workaround for using a hardware-oriented serial tool against a
 * virtual port; the alternative is a small change in the tool/serial2 to skip
 * modem-control setup on a pty.
 */
#define _GNU_SOURCE
#include <stdarg.h>
#include <sys/ioctl.h>
#include <dlfcn.h>
#include <asm/termbits.h>

int ioctl(int fd, unsigned long request, ...)
{
    static int (*real)(int, unsigned long, ...);
    if (!real)
        real = dlsym(RTLD_NEXT, "ioctl");

    va_list ap;
    va_start(ap, request);
    void *arg = va_arg(ap, void *);
    va_end(ap);

    switch (request) {
    case TIOCMGET:
        if (arg)
            *(int *)arg = 0; /* no modem lines asserted */
        return 0;
    case TIOCMBIS:
    case TIOCMBIC:
    case TIOCMSET:
        return 0; /* accept, no-op: a pty has no modem lines */
    default:
        return real(fd, request, arg);
    }
}
