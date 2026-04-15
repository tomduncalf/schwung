/*
 * midi_net.c - MIDI over WiFi core dispatcher
 *
 * Owns the background thread, socket poll loop, shared state, and
 * the packet injector that feeds Move's MIDI_IN buffer via
 * the existing /schwung-midi-inject shared memory.
 *
 * Backends (ipMIDI, AppleMIDI, mDNS, outbound) live in separate files
 * and are driven from the single poll() loop here.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/stat.h>

#include "midi_net.h"
#include "midi_net_internal.h"
#include "unified_log.h"

/* ============================================================================
 * Global state definition
 * ============================================================================ */

midi_net_state_t g_midi_net;

/* ============================================================================
 * Logging helpers
 * ============================================================================ */

void midi_net_log(const char *fmt, ...) {
    char msg[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    if (g_midi_net.log_fn) {
        g_midi_net.log_fn(msg);
    } else {
        LOG_INFO("midi_net", "%s", msg);
    }
}

void midi_net_logd(const char *fmt, ...) {
    char msg[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    /* Debug-level: only goes to unified log (which gates on debug_log_on flag) */
    LOG_DEBUG("midi_net", "%s", msg);
}

/* ============================================================================
 * Time helpers
 * ============================================================================ */

uint64_t midi_net_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)(ts.tv_nsec / 1000000);
}

uint64_t midi_net_now_100us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 10000 + (uint64_t)(ts.tv_nsec / 100000);
}

/* ============================================================================
 * Shared injector
 * ============================================================================ */

int midi_net_inject_usb_packet(uint8_t cin, uint8_t status, uint8_t d1, uint8_t d2) {
    if (!g_midi_net.inject_shm_ptr) return 0;
    shadow_midi_inject_t *shm = *g_midi_net.inject_shm_ptr;
    if (!shm) return 0;

    uint8_t idx = shm->write_idx;
    if ((int)idx + 4 > SHADOW_MIDI_INJECT_BUFFER_SIZE) {
        g_midi_net.stats.drops_buffer_full++;
        return 0;
    }

    shm->buffer[idx]     = cin & 0x0F;  /* cable 0 */
    shm->buffer[idx + 1] = status;
    shm->buffer[idx + 2] = d1;
    shm->buffer[idx + 3] = d2;

    __sync_synchronize();
    shm->write_idx = idx + 4;
    shm->ready++;

    g_midi_net.stats.rx_midi_messages++;
    return 1;
}

/* ============================================================================
 * MIDI message emitter (1-3 byte channel / system messages)
 * ============================================================================ */

int midi_net_emit_midi_message(uint8_t status, uint8_t d1, uint8_t d2) {
    if (!(status & 0x80)) return 0;

    uint8_t type = status & 0xF0;
    uint8_t cin;

    switch (type) {
        case 0x80: cin = CIN_NOTE_OFF; break;
        case 0x90: cin = CIN_NOTE_ON; break;
        case 0xA0: cin = CIN_POLY_KEY_PRESS; break;
        case 0xB0: cin = CIN_CONTROL_CHANGE; break;
        case 0xC0: cin = CIN_PROGRAM_CHANGE; d2 = 0; break;
        case 0xD0: cin = CIN_CHANNEL_PRESS; d2 = 0; break;
        case 0xE0: cin = CIN_PITCH_BEND; break;
        case 0xF0:
            /* System messages */
            if (status >= 0xF8) {
                /* Real-time single byte */
                cin = CIN_SINGLE_BYTE;
                d1 = 0;
                d2 = 0;
            } else if (status == 0xF1 || status == 0xF3) {
                /* MIDI Time Code / Song Select — 2 bytes */
                cin = 0x02;
                d2 = 0;
            } else if (status == 0xF2) {
                /* Song Position Pointer — 3 bytes */
                cin = 0x03;
            } else if (status == 0xF6 || status == 0xF7) {
                /* Tune Request, System Reset */
                cin = CIN_SINGLE_BYTE;
                d1 = 0;
                d2 = 0;
            } else {
                g_midi_net.stats.drops_parse_err++;
                return 0;
            }
            break;
        default:
            g_midi_net.stats.drops_parse_err++;
            return 0;
    }

    return midi_net_inject_usb_packet(cin, status, d1, d2);
}

/* ============================================================================
 * SysEx emitter — splits raw SysEx (F0...F7) into 3-byte USB-MIDI fragments
 * ============================================================================ */

int midi_net_emit_sysex(const uint8_t *bytes, int len) {
    if (len <= 0 || !bytes) return 0;

    int emitted = 0;
    int i = 0;
    while (i < len) {
        int remaining = len - i;
        if (remaining >= 4) {
            /* Fragment of 3 bytes, more to come */
            midi_net_inject_usb_packet(CIN_SYSEX_START_CONT,
                                       bytes[i], bytes[i + 1], bytes[i + 2]);
            emitted++;
            i += 3;
        } else if (remaining == 3) {
            /* Last 3 bytes */
            midi_net_inject_usb_packet(CIN_SYSEX_END_3,
                                       bytes[i], bytes[i + 1], bytes[i + 2]);
            emitted++;
            i += 3;
        } else if (remaining == 2) {
            midi_net_inject_usb_packet(CIN_SYSEX_END_2,
                                       bytes[i], bytes[i + 1], 0);
            emitted++;
            i += 2;
        } else /* remaining == 1 */ {
            midi_net_inject_usb_packet(CIN_SYSEX_END_1,
                                       bytes[i], 0, 0);
            emitted++;
            i += 1;
        }
    }
    return emitted;
}

/* ============================================================================
 * Raw MIDI byte-stream parser
 *
 * Walks a buffer of raw MIDI bytes that may contain:
 *   - Channel-voice messages (0x80..0xEF) with running-status
 *   - System common (0xF0..0xF7)
 *   - System real-time (0xF8..0xFF) that can interleave mid-message
 *   - SysEx (F0 ... F7) spanning arbitrary length
 *
 * *last_status is preserved across calls for continuity.
 * ============================================================================ */

static int expected_data_bytes(uint8_t status) {
    uint8_t type = status & 0xF0;
    switch (type) {
        case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0: return 2;
        case 0xC0: case 0xD0: return 1;
        default:
            if (status == 0xF2) return 2;
            if (status == 0xF1 || status == 0xF3) return 1;
            return 0;
    }
}

int midi_net_parse_raw_stream(const uint8_t *bytes, int len, uint8_t *last_status) {
    int messages = 0;
    int i = 0;
    uint8_t running = last_status ? *last_status : 0;
    uint8_t sysex_buf[1024];
    int     sysex_len = 0;
    int     in_sysex = 0;

    while (i < len) {
        uint8_t b = bytes[i];

        /* System real-time — always emit as standalone, never interrupts running status */
        if (b >= 0xF8) {
            midi_net_emit_midi_message(b, 0, 0);
            messages++;
            i++;
            continue;
        }

        if (in_sysex) {
            if (b == 0xF7) {
                if (sysex_len < (int)sizeof(sysex_buf)) sysex_buf[sysex_len++] = 0xF7;
                midi_net_emit_sysex(sysex_buf, sysex_len);
                messages++;
                sysex_len = 0;
                in_sysex = 0;
                i++;
                continue;
            }
            /* Any status byte other than F7 terminates SysEx ungracefully */
            if (b & 0x80) {
                /* Flush incomplete SysEx as-is */
                midi_net_emit_sysex(sysex_buf, sysex_len);
                sysex_len = 0;
                in_sysex = 0;
                /* Fall through to handle new status */
            } else {
                if (sysex_len < (int)sizeof(sysex_buf)) sysex_buf[sysex_len++] = b;
                i++;
                continue;
            }
        }

        /* SysEx start */
        if (b == 0xF0) {
            in_sysex = 1;
            sysex_buf[0] = 0xF0;
            sysex_len = 1;
            i++;
            continue;
        }

        /* Status byte */
        if (b & 0x80) {
            running = b;
            i++;
        } else if (running == 0) {
            /* Data byte with no running status — skip */
            g_midi_net.stats.drops_parse_err++;
            i++;
            continue;
        }

        /* Running or fresh status: collect data bytes */
        int need = expected_data_bytes(running);
        uint8_t d1 = 0, d2 = 0;

        if (need >= 1) {
            if (i >= len) break;  /* Incomplete at end of buffer */
            /* Real-time bytes can interleave even here */
            while (i < len && bytes[i] >= 0xF8) {
                midi_net_emit_midi_message(bytes[i], 0, 0);
                messages++;
                i++;
            }
            if (i >= len) break;
            d1 = bytes[i++];
        }
        if (need >= 2) {
            if (i >= len) break;
            while (i < len && bytes[i] >= 0xF8) {
                midi_net_emit_midi_message(bytes[i], 0, 0);
                messages++;
                i++;
            }
            if (i >= len) break;
            d2 = bytes[i++];
        }

        midi_net_emit_midi_message(running, d1, d2);
        messages++;
    }

    /* Incomplete SysEx at end of buffer: preserve state by not flushing */
    if (in_sysex && sysex_len > 0) {
        midi_net_emit_sysex(sysex_buf, sysex_len);
    }

    if (last_status) *last_status = running;
    return messages;
}

/* ============================================================================
 * Thread body
 * ============================================================================ */

static void *midi_net_thread_func(void *arg) {
    (void)arg;

    midi_net_log("midi_net: thread starting");

    /* Open backends */
    if (g_midi_net.backend_flags & MIDI_NET_BACKEND_IPMIDI) {
        if (midi_net_ipmidi_open() == 0) {
            midi_net_log("midi_net: ipMIDI listening on %s:%d",
                         MIDI_NET_IPMIDI_GROUP, MIDI_NET_IPMIDI_PORT);
        }
    }
    if (g_midi_net.backend_flags & MIDI_NET_BACKEND_APPLEMIDI) {
        if (midi_net_applemidi_open() == 0) {
            midi_net_log("midi_net: AppleMIDI listening on %d/%d",
                         MIDI_NET_APPLE_CTRL_PORT, MIDI_NET_APPLE_EVT_PORT);
        }
    }
    if (g_midi_net.backend_flags & MIDI_NET_BACKEND_MDNS) {
        if (midi_net_mdns_open() == 0) {
            midi_net_log("midi_net: mDNS responder active on %s:%d",
                         MIDI_NET_MDNS_GROUP, MIDI_NET_MDNS_PORT);
            midi_net_mdns_announce();
        }
    }

    uint64_t last_tick = midi_net_now_ms();
    uint64_t last_announce = last_tick;

    while (g_midi_net.running) {
        struct pollfd pfds[8];
        int nfds = 0;

        /* Always poll the shutdown pipe */
        pfds[nfds].fd = g_midi_net.self_pipe[0];
        pfds[nfds].events = POLLIN;
        nfds++;

        int idx_ipmidi = -1;
        int idx_ctrl = -1;
        int idx_evt = -1;
        int idx_mdns = -1;

        if (g_midi_net.ipmidi_sock >= 0) {
            pfds[nfds].fd = g_midi_net.ipmidi_sock;
            pfds[nfds].events = POLLIN;
            idx_ipmidi = nfds++;
        }
        if (g_midi_net.applemidi_ctrl_sock >= 0) {
            pfds[nfds].fd = g_midi_net.applemidi_ctrl_sock;
            pfds[nfds].events = POLLIN;
            idx_ctrl = nfds++;
        }
        if (g_midi_net.applemidi_evt_sock >= 0) {
            pfds[nfds].fd = g_midi_net.applemidi_evt_sock;
            pfds[nfds].events = POLLIN;
            idx_evt = nfds++;
        }
        if (g_midi_net.mdns_sock >= 0) {
            pfds[nfds].fd = g_midi_net.mdns_sock;
            pfds[nfds].events = POLLIN;
            idx_mdns = nfds++;
        }

        int pr = poll(pfds, nfds, 250);
        if (pr < 0) {
            if (errno == EINTR) continue;
            midi_net_log("midi_net: poll error: %s", strerror(errno));
            struct timespec ts = { 1, 0 };
            nanosleep(&ts, NULL);
            continue;
        }

        /* Shutdown pipe */
        if (pfds[0].revents & POLLIN) {
            char drain[16];
            while (read(g_midi_net.self_pipe[0], drain, sizeof(drain)) > 0) { }
            if (!g_midi_net.running) break;
        }

        if (idx_ipmidi >= 0 && (pfds[idx_ipmidi].revents & POLLIN)) {
            midi_net_ipmidi_handle_rx(g_midi_net.ipmidi_sock);
        }
        if (idx_ctrl >= 0 && (pfds[idx_ctrl].revents & POLLIN)) {
            midi_net_applemidi_handle_ctrl(g_midi_net.applemidi_ctrl_sock);
        }
        if (idx_evt >= 0 && (pfds[idx_evt].revents & POLLIN)) {
            midi_net_applemidi_handle_evt(g_midi_net.applemidi_evt_sock);
        }
        if (idx_mdns >= 0 && (pfds[idx_mdns].revents & POLLIN)) {
            midi_net_mdns_handle_rx(g_midi_net.mdns_sock);
        }

        /* Periodic housekeeping */
        uint64_t now = midi_net_now_ms();
        if (now - last_tick >= 1000) {
            last_tick = now;
            if (g_midi_net.backend_flags & MIDI_NET_BACKEND_APPLEMIDI) {
                midi_net_applemidi_tick();
            }
        }
        if (g_midi_net.backend_flags & MIDI_NET_BACKEND_MDNS) {
            if (now - last_announce >= 120000) {
                last_announce = now;
                midi_net_mdns_announce();
            }
        }
    }

    /* Cleanup */
    midi_net_ipmidi_close();
    midi_net_applemidi_close();
    midi_net_mdns_close();

    midi_net_log("midi_net: thread exiting");
    return NULL;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void midi_net_init(shadow_midi_inject_t **inject_shm_ptr,
                   void (*log_fn)(const char *msg)) {
    memset(&g_midi_net, 0, sizeof(g_midi_net));
    g_midi_net.inject_shm_ptr = inject_shm_ptr;
    g_midi_net.log_fn = log_fn;
    g_midi_net.backend_flags = MIDI_NET_BACKEND_ALL;
    g_midi_net.ipmidi_sock = -1;
    g_midi_net.applemidi_ctrl_sock = -1;
    g_midi_net.applemidi_evt_sock = -1;
    g_midi_net.mdns_sock = -1;
    g_midi_net.self_pipe[0] = -1;
    g_midi_net.self_pipe[1] = -1;
    pthread_mutex_init(&g_midi_net.outbound_mutex, NULL);

    /* Random SSRC for AppleMIDI sessions — seed from time */
    g_midi_net.our_ssrc = (uint32_t)midi_net_now_ms() * 0x9E3779B1u;
}

void midi_net_set_backends(uint32_t backend_flags) {
    g_midi_net.backend_flags = backend_flags;
}

int midi_net_start(void) {
    if (g_midi_net.running) return 0;

    if (pipe(g_midi_net.self_pipe) != 0) {
        midi_net_log("midi_net: self-pipe create failed: %s", strerror(errno));
        return -1;
    }
    int fl0 = fcntl(g_midi_net.self_pipe[0], F_GETFL, 0);
    fcntl(g_midi_net.self_pipe[0], F_SETFL, fl0 | O_NONBLOCK);
    int fl1 = fcntl(g_midi_net.self_pipe[1], F_GETFL, 0);
    fcntl(g_midi_net.self_pipe[1], F_SETFL, fl1 | O_NONBLOCK);

    g_midi_net.running = 1;
    if (pthread_create(&g_midi_net.thread, NULL, midi_net_thread_func, NULL) != 0) {
        midi_net_log("midi_net: pthread_create failed: %s", strerror(errno));
        g_midi_net.running = 0;
        close(g_midi_net.self_pipe[0]); g_midi_net.self_pipe[0] = -1;
        close(g_midi_net.self_pipe[1]); g_midi_net.self_pipe[1] = -1;
        return -1;
    }
    return 0;
}

void midi_net_stop(void) {
    if (!g_midi_net.running) return;

    g_midi_net.running = 0;
    if (g_midi_net.self_pipe[1] >= 0) {
        char b = 1;
        ssize_t wn = write(g_midi_net.self_pipe[1], &b, 1);
        (void)wn;
    }

    /* Best-effort timed join */
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += 1;
#ifdef __GLIBC__
    int rc = pthread_timedjoin_np(g_midi_net.thread, NULL, &deadline);
    if (rc != 0) {
        midi_net_log("midi_net: thread join timeout, detaching");
        pthread_detach(g_midi_net.thread);
    }
#else
    pthread_join(g_midi_net.thread, NULL);
#endif

    if (g_midi_net.self_pipe[0] >= 0) { close(g_midi_net.self_pipe[0]); g_midi_net.self_pipe[0] = -1; }
    if (g_midi_net.self_pipe[1] >= 0) { close(g_midi_net.self_pipe[1]); g_midi_net.self_pipe[1] = -1; }
}

int midi_net_is_running(void) {
    return g_midi_net.running;
}

int midi_net_flag_enabled(void) {
    static uint64_t last_check_ms = 0;
    static int cached = 0;
    uint64_t now = midi_net_now_ms();
    if (now - last_check_ms >= 1000) {
        last_check_ms = now;
        struct stat st;
        cached = (stat(MIDI_NET_FLAG_FILE, &st) == 0) ? 1 : 0;
    }
    return cached;
}

void midi_net_publish(const uint8_t *pkt4) {
    if (!pkt4) return;
    if (!(g_midi_net.backend_flags & MIDI_NET_BACKEND_OUTBOUND)) return;
    if (!g_midi_net.running) return;

    pthread_mutex_lock(&g_midi_net.outbound_mutex);
    if (g_midi_net.backend_flags & MIDI_NET_BACKEND_APPLEMIDI) {
        midi_net_applemidi_send_midi(pkt4);
    }
    pthread_mutex_unlock(&g_midi_net.outbound_mutex);
}

void midi_net_get_stats(midi_net_stats_t *out) {
    if (out) *out = g_midi_net.stats;
}
