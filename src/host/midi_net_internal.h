/*
 * midi_net_internal.h - Shared internal state for midi_net subsystem
 *
 * Not part of the public API. Only midi_net.c, midi_net_ipmidi.c,
 * midi_net_applemidi.c, and midi_net_mdns.c include this.
 */

#ifndef MIDI_NET_INTERNAL_H
#define MIDI_NET_INTERNAL_H

#include <stdint.h>
#include <pthread.h>
#include "midi_net.h"
#include "shadow_constants.h"

/* ============================================================================
 * USB-MIDI Code Index Numbers (CIN)
 * ============================================================================ */
#define CIN_SYSEX_START_CONT 0x04  /* SysEx starts or continues (3 bytes follow) */
#define CIN_SYSEX_END_1      0x05  /* SysEx ends with 1 byte (status byte only) */
#define CIN_SYSEX_END_2      0x06  /* SysEx ends with 2 bytes */
#define CIN_SYSEX_END_3      0x07  /* SysEx ends with 3 bytes */
#define CIN_NOTE_OFF         0x08
#define CIN_NOTE_ON          0x09
#define CIN_POLY_KEY_PRESS   0x0A
#define CIN_CONTROL_CHANGE   0x0B
#define CIN_PROGRAM_CHANGE   0x0C
#define CIN_CHANNEL_PRESS    0x0D
#define CIN_PITCH_BEND       0x0E
#define CIN_SINGLE_BYTE      0x0F  /* Real-time (F8..FF), 1-byte system common */

/* ============================================================================
 * AppleMIDI limits
 * ============================================================================ */
#define MIDI_NET_MAX_PEERS           4
#define MIDI_NET_SYSEX_SCRATCH       4096

/* ============================================================================
 * Internal state (owned by midi_net.c, shared with backends)
 * ============================================================================ */

typedef struct midi_net_peer_t {
    int      active;           /* 0=free, 1=ctrl-accepted, 2=evt-accepted, 3=established */
    uint32_t initiator_token;  /* From IN message */
    uint32_t ssrc;             /* Our SSRC assigned to this session */
    uint32_t peer_ssrc;        /* Peer's SSRC */
    uint32_t ck_count;         /* Number of CK exchanges completed */

    /* Peer address — captured from IN on control port */
    struct sockaddr_storage ctrl_addr;
    socklen_t               ctrl_addrlen;
    struct sockaddr_storage evt_addr;
    socklen_t               evt_addrlen;

    /* Per-peer SysEx accumulator for RTP-MIDI parser */
    uint8_t  sysex_buf[MIDI_NET_SYSEX_SCRATCH];
    uint32_t sysex_len;

    /* Parser running-status */
    uint8_t  last_status;

    /* Last activity — used to reap idle sessions */
    uint64_t last_activity_ms;
} midi_net_peer_t;

typedef struct midi_net_state_t {
    /* Configuration */
    uint32_t backend_flags;

    /* Public inject SHM reference (double indirection so we see SHM remap) */
    shadow_midi_inject_t **inject_shm_ptr;

    /* Optional logging callback (if NULL, unified_log is used) */
    void (*log_fn)(const char *msg);

    /* Thread */
    pthread_t thread;
    volatile int running;
    int         self_pipe[2];  /* Shutdown signal pipe */

    /* Sockets (shared across backends in one poll set) */
    int ipmidi_sock;
    int applemidi_ctrl_sock;
    int applemidi_evt_sock;
    int mdns_sock;

    /* ipMIDI parser state (running status) */
    uint8_t ipmidi_last_status;

    /* AppleMIDI sessions */
    midi_net_peer_t peers[MIDI_NET_MAX_PEERS];

    /* Stats */
    midi_net_stats_t stats;

    /* Outbound serialization */
    pthread_mutex_t outbound_mutex;
    uint16_t        applemidi_seq;      /* RTP sequence number for outbound */
    uint32_t        applemidi_timestamp;/* RTP timestamp base */
    uint32_t        our_ssrc;           /* Our SSRC (random) */
} midi_net_state_t;

extern midi_net_state_t g_midi_net;

/* ============================================================================
 * Logging helper
 * ============================================================================ */
void midi_net_log(const char *fmt, ...);
void midi_net_logd(const char *fmt, ...);

/* ============================================================================
 * Time helpers
 * ============================================================================ */
uint64_t midi_net_now_ms(void);
uint64_t midi_net_now_100us(void);  /* AppleMIDI clock ticks, 100 µs resolution */

/* ============================================================================
 * Shared injector — safely append a 4-byte USB-MIDI packet to the inject SHM.
 * Returns 1 on success, 0 on buffer-full. Increments stats.
 * ============================================================================ */
int midi_net_inject_usb_packet(uint8_t cin, uint8_t status, uint8_t d1, uint8_t d2);

/*
 * Emit a complete MIDI message (1-3 bytes) as a USB-MIDI packet.
 * Handles status/data byte packing, updates stats.
 * Returns number of injected USB-MIDI packets (normally 1, 0 if invalid).
 */
int midi_net_emit_midi_message(uint8_t status, uint8_t d1, uint8_t d2);

/*
 * Emit a SysEx byte stream (including F0/F7 framing) as a sequence
 * of USB-MIDI packets with proper CIN segmentation.
 * Returns number of packets emitted.
 */
int midi_net_emit_sysex(const uint8_t *bytes, int len);

/*
 * Walk a raw MIDI byte stream (channel messages + real-time interleaving +
 * running status) and emit USB-MIDI packets. Updates *last_status for
 * running status continuity across calls. Returns number of messages parsed.
 */
int midi_net_parse_raw_stream(const uint8_t *bytes, int len, uint8_t *last_status);

/* ============================================================================
 * Backend entry points
 * ============================================================================ */

/* ipMIDI */
int  midi_net_ipmidi_open(void);
void midi_net_ipmidi_close(void);
void midi_net_ipmidi_handle_rx(int sock);

/* AppleMIDI */
int  midi_net_applemidi_open(void);
void midi_net_applemidi_close(void);
void midi_net_applemidi_handle_ctrl(int sock);
void midi_net_applemidi_handle_evt(int sock);
void midi_net_applemidi_tick(void);      /* periodic reaping */
void midi_net_applemidi_send_midi(const uint8_t *pkt4);  /* outbound */

/* mDNS */
int  midi_net_mdns_open(void);
void midi_net_mdns_close(void);
void midi_net_mdns_handle_rx(int sock);
void midi_net_mdns_announce(void);       /* unsolicited broadcast */

#endif /* MIDI_NET_INTERNAL_H */
